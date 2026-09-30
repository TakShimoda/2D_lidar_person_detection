import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker

from dr_spaam.detector import Detector


class DrSpaamROS2(Node):
    """ROS2 node to detect pedestrians using DROW3 or DR-SPAAM."""

    def __init__(self):
        super().__init__("dr_spaam_ros2_node")

        # Declare parameters
        self.declare_parameter("weight_file", "")
        self.declare_parameter("detector_model", "DR-SPAAM")
        self.declare_parameter("conf_thresh", 0.8)
        self.declare_parameter("stride", 1)
        self.declare_parameter("panoramic_scan", True)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("gpu", True)

        # Get parameters
        self.weight_file = self.get_parameter("weight_file").get_parameter_value().string_value
        self.detector_model = self.get_parameter("detector_model").get_parameter_value().string_value
        self.conf_thresh = self.get_parameter("conf_thresh").get_parameter_value().double_value
        self.stride = self.get_parameter("stride").get_parameter_value().integer_value
        self.panoramic_scan = self.get_parameter("panoramic_scan").get_parameter_value().bool_value
        self.scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.use_gpu = self.get_parameter("gpu").get_parameter_value().bool_value

        self.get_logger().info(f"Initializing detector '{self.detector_model}' with checkpoint: '{self.weight_file}'")

        if not self.weight_file:
            self.get_logger().error("Parameter 'weight_file' is empty! Please provide a path to a valid checkpoint.")
            raise ValueError("weight_file parameter is empty.")

        # Initialize detector
        self._detector = Detector(
            self.weight_file,
            model=self.detector_model,
            gpu=self.use_gpu,
            stride=self.stride,
            panoramic_scan=self.panoramic_scan,
        )

        # Create Publisher
        self._dets_pub = self.create_publisher(PoseArray, "~/detections", 10)
        self._rviz_pub = self.create_publisher(Marker, "~/rviz_marker", 10)
        self._inst_pub = self.create_publisher(PointCloud2, "~/instance_points", 10)

        # Create Subscriber
        self._scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._scan_callback,
            10
        )

        self.get_logger().info(f"Node initialized. Subscribed to topic: {self.scan_topic}")

    def _scan_callback(self, msg):
        # Configure FOV on the first received scan message
        if not self._detector.is_ready():
            fov_rad = msg.angle_increment * len(msg.ranges)
            self._detector.set_laser_fov(np.rad2deg(fov_rad))
            self.get_logger().info(f"Dynamic LiDAR FOV configured to: {np.rad2deg(fov_rad):.2f} degrees")

        scan = np.array(msg.ranges)
        # Preprocess scan: replace out-of-range points with maximum background range
        scan[scan < msg.range_min] = 29.99
        scan[scan > msg.range_max] = 29.99
        scan[np.isinf(scan)] = 29.99
        scan[np.isnan(scan)] = 29.99

        # Compute exact angles for each ray based on LaserScan metadata
        scan_phi = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment

        # Run inference using the true scan angles
        dets_xy, dets_cls, instance_mask = self._detector(scan, scan_phi=scan_phi)

        # Apply confidence threshold filter. Keep the pre-filter indices: the
        # j-th detection the detector returned owns the scan points marked
        # j + 1 in instance_mask, so the mapping has to be taken before the
        # surviving detections are renumbered.
        conf_mask = (dets_cls >= self.conf_thresh).reshape(-1)
        kept_inds = np.flatnonzero(conf_mask)
        dets_xy = dets_xy[conf_mask]
        dets_cls = dets_cls[conf_mask]

        # Convert to geometry_msgs/PoseArray
        dets_msg = self.detections_to_pose_array(dets_xy, dets_cls)
        dets_msg.header = msg.header
        self._dets_pub.publish(dets_msg)

        # Convert to visualization_msgs/Marker
        rviz_msg = self.detections_to_rviz_marker(dets_xy, dets_cls)
        rviz_msg.header = msg.header
        self._rviz_pub.publish(rviz_msg)

        # The scan points backing each detection, as sensor_msgs/PointCloud2
        inst_msg = self.instances_to_point_cloud(
            scan, scan_phi, instance_mask, kept_inds, msg.header)
        self._inst_pub.publish(inst_msg)

    def instances_to_point_cloud(self, scan, scan_phi, instance_mask, kept_inds, header):
        """The scan points that voted for each surviving detection.

        DR-SPAAM regresses a centre offset per point and groups the votes by
        NMS; instance_mask records which cluster each point fell into. The
        detector itself only returns the grouped centres, so this is the one
        place the per-person point membership is visible -- useful for
        estimating orientation, which the network does not predict.

        The `instance` field indexes the detections published on ~/detections,
        so instance == i belongs to poses[i]. Points in no surviving cluster
        are left out.
        """
        # instance_mask is indexed over the strided scan, and its ids run in
        # descending-confidence order across every NMS cluster -- including the
        # many below conf_thresh. Map the surviving ones onto 0..M-1.
        scan_s = scan[:: self.stride]
        phi_s = scan_phi[:: self.stride]
        remap = np.full(int(instance_mask.max()) + 2, -1, dtype=np.int32)
        remap[kept_inds + 1] = np.arange(len(kept_inds), dtype=np.int32)

        ids = remap[np.clip(instance_mask, 0, len(remap) - 1)]
        sel = ids >= 0

        points = []
        if np.any(sel):
            r, phi, idx = scan_s[sel], phi_s[sel], ids[sel]
            xs, ys = r * np.cos(phi), r * np.sin(phi)
            points = [(float(x), float(y), 0.0, float(i))
                      for x, y, i in zip(xs, ys, idx)]

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            # float32 rather than an integer type so RViz's intensity colour
            # transformer can shade points by which person they belong to.
            PointField(name="instance", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        return point_cloud2.create_cloud(header, fields, points)

    def detections_to_pose_array(self, dets_xy, dets_cls):
        pose_array = PoseArray()
        for d_xy, d_cls in zip(dets_xy, dets_cls):
            p = Pose()
            p.position.x = float(d_xy[0])
            p.position.y = float(d_xy[1])
            p.position.z = 0.0
            pose_array.poses.append(p)
        return pose_array

    def detections_to_rviz_marker(self, dets_xy, dets_cls):
        msg = Marker()
        msg.action = Marker.ADD
        msg.ns = "dr_spaam_ros2"
        msg.id = 0
        msg.type = Marker.LINE_LIST

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        msg.scale.x = 0.03  # Line width
        msg.color.r = 1.0
        msg.color.g = 0.0
        msg.color.b = 0.0
        msg.color.a = 1.0

        # Approximating a circle around each pedestrian with 20 line segments
        r = 0.4  # Radius of 0.4 meters
        ang = np.linspace(0, 2 * np.pi, 20)
        xy_offsets = r * np.stack((np.cos(ang), np.sin(ang)), axis=1)

        for d_xy, d_cls in zip(dets_xy, dets_cls):
            for i in range(len(xy_offsets) - 1):
                p0 = Point()
                p0.x = float(d_xy[0] + xy_offsets[i, 0])
                p0.y = float(d_xy[1] + xy_offsets[i, 1])
                p0.z = 0.0
                msg.points.append(p0)

                p1 = Point()
                p1.x = float(d_xy[0] + xy_offsets[i + 1, 0])
                p1.y = float(d_xy[1] + xy_offsets[i + 1, 1])
                p1.z = 0.0
                msg.points.append(p1)

        return msg


def main(args=None):
    rclpy.init(args=args)
    try:
        node = DrSpaamROS2()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error starting node: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
