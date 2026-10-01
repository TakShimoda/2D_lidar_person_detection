"""Stream a JRDB sequence as a 2D scan, from the Velodyne point clouds.

Two modes:

  --mode slice       (default) collapse a horizontal band of the point cloud
                     into a sensor_msgs/LaserScan here, in Python.

  --mode pointcloud  republish the raw Velodyne as sensor_msgs/PointCloud2 with
                     the `ring` field intact, and let the velodyne_laserscan
                     node pull out a single ring. This is the mode that matches
                     how you would run a real VLP-16:

                       python3 tools/jrdb_scan_publisher.py --mode pointcloud --loop
                       ros2 run velodyne_laserscan velodyne_laserscan_node --ros-args \
                         -p ring:=8 -p resolution:=0.00576 \
                         -r velodyne_points:=/velodyne_points -r scan:=/scan

                     Measured on bytes-cafe-2019-02-07_0 (160 frames, conf 0.8),
                     a single ring recovers the range the slice throws away:

                       slice      recall 0.373  precision 0.782  >4 m 0.015
                       ring 8     recall 0.385  precision 0.689  >4 m 0.191

                     The slice is better close in and the ring is far better
                     past 4 m, because a ring is a cone centred on the sensor
                     while a slice cuts across every cone -- see the README.

    source dr_spaam_env.sh
    python3 tools/jrdb_scan_publisher.py --sequence bytes-cafe-2019-02-07_0

JRDB ships a real 2D laser (the `lasers/` archive, one .txt of ranges per frame),
and that is what DR-SPAAM was trained and evaluated on. If you have it, prefer it.
This script is for the case where only `pointclouds/` was downloaded: it slices a
thin horizontal band out of the 3D point cloud at the height of the 2D laser and
bins it into the same 1091-ray panoramic grid.

The slice is an approximation, not the real sensor -- see the README for measured
recall/precision. It is good enough to get true positives on real pedestrians and
to exercise the pipeline end to end.

Geometry (all constants from dr_spaam.utils.jrdb_transforms and jrdb_handle):
  - lower velodyne origin sits at base z = -0.13511, upper at +0.33529
  - the 2D laser plane is at base z = -0.5
  - laser frame is the base frame rotated pi/120 about z
"""
import argparse
import glob
import os

import array

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField

import dr_spaam.utils.jrdb_transforms as jt
from dr_spaam.datahandle._pypcd import point_cloud_from_path

NUM_RAYS = 1091          # JRDB's 2D laser ray count, what the checkpoints expect
LASER_Z = -0.5           # height of the 2D laser plane in base frame
PADDING_VAL = 29.99      # "no return", matching the detector's padding value


def load_velodyne_in_base(root, sequence, which, frame):
    path = os.path.join(root, "pointclouds", which + "_velodyne", sequence,
                        "{:06d}.pcd".format(frame))
    pc = point_cloud_from_path(path).pc_data
    xyz = np.stack([pc["x"], pc["y"], pc["z"]], axis=0).astype(np.float32)
    xyz = xyz[:, np.isfinite(xyz).all(axis=0)]
    if which == "lower":
        return jt.transform_pts_lower_velodyne_to_base(xyz)
    return jt.transform_pts_upper_velodyne_to_base(xyz)


def load_velodyne_raw(root, sequence, which, frame):
    """Raw Velodyne points in the SENSOR frame, with the ring index.

    velodyne_laserscan measures range from the cloud's own origin, so the cloud
    has to stay in the sensor frame -- transforming it to base would move the
    origin and corrupt every range.
    """
    path = os.path.join(root, "pointclouds", which + "_velodyne", sequence,
                        "{:06d}.pcd".format(frame))
    pc = point_cloud_from_path(path).pc_data
    xyz = np.stack([pc["x"], pc["y"], pc["z"]], axis=0).astype(np.float32)
    inten = np.asarray(pc["intensity"], dtype=np.float32)
    ring = np.asarray(pc["ring"], dtype=np.uint16)
    keep = np.isfinite(xyz).all(axis=0)
    return xyz[:, keep], inten[keep], ring[keep]


# Packed layout; PointCloud2 addresses fields by offset so no padding is needed.
_PC_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                      ("intensity", "<f4"), ("ring", "<u2")])
_PC_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    # velodyne_laserscan requires a UINT16 field named exactly "ring".
    PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
]


def make_point_cloud(xyz, inten, ring, frame_id, stamp):
    rec = np.empty(xyz.shape[1], dtype=_PC_DTYPE)
    rec["x"], rec["y"], rec["z"] = xyz[0], xyz[1], xyz[2]
    rec["intensity"], rec["ring"] = inten, ring

    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = rec.shape[0]
    msg.fields = _PC_FIELDS
    msg.is_bigendian = False
    msg.point_step = _PC_DTYPE.itemsize
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    # array.array, not bytes: rclpy validates sequence fields element by element
    # otherwise, which costs ~100 ms for a cloud this size.
    msg.data = array.array("B", rec.tobytes())
    return msg


def pointcloud_to_scan(root, sequence, frame, band, use_upper):
    """Slice the point cloud at the laser plane and bin it into NUM_RAYS ranges."""
    clouds = [load_velodyne_in_base(root, sequence, "lower", frame)]
    if use_upper:
        clouds.append(load_velodyne_in_base(root, sequence, "upper", frame))
    pts = np.concatenate(clouds, axis=1)

    pts = pts[:, np.abs(pts[2] - LASER_Z) <= band]
    pts = jt.transform_pts_base_to_laser(pts)

    ranges = np.hypot(pts[0], pts[1])
    phi = np.arctan2(pts[1], pts[0])
    # Bin onto the same grid the detector assumes: linspace(-pi, pi, NUM_RAYS).
    inds = np.rint((phi + np.pi) / (2.0 * np.pi) * (NUM_RAYS - 1)).astype(int)
    inds = np.clip(inds, 0, NUM_RAYS - 1)

    scan = np.full(NUM_RAYS, PADDING_VAL, dtype=np.float32)
    np.minimum.at(scan, inds, ranges)   # nearest return wins, as a real laser does
    return scan


class JrdbScanPublisher(Node):
    def __init__(self, args, frames):
        super().__init__("jrdb_scan_publisher")
        self._args = args
        self._frames = frames
        self._i = 0
        if args.mode == "pointcloud":
            self._pub = self.create_publisher(PointCloud2, args.cloud_topic, 10)
            self.get_logger().info(
                "{}: {} frames, raw {} velodyne as PointCloud2 on {} "
                "(frame '{}'), {} Hz".format(
                    args.sequence, len(frames), args.which, args.cloud_topic,
                    args.cloud_frame_id, args.rate))
            self.get_logger().info(
                "run velodyne_laserscan to pull out a ring, e.g. ring 8 (+1 deg):"
                "  ros2 run velodyne_laserscan velodyne_laserscan_node --ros-args "
                "-p ring:=8 -p resolution:=0.00576 "
                "-r velodyne_points:={} -r scan:=/scan".format(args.cloud_topic))
        else:
            self._pub = self.create_publisher(LaserScan, args.topic, 10)
            self.get_logger().info(
                "{}: {} frames, band +-{:.2f} m, upper={}, {} Hz on {}".format(
                    args.sequence, len(frames), args.band, args.use_upper,
                    args.rate, args.topic))
        self.create_timer(1.0 / args.rate, self._tick)

    def _tick(self):
        if self._i >= len(self._frames):
            if not self._args.loop:
                self.get_logger().info("sequence finished")
                raise SystemExit(0)
            self._i = 0
        frame = self._frames[self._i]
        self._i += 1
        a = self._args

        if a.mode == "pointcloud":
            xyz, inten, ring = load_velodyne_raw(a.root, a.sequence, a.which, frame)
            self._pub.publish(make_point_cloud(
                xyz, inten, ring, a.cloud_frame_id, self.get_clock().now().to_msg()))
            if frame % 50 == 0:
                self.get_logger().info("frame {:06d}  {} points, rings {}-{}".format(
                    frame, xyz.shape[1], int(ring.min()), int(ring.max())))
            return

        scan = pointcloud_to_scan(a.root, a.sequence, frame, a.band, a.use_upper)

        msg = LaserScan()
        msg.header.frame_id = self._args.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle_min = -np.pi
        msg.angle_max = np.pi
        # NUM_RAYS samples spanning [-pi, pi] inclusive, so NUM_RAYS-1 intervals.
        msg.angle_increment = 2.0 * np.pi / (NUM_RAYS - 1)
        msg.range_min = 0.0
        msg.range_max = 30.0
        msg.ranges = scan.tolist()
        self._pub.publish(msg)

        if frame % 50 == 0:
            hits = int((scan < PADDING_VAL).sum())
            self.get_logger().info("frame {:06d}  {}/{} rays with a return".format(
                frame, hits, NUM_RAYS))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--root", default=os.path.expanduser(
        "~/Documents/JRDB/train_dataset_with_activity"))
    p.add_argument("--sequence", default="bytes-cafe-2019-02-07_0")
    p.add_argument("--band", type=float, default=0.10,
                   help="half-thickness of the slice, metres (default 0.10)")
    p.add_argument("--use-upper", action="store_true",
                   help="also slice the upper velodyne (measured slightly worse)")
    p.add_argument("--rate", type=float, default=7.5, help="Hz (JRDB records ~7.5)")
    p.add_argument("--start", type=int, default=0)
    p.add_argument("--count", type=int, default=0, help="0 = to the end")
    p.add_argument("--loop", action="store_true")
    p.add_argument("--topic", default="/scan")
    p.add_argument("--frame-id", default="base_link",
                   help="frame_id for the LaserScan in slice mode")
    p.add_argument("--mode", choices=("slice", "pointcloud"), default="slice",
                   help="slice: collapse a height band to a LaserScan here. "
                        "pointcloud: republish the raw Velodyne so "
                        "velodyne_laserscan can extract one ring (default slice)")
    p.add_argument("--which", choices=("lower", "upper"), default="lower",
                   help="which Velodyne to republish in pointcloud mode")
    p.add_argument("--cloud-topic", default="/velodyne_points")
    p.add_argument("--cloud-frame-id", default="velodyne",
                   help="frame_id for the cloud; it stays in the SENSOR frame, "
                        "because velodyne_laserscan measures range from the origin")
    args = p.parse_args()

    seq_dir = os.path.join(args.root, "pointclouds", "lower_velodyne", args.sequence)
    if not os.path.isdir(seq_dir):
        raise SystemExit("no such sequence: " + seq_dir)
    n = len(glob.glob(os.path.join(seq_dir, "*.pcd")))
    end = n if args.count <= 0 else min(n, args.start + args.count)
    frames = list(range(args.start, end))
    if not frames:
        raise SystemExit("no frames selected")

    rclpy.init()
    node = JrdbScanPublisher(args, frames)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
