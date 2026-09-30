"""Stream a JRDB sequence as sensor_msgs/LaserScan, synthesised from the Velodyne
point clouds.

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

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

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
        self._pub = self.create_publisher(LaserScan, args.topic, 10)
        self.create_timer(1.0 / args.rate, self._tick)
        self.get_logger().info(
            "{}: {} frames, band +-{:.2f} m, upper={}, {} Hz on {}".format(
                args.sequence, len(frames), args.band, args.use_upper,
                args.rate, args.topic))

    def _tick(self):
        if self._i >= len(self._frames):
            if not self._args.loop:
                self.get_logger().info("sequence finished")
                raise SystemExit(0)
            self._i = 0
        frame = self._frames[self._i]
        self._i += 1

        scan = pointcloud_to_scan(self._args.root, self._args.sequence, frame,
                                  self._args.band, self._args.use_upper)

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
    p.add_argument("--frame-id", default="base_link")
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
