"""Publish a synthetic sensor_msgs/LaserScan so the detector can be smoke-tested
without hardware or a bag.

    source dr_spaam_env.sh
    python3 tools/fake_scan_publisher.py

The defaults match a JRDB-style panoramic scan (1091 rays over 360 degrees), which
is what the pretrained DR-SPAAM checkpoints expect. The ranges are a smooth sine,
so the detector will usually find nothing -- the point is to exercise the callback
path and measure throughput, not to produce detections.
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FakeScanPublisher(Node):
    def __init__(self, topic, num_pts, rate_hz, frame_id):
        super().__init__("fake_scan_publisher")
        self._num_pts = num_pts
        self._frame_id = frame_id
        self._pub = self.create_publisher(LaserScan, topic, 10)
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            "publishing {} rays on {} at {} Hz".format(num_pts, topic, rate_hz)
        )

    def _tick(self):
        msg = LaserScan()
        msg.header.frame_id = self._frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 2.0 * math.pi / self._num_pts
        msg.range_min = 0.05
        msg.range_max = 30.0
        msg.ranges = [3.0 + 0.5 * math.sin(i / 40.0) for i in range(self._num_pts)]
        self._pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/scan")
    parser.add_argument("--num-pts", type=int, default=1091)
    parser.add_argument("--rate", type=float, default=5.0, help="Hz")
    parser.add_argument("--frame-id", default="base_link")
    args = parser.parse_args()

    rclpy.init()
    node = FakeScanPublisher(args.topic, args.num_pts, args.rate, args.frame_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
