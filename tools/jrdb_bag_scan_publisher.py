"""Replay the real 2D laser -- and optionally a camera -- from a JRDB rosbag.

    source dr_spaam_env.sh
    python3 tools/jrdb_bag_scan_publisher.py ~/Documents/JRDB/train_dataset_with_activity/rosbags/bytes-cafe-2019-02-07_0.bag

This reads `segway/scan_multi` -- the merged 360 degree, 1091-ray scan from the
robot's two SICK lasers, and the sensor DR-SPAAM was trained on -- straight out of
the ROS 1 bag and republishes it as a ROS 2 sensor_msgs/LaserScan. No conversion
step, and no approximation: unlike tools/jrdb_scan_publisher.py, which slices the
Velodyne point cloud, these are the laser's own measurements.

Reading ROS 1 bags uses the pure-Python `rosbags` package, so no ROS 1 install is
needed (the upstream bin/setup_jrdb_dataset.py does `import rosbag`, which is the
ROS 1 API and is unavailable on Humble):

    pip install rosbags

By default playback follows the bag's own timestamps. Use --hz to force a fixed
rate instead, or --rate to speed up or slow down.

With --camera N the matching camera stream is replayed alongside the scan. The
two are NOT frame-synchronised in the bag -- the laser runs at 15 Hz and the
cameras at 14 Hz, on independent triggers -- so there is no pairing to preserve.
What there is, is one shared bag clock, and this player replays both streams
against it, so their true relative timing is reproduced. Nearest-image-to-scan
offset measured on bytes-cafe-2019-02-07_0: median 16.6 ms, p95 36.5 ms, max
52.1 ms, with no drift across the sequence. That is the floor imposed by 14 Hz
sampling (half a period is 35.7 ms), not slop introduced here.

Camera N maps to bag topic ros_indigosdk_node/imageN/compressed and to
`cameras.yaml: sensor_N`. The five cameras that the dataset ships as
images/image_0 ... image_8 are the even-numbered ones.
"""
import argparse
import array
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan

DEFAULT_TOPIC_IN = "segway/scan_multi"
IMAGE_TOPIC_FMT = "ros_indigosdk_node/image{}/compressed"


def load_bag(bag_path, scan_topic, image_topic):
    """Read the scan topic, and the image topic when given.

    Returns a single list of (bag_time_seconds, kind, payload) events in bag
    order, so replaying it reproduces the real interleaving of the two streams.
    """
    from pathlib import Path

    from rosbags.highlevel import AnyReader

    wanted = {scan_topic}
    if image_topic:
        wanted.add(image_topic)

    events = []
    with AnyReader([Path(bag_path)]) as reader:
        present = {c.topic for c in reader.connections}
        for t in wanted:
            if t not in present:
                kind = "LaserScan" if t == scan_topic else "CompressedImage"
                avail = sorted({c.topic for c in reader.connections
                                if c.msgtype.endswith(kind)})
                raise SystemExit(
                    "topic {!r} not in bag. {} topics present: {}".format(
                        t, kind, ", ".join(avail) or "(none)"))

        conns = [c for c in reader.connections if c.topic in wanted]
        for conn, timestamp, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            t = timestamp * 1e-9              # bag timestamps are nanoseconds
            if conn.topic == scan_topic:
                events.append((t, "scan", {
                    "frame_id": m.header.frame_id,
                    "angle_min": float(m.angle_min),
                    "angle_max": float(m.angle_max),
                    "angle_increment": float(m.angle_increment),
                    "range_min": float(m.range_min),
                    "range_max": float(m.range_max),
                    "ranges": np.asarray(m.ranges, dtype=np.float32),
                }))
            else:
                # Keep the JPEG bytes; decoding 1800 frames up front would cost
                # ~1 GB of RAM, and cv2.imdecode is well under a millisecond.
                events.append((t, "image", {
                    "frame_id": m.header.frame_id,
                    "data": bytes(m.data),
                }))

    events.sort(key=lambda e: e[0])
    return events


def pair_by_timestamp(events):
    """Pair each scan with its nearest-in-time image.

    Returns events of kind "pair", scheduled on the scan's own bag time, each
    carrying both payloads so they can be published with one shared stamp.

    This does NOT bring the two closer together in time -- the nearest image is
    as far away as it is (median 16.6 ms, max 52.1 ms on bytes-cafe), and no
    choice of pairing changes that. What it buys is a deterministic 1:1
    association and identical header stamps, so downstream consumers treat a
    scan and an image as one observation.
    """
    scans = [(t, d) for t, k, d in events if k == "scan"]
    imgs = [(t, d) for t, k, d in events if k == "image"]
    if not imgs:
        return events, None

    itimes = np.array([t for t, _ in imgs])
    out, used = [], set()
    dts = []
    for t, d in scans:
        j = int(np.abs(itimes - t).argmin())
        used.add(j)
        dts.append(abs(itimes[j] - t))
        out.append((t, "pair", (d, imgs[j][1])))
    stats = {
        "pairs": len(out),
        "images_total": len(imgs),
        "images_used": len(used),
        "median_ms": float(np.median(dts) * 1000),
        "max_ms": float(np.max(dts) * 1000),
    }
    return out, stats


class BagScanPublisher(Node):
    def __init__(self, args, events, pair_stats=None):
        super().__init__("jrdb_bag_scan_publisher")
        self._args = args
        self._events = events
        self._pair_stats = pair_stats
        self._i = 0
        self._n_scan = sum(1 for e in events if e[1] in ("scan", "pair"))
        self._n_img = sum(1 for e in events if e[1] in ("image", "pair"))

        self._pub = self.create_publisher(LaserScan, args.topic, 10)
        self._img_pub = (self.create_publisher(Image, args.image_topic, 10)
                         if self._n_img else None)
        if self._img_pub is not None:
            import cv2
            self._cv2 = cv2

        t = np.array([e[0] for e in events])
        if args.hz:
            # Fixed rate refers to the SCAN rate; images keep their relative
            # spacing because the whole timeline is scaled by one factor.
            scan_t = np.array([e[0] for e in events if e[1] in ("scan", "pair")])
            scale = (1.0 / (np.median(np.diff(scan_t)) * args.hz)) if len(scan_t) > 1 else 1.0
        else:
            scale = 1.0 / args.rate
        # Cumulative offsets from playback start. Per-event gaps are clamped so a
        # recording gap does not stall playback, then accumulated -- scheduling
        # against absolute offsets keeps the two streams from drifting apart.
        gaps = np.clip(np.diff(t, prepend=t[0]) * scale, 0.0, 1.0)
        self._sched = np.cumsum(gaps)
        self._period = float(np.median(gaps[gaps > 0])) if (gaps > 0).any() else 0.03

        s0 = next((e[2][0] if e[1] == "pair" else e[2])
                  for e in events if e[1] in ("scan", "pair"))
        self.get_logger().info(
            "{}: {} scans, {} rays, {:.2f} deg, frame_id '{}'".format(
                os.path.basename(args.bag), self._n_scan, len(s0["ranges"]),
                np.rad2deg(s0["angle_increment"] * (len(s0["ranges"]) - 1)),
                s0["frame_id"]))
        if self._n_img and pair_stats:
            self.get_logger().info(
                "camera {}: timestamp-matched, {} pairs from {} images "
                "({} used, {} never shown); pair offset median {:.1f} ms, "
                "max {:.1f} ms -- matching fixes the pairing, it cannot close "
                "the gap".format(
                    args.camera, pair_stats["pairs"], pair_stats["images_total"],
                    pair_stats["images_used"],
                    pair_stats["images_total"] - pair_stats["images_used"],
                    pair_stats["median_ms"], pair_stats["max_ms"]))
        elif self._n_img:
            self.get_logger().info(
                "camera {}: {} images on {} (14 Hz vs the laser's 15 Hz -- not "
                "frame-locked, replayed on the bag's own clock; --sync pairs "
                "them instead)".format(
                    args.camera, self._n_img, args.image_topic))
        self.get_logger().info(
            "scan on {} at {}".format(
                args.topic,
                "{} Hz".format(args.hz) if args.hz
                else "bag rate x{:g}".format(args.rate)))

        # One fixed-period timer, ticking several times per event. Recreating a
        # timer per event (the obvious approach) costs an rcl timer allocation and
        # a wait-set rebuild each time, which caps throughput at a few Hz once two
        # streams are interleaved.
        self._t0 = time.monotonic()
        self._timer = self.create_timer(max(self._period / 4.0, 0.002), self._tick)

    def _publish_scan(self, s, stamp=None):
        msg = LaserScan()
        msg.header.frame_id = self._args.frame_id or s["frame_id"]
        msg.header.stamp = stamp or self.get_clock().now().to_msg()
        msg.angle_min = s["angle_min"]
        msg.angle_max = s["angle_max"]
        msg.angle_increment = s["angle_increment"]
        msg.range_min = s["range_min"]
        msg.range_max = s["range_max"]
        # array.array, not a list: rclpy validates sequence fields element by
        # element, but takes the fast path for an array of matching typecode.
        msg.ranges = array.array("f", s["ranges"].tobytes())
        self._pub.publish(msg)

    def _publish_image(self, d, stamp=None):
        buf = np.frombuffer(d["data"], dtype=np.uint8)
        bgr = self._cv2.imdecode(buf, self._cv2.IMREAD_COLOR)
        if bgr is None:
            return
        msg = Image()
        # Raw rather than CompressedImage: RViz's Image display then needs no
        # transport plugin or per-display configuration to show it.
        msg.header.frame_id = d["frame_id"]
        msg.header.stamp = stamp or self.get_clock().now().to_msg()
        msg.height, msg.width = bgr.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = 3 * msg.width
        # Same fast path, and here it is the difference between working and not:
        # assigning bytes to this field measures 94.9 ms per 752x480 frame
        # against a 71 ms budget at 14 Hz, versus 0.02 ms for array.array.
        msg.data = array.array("B", bgr.tobytes())
        self._img_pub.publish(msg)

    def _tick(self):
        if not rclpy.ok():
            return
        now = time.monotonic() - self._t0

        # Publish everything now due. Bounded so a long stall cannot block the
        # executor for an unbounded time catching up.
        for _ in range(64):
            if self._i >= len(self._events):
                if not self._args.loop:
                    self.get_logger().info("bag finished")
                    raise SystemExit(0)
                self._i = 0
                self._t0 = time.monotonic()
                self._dropped = 0
                return
            if self._sched[self._i] > now:
                return

            _, kind, payload = self._events[self._i]
            if kind == "pair":
                # One stamp for both, so they read as a single observation.
                stamp = self.get_clock().now().to_msg()
                self._publish_scan(payload[0], stamp)
                if self._img_pub is not None:
                    self._publish_image(payload[1], stamp)
            elif kind == "scan":
                self._publish_scan(payload)
            elif self._img_pub is not None:
                self._publish_image(payload)

            if self._i % 400 == 0:
                self.get_logger().info("event {}/{}  (playback {:+.2f} s vs bag)".format(
                    self._i, len(self._events), now - self._sched[self._i]))
            self._i += 1


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("bag", help="path to a JRDB .bag file")
    p.add_argument("--bag-topic", default=DEFAULT_TOPIC_IN,
                   help="LaserScan topic inside the bag (default %(default)s)")
    p.add_argument("--topic", default="/scan", help="topic to publish on")
    p.add_argument("--frame-id", default="",
                   help="override the bag's frame_id (default: keep it)")
    p.add_argument("--rate", type=float, default=1.0,
                   help="playback speed multiplier (default real time)")
    p.add_argument("--hz", type=float, default=0.0,
                   help="publish at this fixed rate instead of bag timing")
    p.add_argument("--loop", action="store_true")
    p.add_argument("--camera", type=int, default=None, metavar="N",
                   help="also replay camera N (the dataset ships 0, 2, 4, 6, 8)")
    p.add_argument("--image-topic", default="/camera/image",
                   help="topic to publish images on (sensor_msgs/Image)")
    p.add_argument("--sync", action="store_true",
                   help="pair each scan with its nearest image and give both the "
                        "same header stamp. Deterministic 1:1 pairing, but it "
                        "cannot reduce the offset between them (see --help notes)")
    args = p.parse_args()

    if not os.path.isfile(args.bag):
        raise SystemExit("no such bag: " + args.bag)

    image_topic = IMAGE_TOPIC_FMT.format(args.camera) if args.camera is not None else ""
    what = args.bag_topic + (" + " + image_topic if image_topic else "")
    print("reading {} from {} ...".format(what, args.bag))
    events = load_bag(args.bag, args.bag_topic, image_topic)
    if not events:
        raise SystemExit("no messages on " + args.bag_topic)

    stats = None
    if args.sync:
        events, stats = pair_by_timestamp(events)

    rclpy.init()
    node = BagScanPublisher(args, events, stats)
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
