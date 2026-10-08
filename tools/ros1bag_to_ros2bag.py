#!/usr/bin/env python3
"""Convert JRDB ROS1 bags to ROS2 (sqlite3), keeping only selected streams.

Available streams (--streams):
  image0        ros_indigosdk_node/image0/compressed   CompressedImage  ~15 Hz
  camera_info0  ros_indigosdk_node/camera_info0        CameraInfo       ~15 Hz (static)
  lower_lidar   lower_velodyne/velodyne_points         PointCloud2      ~14.8 Hz
  upper_lidar   upper_velodyne/velodyne_points         PointCloud2      ~14.8 Hz
  odom          segway/odometry/local_filtered         Odometry        ~100 Hz (drives tf)
  wheel_odom    segway/feedback/wheel_odometry         Odometry        ~100 Hz (raw)
  tf            tf                                     TFMessage       ~110 Hz

Usage:
  python3 convert_jrdb_to_ros2.py train_dataset_with_activity/rosbags -o ros2bags
  python3 convert_jrdb_to_ros2.py some.bag -o ros2bags --streams image0 lower_lidar odom
"""

import argparse
import sys
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.interfaces.typing import Nodetype
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

TS2 = get_typestore(Stores.ROS2_HUMBLE)

STREAMS = {
    'image0':       ('ros_indigosdk_node/image0/compressed', 'sensor_msgs/msg/CompressedImage'),
    'camera_info0': ('ros_indigosdk_node/camera_info0',      'sensor_msgs/msg/CameraInfo'),
    'lower_lidar':  ('lower_velodyne/velodyne_points',       'sensor_msgs/msg/PointCloud2'),
    #'upper_lidar':  ('upper_velodyne/velodyne_points',       'sensor_msgs/msg/PointCloud2'),
    'odom':         ('segway/odometry/local_filtered',       'nav_msgs/msg/Odometry'),
    'wheel_odom':   ('segway/feedback/wheel_odometry',       'nav_msgs/msg/Odometry'),
    'tf':           ('tf',                                   'tf2_msgs/msg/TFMessage'),
}
DEFAULT_STREAMS = ['image0', 'camera_info0', 'lower_lidar', 'odom', 'tf']


def ros1_to_ros2(src, msgtype):
    """Rebuild a ROS1 message as its ROS2 equivalent, driven by the ROS2 field defs.

    Walking the ROS2 definition (rather than copying ROS1 attributes) drops fields
    that only exist in ROS1 -- notably Header.seq -- without special-casing.
    """
    kwargs = {}
    for name, (nodetype, spec) in TS2.fielddefs[msgtype][1]:
        # ROS1 spells CameraInfo's matrices D/K/R/P; ROS2 spells them d/k/r/p.
        val = getattr(src, name, None)
        if val is None:
            val = getattr(src, name.upper())

        if nodetype is Nodetype.NAME:
            val = ros1_to_ros2(val, spec)
        elif nodetype in (Nodetype.SEQUENCE, Nodetype.ARRAY) and spec[0][0] is Nodetype.NAME:
            val = [ros1_to_ros2(v, spec[0][1]) for v in val]
        kwargs[name] = val
    return TS2.types[msgtype](**kwargs)


def convert(bag: Path, outdir: Path, streams: list[str], overwrite: bool) -> None:
    dest = outdir / bag.stem
    if dest.exists() and not overwrite:
        print(f'skip {bag.stem}: {dest} exists (use --overwrite)')
        return

    wanted = {STREAMS[s][0]: STREAMS[s][1] for s in streams}
    names = {STREAMS[s][0]: s for s in streams}
    counts: dict[str, int] = {}

    with AnyReader([bag]) as reader, Writer(dest, version=8) as writer:
        conns = [c for c in reader.connections if c.topic in wanted]
        if missing := sorted(set(wanted) - {c.topic for c in conns}):
            print(f'warn {bag.stem}: missing {missing}')
        if not conns:
            return

        out = {t: writer.add_connection(t, wanted[t], typestore=TS2)
               for t in {c.topic for c in conns}}

        for conn, timestamp, raw in reader.messages(connections=conns):
            msgtype = wanted[conn.topic]
            msg = ros1_to_ros2(reader.deserialize(raw, conn.msgtype), msgtype)
            writer.write(out[conn.topic], timestamp, TS2.serialize_cdr(msg, msgtype))
            counts[conn.topic] = counts.get(conn.topic, 0) + 1

    size = sum(f.stat().st_size for f in dest.rglob('*')) / 2**20
    print(f'{bag.stem} [{size:.0f} MiB] ' +
          ' '.join(f'{names[t]}={n}' for t, n in sorted(counts.items())))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('source', type=Path, help='a .bag file or a directory of .bag files')
    ap.add_argument('-o', '--outdir', type=Path, required=True)
    ap.add_argument('--streams', nargs='+', choices=sorted(STREAMS), default=DEFAULT_STREAMS,
                    metavar='NAME', help=f'default: {" ".join(DEFAULT_STREAMS)}')
    ap.add_argument('--overwrite', action='store_true')
    args = ap.parse_args()

    bags = sorted(args.source.glob('*.bag')) if args.source.is_dir() else [args.source]
    if not bags:
        print(f'no bags found in {args.source}', file=sys.stderr)
        return 1

    args.outdir.mkdir(parents=True, exist_ok=True)
    for bag in bags:
        convert(bag, args.outdir, args.streams, args.overwrite)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
