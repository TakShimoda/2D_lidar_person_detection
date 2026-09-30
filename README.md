# DR-SPAAM ROS 2

A ROS 2 implementation of **DR-SPAAM** for 2D LiDAR person detection.

- Paper: https://arxiv.org/abs/2004.14079
- Original implementation: https://github.com/VisualComputingInstitute/2D_lidar_person_detection
- ROS 2 fork this is based on: https://github.com/nilum2002/2D_lidar_person_detection

All credit for the DR-SPAAM algorithm goes to the original authors. This repository
packages their work as a ROS 2 node.

## Layout

| Path | What it is |
|---|---|
| `dr_spaam/` | The upstream detection library (pure Python, no ROS). `dr_spaam/dr_spaam/` is the importable package; the outer directory is the pip project root. |
| `dr_spaam_ros2/` | The ROS 2 (`ament_python`) wrapper package. |
| `dr_spaam_ros/` | The original ROS 1 package. Not used; kept for reference. |
| `dr_spaam_ros2/config/dr_spaam.rviz` | RViz2 config for the scan + detection markers. |
| `dr_spaam_env.sh` | Sources ROS, the workspace, and the venv bridge; defines the `pulse` helper. See [Why the environment needs a bridge](#why-the-environment-needs-a-bridge). |
| `tools/fake_scan_publisher.py` | Publishes a synthetic `/scan` so you can smoke-test without hardware. |
| `tools/jrdb_bag_scan_publisher.py` | Replays the **real** 2D laser from a JRDB rosbag as `/scan`. Preferred. |
| `tools/jrdb_scan_publisher.py` | Fallback: synthesises `/scan` from the Velodyne point clouds. |

## Requirements

Verified working on:

- Ubuntu 22.04, ROS 2 **Humble**, Python **3.10**
- A CUDA GPU, with torch **2.13.0+cu130** and numpy **2.2.6** in a virtualenv at `~/venvs/torch`

The Python version matters. The venv and ROS must use the *same* interpreter version
(`/usr/bin/python3`), or the bridge below cannot work. `dr_spaam_env.sh` checks this and
refuses to load if they ever drift apart.

## Setup

### 1. Virtualenv

Create it from the system interpreter, so its ABI matches the one ROS is built against:

```bash
/usr/bin/python3 -m venv ~/venvs/torch
```

Install a torch build matching your CUDA version, then the rest of `req.txt`.

### 2. Install the detection library

Install it **editable**, so edits in this repo take effect without reinstalling:

```bash
~/venvs/torch/bin/pip install -e ~/Documents/2D_lidar_person_detection/dr_spaam
```

Check it resolves to the working tree, not a copy:

```bash
~/venvs/torch/bin/python -c "import dr_spaam.detector as d; print(d.__file__)"
```

It must print a path under this repo. If it prints one under `site-packages`, the install
was not editable — uninstall and redo it with `-e`.

### 3. Checkpoints

Download the pretrained weights from the upstream repo's README and put them somewhere
readable. The default config expects:

```
dr_spaam_ros2/self_supervised_person_detection/ckpt_jrdb_ann_dr_spaam_e20.pth
```

The whole `self_supervised_person_detection/` folder is gitignored, so the weights are
never committed. Set `weight_file` in `dr_spaam_ros2/config/params.yaml` to wherever you
put them -- the committed path is the author's and will not exist on your machine.

Replaying JRDB additionally needs `python-lzf` (its point clouds are `binary_compressed`
PCDs). It is in `req.txt` but easy to miss:

```bash
~/venvs/torch/bin/pip install python-lzf
```

### 4. Workspace

The ROS package lives in this repo but has to be built from a colcon workspace:

```bash
mkdir -p ~/Documents/dr_spaam_ws/src
ln -sfn ~/Documents/2D_lidar_person_detection/dr_spaam_ros2 ~/Documents/dr_spaam_ws/src/dr_spaam_ros2
```

Build it with the **system** colcon, with the venv *not* activated:

```bash
source /opt/ros/humble/setup.bash && cd ~/Documents/dr_spaam_ws && colcon build --symlink-install
```

Building inside the venv does not work: `--symlink-install` runs `setup.py develop`, which
setuptools removed in 80, and the venv ships a newer setuptools than the system one.

> **Do not source `dr_spaam_ros2/install/setup.bash`.** That tree is committed in the fork,
> was built against Python 3.12 on another machine, and is unusable on Humble. The real
> overlay is `~/Documents/dr_spaam_ws/install`.

## Running

Every shell that runs the node needs the environment (adds venv to the PYTHONPATH without sourcing it which conflicts with colcon):

```bash
source ~/Documents/2D_lidar_person_detection/dr_spaam_env.sh
```

Then run the following commands:

1. Run the DR-SPAAM node:
```bash
ros2 launch dr_spaam_ros2 dr_spaam_ros2.launch.py
```
2. Run the publisher script to publish the camera/LiDAR:
```bash
python3 tools/jrdb_bag_scan_publisher.py ~/Documents/JRDB/train_dataset_with_activity/rosbags/memorial-court-2019-03-16_0.bag  --camera 0 --loop
```
3. Run rviz:
```bash
rviz2 -d ~/Documents/dr_spaam_ws/install/dr_spaam_ros2/share/dr_spaam_ros2/config/dr_spaam.rviz
```
4. (For now) run tf2 to get the transform to ```base_link```:
```bash
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id base_link --child-frame-id laser
```
5. (Optional) To monitor topics with ros2_pulse:
```
pulse-top
```
- Note, to run this, the first two commands above processes that publish/subscribe to the topics you want to monitor must run with ```pulse``` before them. e.g. ```pulse ros2 launch dr_spaam_ros2 dr_spaam_ros2.launch.py```

Override the defaults by exporting before sourcing: `DRS_ROS_DISTRO`, `DRS_REPO`, `DRS_WS`,
`DRS_VENV`.

With `ros2 run`, pass the config explicitly -- `ros2 run` does not read it on its own:

```bash
ros2 run dr_spaam_ros2 dr_spaam_ros2_node --ros-args --params-file ~/Documents/2D_lidar_person_detection/dr_spaam_ros2/config/params.yaml
```

> A YAML block only applies to a node whose **runtime name** matches its top-level key. If
> they disagree, ROS 2 ignores the block silently and you get the `declare_parameter`
> defaults -- which means an empty `weight_file` and `weight_file parameter is empty`. The
> node, the launch file and `params.yaml` all agree on `dr_spaam_ros2_node` here, so this
> works; if you rename any one of them, rename all three. Keying the YAML `/**:` instead
> makes it apply to any node name.

`--symlink-install` links the installed config and Python sources back to this repo, so
edits to `params.yaml` or the node take effect on the next run with no rebuild. Adding or
removing a *file* still needs `colcon build`.

### Parameters

`dr_spaam_ros2/config/params.yaml`:

| Parameter | Meaning |
|---|---|
| `weight_file` | Absolute path to the `.pth` checkpoint. Required, and **machine-specific** -- the committed value points into the author's home directory, so change it after cloning. |
| `detector_model` | `DR-SPAAM` or `DROW3`. Must match the checkpoint. |
| `conf_thresh` | Detections below this confidence are dropped. |
| `stride` | Downsample the scan by this factor. Faster, less accurate. |
| `panoramic_scan` | `true` for a 360° scan. Must match the checkpoint's training setup. |
| `scan_topic` | Input `sensor_msgs/LaserScan` topic. |
| `gpu` | `true` to run on CUDA. |

The node reads the field of view from the first `LaserScan` it receives, so no FoV
parameter is needed.

### Topics

| Topic | Type | Contents |
|---|---|---|
| `/scan` (configurable) | `sensor_msgs/LaserScan` | Input. |
| `/dr_spaam_ros2_node/detections` | `geometry_msgs/PoseArray` | One pose per person, `z = 0`, orientation unset. |
| `/dr_spaam_ros2_node/rviz_marker` | `visualization_msgs/Marker` | `LINE_LIST` drawing a 0.4 m circle per detection. |
| `/dr_spaam_ros2_node/instance_points` | `sensor_msgs/PointCloud2` | The scan points backing each detection: `x`, `y`, `z`, and an `instance` field indexing `detections`. |

#### Instance points

DR-SPAAM regresses a centre offset per scan point and groups the votes with NMS. Only the
grouped centres come back from `Detector.__call__`'s first two return values; the third,
`instance_mask`, records which cluster each point fell into, and the node republishes it as
a point cloud. `instance == i` means the point voted for `detections.poses[i]`; points in
no surviving cluster are omitted.

This is the per-person point membership, which is what you need to estimate **orientation** --
the network does not predict it (the regression head is `Conv1d(128, 2)`, just `dx, dy`).
Fitting an axis to a person's leg points gives a heading up to a 180-degree ambiguity,
which tracking resolves.

Two details about the raw mask, both handled by the node but worth knowing if you use
`instance_mask` directly:

- Ids are assigned over **every** NMS cluster in descending-confidence order, not just the
  ones above `conf_thresh`. A typical frame has ~170 ids for ~11 published detections.
- The j-th detection the detector returns owns the points marked **j + 1**. Verified
  against a control: points marked `j+1` sit 0.192 m from detection j's centre on average,
  versus ~3.5 m for `j`, `j+2` or `j+5`. The node renumbers them onto the published
  detections, so `instance` is a direct index into `poses`.

Both outputs copy the input scan's header, so they share its `frame_id` and stamp.

### Visualizing

```bash
rviz2 -d ~/Documents/dr_spaam_ws/install/dr_spaam_ros2/share/dr_spaam_ros2/config/dr_spaam.rviz
```

This shows the raw scan as grey points, each detection as a red 0.4 m circle viewed
top-down, and -- when the bag player is run with `--camera N` -- the camera image in a side
panel. RViz opens the Image panel wherever it likes; drag it to the right edge once and
`File > Save Config` to keep that layout.

**No TF tree is needed.** RViz only has to transform data into its *Fixed Frame*, and a
frame transforms to itself by identity. The config sets `Fixed Frame: base_link`, which is
what the JRDB bags publish in and what all three tools here now default to. If your LiDAR uses a different one, the
displays stay empty until you change it to match:

```bash
ros2 topic echo /scan --field header.frame_id --once
```

Once the detector is on a robot with a real TF tree, switch Fixed Frame to `odom` or `map`
and the same config keeps working -- at that point the transform is looked up rather than
assumed. To preview that arrangement before the robot exists, one static transform is
enough:

```bash
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id odom --child-frame-id base_link
```

The config subscribes Best Effort, which matches both best-effort drivers (most real LiDARs
publish `SensorDataQoS`) and reliable publishers such as `tools/fake_scan_publisher.py`.
Reliable-only would silently fail to connect to a best-effort driver.

### Smoke test without hardware

In one shell:

```bash
source ~/Documents/2D_lidar_person_detection/dr_spaam_env.sh && python3 ~/Documents/2D_lidar_person_detection/tools/fake_scan_publisher.py
```

In another, launch the node, then:

```bash
source ~/Documents/2D_lidar_person_detection/dr_spaam_env.sh && ros2 topic hz /dr_spaam_ros2_node/detections
```

On an RTX-class GPU with a 1091-point panoramic scan, the node keeps up with a 5 Hz input
(measured 4.97 Hz, 0.18–0.23 s per cycle). If the measured rate is well below the input
rate, raise `stride`.

### Replaying JRDB from a rosbag (the real laser)

This is the preferred path: the bag carries the laser's own measurements, so there is no
approximation. Two ways, both verified end to end.

**1. Play the ROS 1 bag directly.** No conversion; `rosbags` reads it in pure Python.

```bash
python3 tools/jrdb_bag_scan_publisher.py ~/Documents/JRDB/train_dataset_with_activity/rosbags/bytes-cafe-2019-02-07_0.bag --loop
```

Reads `segway/scan_multi` (5 s to preload a 2.9 GB bag), then republishes on `/scan`
following the bag's own timestamps. `--rate` scales playback speed, `--hz` forces a fixed
rate instead, `--loop` repeats. Gives ~10-16 detections per frame on `bytes-cafe`.

Add `--camera N` to replay a camera alongside the scan, for visual playback:

```bash
python3 tools/jrdb_bag_scan_publisher.py <bag> --camera 0 --loop
```

Images go out on `/camera/image` as `sensor_msgs/Image` (bgr8, 752x480), decoded from the
bag's JPEGs. Camera N is bag topic `ros_indigosdk_node/imageN/compressed` and
`cameras.yaml: sensor_N`; the five the dataset ships as `images/image_0 ... image_8` are
the even-numbered ones. Measured with the detector also running: `/scan` 14.0 Hz,
`/camera/image` 14.8 Hz, playback within 0.01 s of bag time.

#### Are the scan and images synchronised?

**Not frame-locked, but sharing one clock.** The laser runs at 15 Hz and the cameras at
14 Hz, on independent triggers, so there is no fixed pairing between them. The calibration
files carry no time offsets either -- `cameras.yaml` and `defaults.yaml` are purely spatial
(intrinsics, and `lidar_*_to_rgb` extrinsics). What they do share is the bag clock, and the
player replays both streams against it, so their true relative timing is reproduced.

Measured on `bytes-cafe-2019-02-07_0`, for every scan, the offset to the nearest image:

| | |
|---|---|
| median \|dt\| | 16.6 ms |
| p95 \|dt\| | 36.5 ms |
| max \|dt\| | 52.1 ms |
| drift across the sequence | +1.1 ms (first half) to +9.5 ms (second half) |

Half an image period is 35.7 ms, so this is the floor imposed by 14 Hz sampling rather than
slop in the player. At walking pace 36 ms is about 5 cm.

`--sync` pairs each scan with its nearest image and publishes both under **one shared
header stamp**, so downstream consumers see them as a single observation:

```bash
python3 tools/jrdb_bag_scan_publisher.py <bag> --camera 0 --sync --loop
```

```
camera 0: timestamp-matched, 1753 pairs from 1801 images (1692 used, 109 never shown);
pair offset median 16.6 ms, max 52.1 ms
```

Read those numbers before relying on it. There are 1753 scans and 1801 images, so the
streams are **not 1:1**: matching consumes 1692 distinct images, shows 109 never, and
reuses 61. And the offsets are unchanged -- median 16.6 ms, max 52.1 ms, exactly as
without `--sync`. **Matching cannot move an image closer in time to a scan**; the nearest
one is as far away as it is. What it buys is a deterministic association and identical
stamps, which is what you want for `message_filters`, for overlaying detections on the
image, or for anything that needs to know which image a detection belongs to. If you only
want to watch the two side by side, plain playback is more faithful to the recording.

Upstream does the same thing offline, building `frames_pc_im_laser.json` (see
`dr_spaam/bin/setup_jrdb_dataset.py`, `_match_pc_im_laser_one_sequence`).

**2. Convert once to a ROS 2 bag, then use `ros2 bag play`.**

```bash
rosbags-convert --src ~/Documents/JRDB/train_dataset_with_activity/rosbags/bytes-cafe-2019-02-07_0.bag --dst ~/Documents/JRDB/ros2bags/bytes-cafe-2019-02-07_0 --include-topic segway/scan_multi --dst-typestore ros2_humble --dst-version 8
```

2.3 s, and filtering to the one topic takes 2.9 GB down to 7.8 MB. Then:

```bash
ros2 bag play ~/Documents/JRDB/ros2bags/bytes-cafe-2019-02-07_0 --loop --remap segway/scan_multi:=/scan
```

`--dst-version 8` matters. Without it `rosbags` 0.11 writes metadata version 9, which
Humble's rosbag2 0.15 cannot parse:

```
Exception on parsing info file: yaml-cpp: error at line 25, column 29: bad conversion
```

Versions 4 through 8 all read correctly on Humble; 8 is the newest that works.

#### Measuring topic rates

`ros2 topic hz` is misleading on the image topic -- it reported ~10 Hz against a true
14.8 Hz, because it deserializes every 1 MB message in Python and cannot keep up.

[`ros2_pulse`](https://github.com/TanayK07/ros2_pulse) counts messages by intercepting
`rcl_publish` instead, so message size costs it nothing. It is in the **main** ROS index
for Humble -- `ros2-testing` is not needed, despite what its README says:

```bash
sudo apt install ros-humble-ros2-pulse
```

`dr_spaam_env.sh` wraps it as a `pulse` function. Prefix whatever you want measured, then
watch it live in another terminal with `pulse-top`:

```bash
pulse ros2 launch dr_spaam_ros2 dr_spaam_ros2.launch.py
```

```bash
pulse python3 tools/jrdb_bag_scan_publisher.py <bag> --camera 0 --loop
```

```bash
pulse-top
```

```
TOPIC          PUB Hz  INTRA  RECV  GAP ms  60s
/camera/image    14.8    0.0     —       —   ▁▂▄▆█▆▄▂
/scan            15.0    0.0     —       —   ▂▄▆█▆▄▂▁

SELECTED /camera/image          WARNS · structured from jsonl
pub inter  14.8 Hz              none
pub intra   0.0 Hz
recv inter     — Hz
```

`pulse-top` needs no ROS environment and no venv -- it only reads the probe's log files. It
takes no arguments in the normal case because it globs every `$TMPDIR/topic_freq.<pid>.log`,
and the probe writes one per process, so **each pulsed terminal appears in the same
dashboard automatically**. This is why `pulse` deliberately leaves
`ROS_TOPIC_STATS_OUTPUT_FILE` alone; pointing several processes at one file breaks the
aggregation. `pulse-top --demo` renders the UI against synthetic data if you just want to
see the layout first.

Reading it: `PUB Hz` is inter-process publish rate, `INTRA` intra-process, `60s` a
sparkline. `RECV` stays `—` for every node here -- that column needs `callback_start`,
which is rclcpp-only. Windows are 5 s, so figures appear a few seconds after you start and
trail reality by up to that; `--poll` changes only how often the file is re-read, not the
window.

Install note: `pip3 install --user ros2-pulse-top` can land without its dependencies -- if
`pulse-top` dies with `ModuleNotFoundError: No module named 'rich'`, rerun it as
`pip3 install --user --upgrade ros2-pulse-top`.

Measured on this pipeline, which is what `ros2 topic hz` could not do:

```
TOPIC /scan         15.00      TOPIC /dr_spaam_ros2_node/detections  12.80
TOPIC /camera/image 15.00      TOPIC /dr_spaam_ros2_node/rviz_marker 12.80
```

`pulse` applies to the command it runs **and everything that command starts** --
`LD_PRELOAD` is inherited -- so one `pulse ros2 launch ...` covers every node in the
launch file. It does *not* observe the rest of the system: with only the launch pulsed,
its log showed `/dr_spaam_ros2_node/detections` and `/rviz_marker` and nothing at all for
`/scan` or `/camera/image`, which a separately-started player was publishing at the time.
So pulse each process tree you actually want counted, and give each its own
`ROS_PULSE_LOG` -- pointing two at one file interleaves them and drops blocks.

Two caveats. It counts **publishers only** for rclpy nodes -- `rcl_publish` is in the C
layer, but `callback_start` is rclcpp-only and rclpy was never instrumented, so a Python
node's subscription activity is invisible. Everything here publishes from rclpy, so
publish rates are covered. And its per-window figures wobble by a Hz or two on a loaded
machine; it is a rate monitor, not a precision instrument.

**Do not put `LD_PRELOAD=libros2_pulse.so` in `.bashrc`**, which is why the function
exists. `libros2_pulse.so` lives in `/opt/ros/humble/lib`, which is not on the default
loader path, so in any shell where ROS has not been sourced yet -- including the top of
`.bashrc` itself -- *every command* prints:

```
ERROR: ld.so: object 'libros2_pulse.so' from LD_PRELOAD cannot be preloaded (cannot open shared object file): ignored.
```

Commands still run, but stderr is polluted for everything. And even when it does resolve,
`LD_PRELOAD` injects the shim into every process the shell starts, not just ROS nodes.

`segway/scan_multi` runs at **15 Hz**, twice the camera and point-cloud rate. The detector
keeps up with most of it (~12.9 Hz of detections out of 14.7 Hz of scans on an RTX-class
GPU); raise `stride` if your GPU falls further behind.

### Replaying JRDB from point clouds (fallback)

```bash
python3 tools/jrdb_scan_publisher.py --sequence bytes-cafe-2019-02-07_0 --loop
```

Then launch the node and open RViz as above. On `bytes-cafe-2019-02-07_0` this gives
roughly 10 detections per frame at `conf_thresh: 0.8`.

JRDB has a **real 2D laser**, and it is the sensor DR-SPAAM was trained and evaluated on
(upstream reports AP0.5 = 0.849 with it). There is no `lasers/` archive to download --
the scans live inside the JRDB **rosbags**, on topic `segway/scan_multi`, which is the
merged 360-degree, 1091-point scan from the robot's two SICK lasers. `bin/setup_jrdb_dataset.py`
extracts them to `<split>_dataset/lasers/<sequence>/000000.txt`, which is what
`dr_spaam.datahandle.jrdb_handle` then reads. Getting it needs two archives from the JRDB
downloads page that a `pointclouds`-only download does not include:

- **`rosbags`** -- the laser data itself
- **`timestamps`** -- `frames_pc.json` / `frames_img.json`, which the extraction script
  needs to synchronise laser frames to point-cloud frames (not needed for live replay)

Verified contents of `rosbags/bytes-cafe-2019-02-07_0.bag`:

| topic | type | msgs |
|---|---|---|
| `segway/scan_multi` | `sensor_msgs/LaserScan` | 1753 |
| `segway/front_scan`, `segway/rear_scan` | `sensor_msgs/LaserScan` | the two unmerged SICKs |
| `segway/odometry/local_filtered` | `nav_msgs/Odometry` | 11980 |
| `segway/feedback/wheel_odometry` | `nav_msgs/Odometry` | 11980 |
| `upper_velodyne/velodyne_points`, `lower_velodyne/...` | `sensor_msgs/PointCloud2` | 1773 |

`segway/scan_multi` is 1091 rays spanning 359.70 degrees (`angle_min` -3.140,
`angle_increment` 0.00575959), `frame_id: base_link`, `range_max` 25 m -- matching the
1091-ray panoramic grid the JRDB checkpoints expect. Odometry is in the bags too, should
you want it for something else; DR-SPAAM itself does not use it.

Note that `setup_jrdb_dataset.py` does `import rosbag`, the ROS 1 Python API, which does not
exist on Humble. Read the bags with the pure-Python `rosbags` package instead, or convert
them with `rosbags-convert` and replay straight onto `/scan` with `ros2 bag play` -- no
extraction step needed for live inference.

The publisher above is for the case where only `pointclouds/` was downloaded: it slices a
0.2 m-thick horizontal band out of the lower Velodyne at the laser's height (base
`z = -0.5`) and bins it onto the same 1091-ray grid.

Measured against `labels_3d` on that sequence, at `conf_thresh 0.8`, matching within 0.5 m:

| GT distance | instances | recall |
|---|---|---|
| 0-3 m | 749 | **0.88** |
| 3-5 m | 696 | 0.22 |
| 5-8 m | 266 | 0.00 |
| 8-12 m | 381 | 0.00 |

Overall recall 0.40 at precision 0.83. The near field is good; past ~4 m it sees nothing.

That cliff is a property of the slice, not of the detector. What it is **not**, despite the
obvious hypothesis, is the Velodyne's rings leaving gaps: measured over 50 frames, the slice
carries a comparable number of returns to the real laser in every distance band (46.9 vs
69.1 rays per frame at 4-5 m) and its contiguous runs of returns are if anything *longer*.
The slice is not starved of data at those ranges, so the exact mechanism is unresolved.

The leading hypothesis is that the slice's effective measuring height drifts with range. A
ring at elevation `theta` sits at `z = -0.135 - d*tan(theta)`, so which part of a leg gets
measured depends on how far away it is -- mean source height runs -0.48 m at 1-2 m out to
-0.54 m at 7-10 m. A real planar laser measures the same cross-section at every range, which
is what the checkpoints were trained on. This has not been verified.

What is measured, on 201 frames of `bytes-cafe-2019-02-07_0` at `conf_thresh 0.8`:

| | real laser (bag) | Velodyne slice |
|---|---|---|
| detections / frame | 11.1 | 10.3 |
| of those, at 4-6 m | **1.50** | 0.03 |
| furthest detection | 8.8 m | 19.2 m (spurious) |

Widening the slab does not help -- it makes things worse, because the nearest return per
azimuth stops resembling a knee-height profile:

| slab (base z) | recall | precision | recall < 4 m | recall > 4 m |
|---|---|---|---|---|
| -0.60 .. -0.40 (default) | **0.405** | **0.829** | **0.676** | 0.030 |
| -0.80 .. -0.20 | 0.200 | 0.770 | 0.344 | 0.000 |
| -1.00 .. -0.20 | 0.129 | 0.715 | 0.222 | 0.000 |
| -0.90 .. 0.00 | 0.109 | 0.617 | 0.187 | 0.000 |

Adding the upper Velodyne also measured slightly worse (recall 0.35 vs 0.39 at conf 0.5),
so `--use-upper` is off by default.

**No odometry is needed.** DR-SPAAM's spatial attention matches cutouts between consecutive
scans by appearance similarity, with no ego-motion input -- that is the point of the method,
versus stacking odometry-aligned scans. Nothing in `Detector` or the node consumes a pose,
so the absence of odometry in your JRDB download does not block anything here.

## Why the environment needs a bridge

`ros2 run` executes the launcher colcon generated, and colcon stamps it with a hard-coded
`#!/usr/bin/python3`. An activated virtualenv is therefore **ignored** — the node runs under
the system interpreter and cannot see torch:

```
ModuleNotFoundError: No module named 'dr_spaam'
```

`PYTHONPATH`, unlike `$PATH`, *is* honoured by that interpreter. So `dr_spaam_env.sh` puts
the venv on `PYTHONPATH` instead of activating it. It adds two entries:

- the venv's `site-packages`, for torch, numpy and friends;
- this repo's `dr_spaam/` project directory, for the detection library itself.

The second is not redundant. A PEP 660 editable install is a `.pth` file that registers a
MetaPathFinder, and `.pth` files are executed by the `site` module for real site
directories only — putting `site-packages` on `PYTHONPATH` does not run them. Pointing at
the project directory resolves the package straight from the working tree, which is what
editable mode is for.

## Known issues

**numpy 2 vs ROS C extensions.** The bridge puts the venv's numpy 2.2.6 ahead of Ubuntu's
1.21.5 for every ROS package in that shell. This node is fine — it imports only `rclpy`,
`numpy` and message packages. But Humble's compiled extensions are built against the
numpy 1.x ABI and some break:

```
cv_bridge/__init__.py: AttributeError: _ARRAY_API not found
```

It still appears to import; its C extension has silently failed to initialise. If you add a
node that needs `cv_bridge`, either pin `numpy<2` in the venv or decode images with
`cv2.imdecode` directly.

**A callback exception kills the node.** `main()` wraps `rclpy.spin()` in a bare
`except Exception`, so any error inside `_scan_callback` terminates the process — and prints
`Error starting node:` even when the failure happened long after startup, or was just
Ctrl-C. Launch still reports "process has finished cleanly". Worth moving the handler
inside the callback.

**Stale build artifacts are committed.** `dr_spaam_ros2/install/` and `dr_spaam/build/` are
tracked in git from the fork. They are built for the wrong Python version and should be
removed from the index.

**Do not run `python3` from inside `dr_spaam/`.** That directory is the pip project root and
has no `__init__.py`, so an interpreter started there picks it up as a namespace package and
shadows the real one.

## Local changes to the upstream library

`dr_spaam/` is upstream code with three fixes. Reapply them if you re-pull upstream.

1. **`np.int` / `np.bool` removed** (`utils/utils.py`, `utils/jrdb_utils.py`,
   `utils/precision_recall.py`, `model/dr_spaam_fn.py`, `dataset/drow_dataset.py`).
   These aliases were deleted in numpy 1.24, so `scans_to_cutout` — on the inference path —
   raised `AttributeError` on every scan. `np.int` *was* the builtin `int` (`np.int is int`
   is `True` on numpy 1.21.5), so `int` and `bool` are exact substitutions. Verified
   bit-identical over 3.37M values across 24 configurations, both under numpy 1.21.5 and
   cross-checked against numpy 2.2.6.

2. **`Detector.__call__` accepts `scan_phi`** (`dr_spaam/detector.py`). The ROS 2 node passes
   the true per-ray angles from the `LaserScan` metadata, but upstream only derived them
   from `set_laser_fov()`, which assumes a symmetric FoV centred on zero. Without this the
   node raised `TypeError: unexpected keyword argument 'scan_phi'` on its first scan. The
   argument is optional and the old behaviour is unchanged when it is omitted.

3. **`dr_spaam/detector.py` moved to `dr_spaam/examples/minimal_usage.py`.** It was a copy of
   the upstream README's usage snippet, sitting where it shadowed the real
   `dr_spaam.detector` module and turned any import from the repo root into a confusing
   circular-import error.

## Local change to the ROS 2 wrapper

`dr_spaam_ros2_node.py` names itself `dr_spaam_ros2_node`. It previously used
`dr_spaam_ros2`, which did not match the `params.yaml` key, so `ros2 run --params-file` was
silently ignored and only `ros2 launch` worked (the launch file renames the node). As a side
effect the output topics are now the same under both -- they used to be
`/dr_spaam_ros2/...` under `ros2 run` and `/dr_spaam_ros2_node/...` under `ros2 launch`.
