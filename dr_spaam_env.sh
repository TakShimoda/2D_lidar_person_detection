# dr_spaam_env.sh — activate the DR-SPAAM ROS 2 environment for the current shell.
#
#   source ~/Documents/2D_lidar_person_detection/dr_spaam_env.sh
#
# Puts the torch venv's site-packages on PYTHONPATH rather than activating the
# venv. `ros2 run` execs the generated launcher directly, and colcon stamps that
# launcher with `#!/usr/bin/python3` (colcon itself is a system-python script),
# so an activated venv is ignored — PYTHONPATH is not. Safe here because the
# venv was created from /usr/bin/python3, the same interpreter ROS is built for.
#
# Same mechanism as ~/Documents/VPT_ws/vpt_env.sh; keep the two in step.
#
# Override any of these by exporting them before sourcing.

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    echo "dr_spaam_env.sh must be sourced, not executed:  source $0" >&2
    exit 1
fi

_drs_distro="${DRS_ROS_DISTRO:-humble}"
_drs_repo="${DRS_REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
_drs_ws="${DRS_WS:-$HOME/Documents/dr_spaam_ws}"
_drs_venv="${DRS_VENV:-$HOME/venvs/torch}"

# 1. ROS 2 underlay -----------------------------------------------------------
if [ ! -f "/opt/ros/${_drs_distro}/setup.bash" ]; then
    echo "dr_spaam_env: no ROS 2 at /opt/ros/${_drs_distro}" >&2
    return 1
fi
source "/opt/ros/${_drs_distro}/setup.bash"

# 2. Workspace overlay --------------------------------------------------------
# Note this is NOT the install/ tree committed inside dr_spaam_ros2/ — that one
# was built against python3.12 on the fork author's machine and is unusable on
# Humble. The real overlay lives in ${_drs_ws}, whose src/ symlinks the package.
if [ -f "${_drs_ws}/install/setup.bash" ]; then
    source "${_drs_ws}/install/setup.bash"
else
    echo "dr_spaam_env: ${_drs_ws}/install not found — run 'colcon build' in ${_drs_ws}" >&2
fi

# 3. Virtualenv packages ------------------------------------------------------
_drs_site="$(ls -d "${_drs_venv}"/lib/python3.*/site-packages 2>/dev/null | head -1)"
if [ -z "${_drs_site}" ]; then
    echo "dr_spaam_env: no site-packages under ${_drs_venv}/lib" >&2
    return 1
fi

# The venv only works via PYTHONPATH if its ABI matches the interpreter that
# will actually run the node (/usr/bin/python3). Fail loudly if it ever drifts.
_drs_venv_py="$(basename "$(dirname "${_drs_site}")")"
_drs_sys_py="$(/usr/bin/python3 -c 'import sys; print("python%d.%d" % sys.version_info[:2])')"
if [ "${_drs_venv_py}" != "${_drs_sys_py}" ]; then
    echo "dr_spaam_env: venv is ${_drs_venv_py} but /usr/bin/python3 is ${_drs_sys_py}." >&2
    echo "dr_spaam_env: recreate the venv with /usr/bin/python3 -m venv ${_drs_venv}" >&2
    return 1
fi

# Prepend (idempotently) so venv torch/numpy win over the system copies, while
# ROS's own entries stay reachable further down the path.
#
# ${_drs_repo}/dr_spaam is listed as well, and is not redundant: dr_spaam is
# installed editable, and a PEP 660 editable install is a .pth file that
# registers a MetaPathFinder. .pth files are executed by the `site` module for
# real site directories only -- putting site-packages on PYTHONPATH does not
# run them. So the venv entry alone resolves torch but NOT dr_spaam; pointing
# at the project directory (the one holding the dr_spaam package) resolves it
# straight from the working tree, which is what editable mode is for anyway.
for _drs_p in "${_drs_repo}/dr_spaam" "${_drs_site}"; do
    case ":${PYTHONPATH}:" in
        *":${_drs_p}:"*) ;;
        *) export PYTHONPATH="${_drs_p}${PYTHONPATH:+:${PYTHONPATH}}" ;;
    esac
done

# ros2_pulse: count topic rates by intercepting rcl_publish, instead of
# deserialising every message the way `ros2 topic hz` does. Exposed as a
# function, deliberately not as an exported LD_PRELOAD: LD_PRELOAD applies to
# every process started from the shell, and if the shell has no ROS on its
# LD_LIBRARY_PATH the loader prints an error before *every* command.
#
#   pulse ros2 launch dr_spaam_ros2 dr_spaam_ros2.launch.py    # terminal 1
#   pulse-top                                                  # terminal 2
#
# Scope: LD_PRELOAD is inherited, so one `pulse ros2 launch ...` covers every
# node that launch starts. It does NOT see other process trees -- a publisher
# started in another terminal needs its own `pulse`.
#
# No output file is set on purpose. Left alone, the probe writes one
# $TMPDIR/topic_freq.<pid>.log per process, which is what `pulse-top` globs, so
# several pulsed trees aggregate in one dashboard with no collision. Setting
# ROS_PULSE_LOG points them all at a single file instead, which interleaves and
# drops blocks -- only do that for a one-process `tail -f`, and with
# ROS_TOPIC_STATS_FORMAT=text for something human-readable.
#
# Counts publishers only for rclpy nodes; rclpy subscription callbacks are not
# instrumented upstream, so `pulse-top` shows no recv rate for them.
pulse() {
    if [ $# -eq 0 ]; then
        echo "usage: pulse <command...>   (e.g. pulse ros2 launch ...)" >&2
        return 2
    fi
    if [ ! -f "/opt/ros/${DRS_ROS_DISTRO:-humble}/lib/libros2_pulse.so" ]; then
        echo "pulse: libros2_pulse.so not found — sudo apt install ros-${DRS_ROS_DISTRO:-humble}-ros2-pulse" >&2
        return 1
    fi
    LD_PRELOAD=libros2_pulse.so \
    ROS_TOPIC_STATS_FORMAT="${ROS_TOPIC_STATS_FORMAT:-jsonl}" \
    ${ROS_PULSE_LOG:+ROS_TOPIC_STATS_OUTPUT_FILE="$ROS_PULSE_LOG"} \
    "$@"
}

echo "dr_spaam_env: ROS ${_drs_distro} + $(basename "${_drs_venv}") venv ready (venv is on PYTHONPATH, not activated)"

unset _drs_distro _drs_repo _drs_ws _drs_venv _drs_site _drs_venv_py _drs_sys_py _drs_p
