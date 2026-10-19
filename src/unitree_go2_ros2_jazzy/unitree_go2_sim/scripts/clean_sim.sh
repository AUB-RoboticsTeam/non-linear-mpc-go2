#!/usr/bin/env bash
# Forcefully kills every process from a unitree_go2_sim run (gz sim, ros2 launch,
# controller_manager/spawners, bridges, rviz).
#
# Why this exists: signals sent through screen/tmux/ros2-launch/shell-wrapper
# layers do not reliably reach the "gz sim server"/"gz sim gui" binaries at the
# bottom of that process chain -- a single `pkill -f "gz sim"` can silently miss
# them, especially the GUI process. A leftover gz sim server shares the same
# default gz-transport bus (GZ_PARTITION=hostname:user) as any new run, which is
# what causes symptoms like "the robot sometimes doesn't spawn" or a controller
# spawner that hangs waiting on a service. This script keeps re-checking and
# force-killing by PID until nothing matching is left, instead of trusting a
# single pattern-matched pkill pass.
#
# It also sweeps /dev/shm for orphaned FastDDS shared-memory segments
# (fastrtps_*) left behind by any ROS 2 process that was ever killed with
# SIGKILL instead of shutting down cleanly -- those don't get cleaned up by the
# OS, and a pile of them is a documented cause of ROS 2 nodes intermittently
# failing to discover/match each other (which looks exactly like "sometimes
# the robot spawns, sometimes it doesn't"). Only segments with no process
# still holding them open are removed, so this is safe to run even if other,
# unrelated ROS 2 work is active on the machine.
#
# Usage: ./clean_sim.sh [max_attempts]

set -u

PATTERNS=(
    "gz sim"
    "ruby.*gz"
    "ros2 launch unitree_go2_sim"
    "lib/controller_manager/controller_manager"
    "lib/controller_manager/spawner"
    "lib/ros2_control_node/ros2_control_node"
    "lib/ros_gz_sim/create"
    "ros2 control list_controllers"
    "ros_gz_bridge"
    "robot_state_publisher"
    "rviz2"
)

sweep_stale_shm() {
    local removed=0
    shopt -s nullglob
    for f in /dev/shm/fastrtps_* /dev/shm/fastrtps_*_el; do
        if ! fuser "$f" >/dev/null 2>&1; then
            rm -f "$f"
            removed=$((removed + 1))
        fi
    done
    shopt -u nullglob
    echo "clean_sim.sh: removed $removed orphaned /dev/shm/fastrtps_* segment(s)."
}

max_attempts="${1:-5}"

for attempt in $(seq 1 "$max_attempts"); do
    pids=""
    for pat in "${PATTERNS[@]}"; do
        found=$(pgrep -f "$pat" 2>/dev/null || true)
        pids="$pids $found"
    done
    pids=$(echo "$pids" | tr ' ' '\n' | grep -E '^[0-9]+$' | sort -u)

    if [ -z "$pids" ]; then
        echo "clean_sim.sh: no matching processes remain (attempt $attempt)."
        sweep_stale_shm
        exit 0
    fi

    echo "clean_sim.sh: attempt $attempt, killing PIDs: $(echo "$pids" | tr '\n' ' ')"
    echo "$pids" | xargs -r kill -9
    sleep 1
done

remaining=""
for pat in "${PATTERNS[@]}"; do
    remaining="$remaining $(pgrep -f "$pat" 2>/dev/null || true)"
done
remaining=$(echo "$remaining" | tr ' ' '\n' | grep -E '^[0-9]+$' | sort -u)

if [ -n "$remaining" ]; then
    echo "clean_sim.sh: WARNING -- still running after $max_attempts attempts:"
    ps -o pid,ppid,cmd -p $(echo "$remaining" | tr '\n' ',' | sed 's/,$//')
    exit 1
fi

echo "clean_sim.sh: clean."
sweep_stale_shm
