#!/bin/bash
# Prepend a newer NVRTC (12.9.86, from pip nvidia-cuda-nvrtc-cu12) so cupy can
# JIT-compile kernels for this machine's Blackwell GPU (RTX 5060, sm_120).
# The system libnvrtc.so.12 (12.0.140) predates sm_120 support and causes
# CUDA_ERROR_NO_BINARY_FOR_GPU on any cupy kernel launch.
export LD_LIBRARY_PATH="/home/user/.local/lib/python3.12/site-packages/nvidia/cuda_nvrtc/lib:${LD_LIBRARY_PATH}"

source /opt/ros/jazzy/setup.bash
source "$(dirname "$(readlink -f "$0")")/install/setup.bash"

# MuJoCo is the single dynamic TF authority for odom -> base.  Do not start a
# second broadcaster here: the old helper actually published map -> base,
# giving `base` two parents and making the TF tree nondeterministic.

ros2 launch elevation_mapping_cupy elevation_mapping_cupy.launch.py "$@"
