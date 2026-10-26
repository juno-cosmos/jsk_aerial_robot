#!/bin/bash
# usage: rosrun orochi_hx rosbag_record.sh [name] [robot_ns] [extra topics...]
#   e.g. rosrun orochi_hx rosbag_record.sh hover_fail
#        -> logs/hover_fail_2026-10-26-11-30-00.bag

NAME=${1:-orochi_hx}
NS=${2:-orochi_hx}

# save to <package>/logs regardless of the current directory
LOG_DIR=$(cd "$(dirname "$0")/.." && pwd)/logs
mkdir -p "${LOG_DIR}"

rosbag record -o "${LOG_DIR}/${NAME}" \
  /${NS}/joint_states \
  /${NS}/gimbals_ctrl \
  /${NS}/uav/cog/odom \
  /${NS}/uav/baselink/odom \
  /${NS}/debug/target_acc_cog \
  /${NS}/four_axes/command \
  /${NS}/debug/nlopt_result \
  /${NS}/debug/nlopt_log \
  /${NS}/flight_state \
  /tf /tf_static \
  ${@:3}
