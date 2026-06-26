#!/usr/bin/env bash

_general_planner_release_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "${ROS_DISTRO:-}" ] && [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi

export GENERAL_PLANNER_RELEASE_DIR="${_general_planner_release_dir}"
export ROS_PACKAGE_PATH="${GENERAL_PLANNER_RELEASE_DIR}/src:${ROS_PACKAGE_PATH:-}"
export PYTHONPATH="${GENERAL_PLANNER_RELEASE_DIR}/lib/python3/dist-packages:${PYTHONPATH:-}"
export CPLUS_INCLUDE_PATH="${GENERAL_PLANNER_RELEASE_DIR}/include:${CPLUS_INCLUDE_PATH:-}"
