#!/usr/bin/env bash
# Source this, do not execute it:
#
#   source scripts/env.sh
#
# Brings up one shell that has ROS 2 Humble, the Doosan overlay carrying
# dsr_common2 / dsr_msgs2 / DSR_ROBOT2, this workspace's install space, and the
# keys from .env.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "scripts/env.sh must be sourced, not executed: source scripts/env.sh" >&2
  exit 1
fi

_vla_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ROS 2 / colcon setup scripts read internal vars they unset before returning,
# which trips `set -u` if the calling shell has it on. Restore it afterwards.
_vla_had_nounset=0
[[ -o nounset ]] && _vla_had_nounset=1
set +u

source /opt/ros/humble/setup.bash

# Python deps (openai, torch, ultralytics, ...) live in .venv, not ~/.local —
# ~/.local is shared with the cobot2_ws account and pip installing there broke
# its colcon build (2026-08-10 incident, see CLAUDE.md §1). The venv must have
# been created with --system-site-packages so rclpy stays visible.
if [[ -f "$_vla_root/.venv/bin/activate" ]]; then
  source "$_vla_root/.venv/bin/activate"
else
  echo "note: $_vla_root/.venv not found; run: python3 -m venv --system-site-packages .venv" >&2
fi

# Overlay with the Doosan packages. scripts/build.sh reads DOOSAN_SETUP too.
# This machine's real overlay is ~/cobot2_ws/install/setup.bash (2026-08-11
# confirmed present). Only table_homography_test needs it now that robot_node/
# wrist_grasp_node are gone (CLAUDE.md #3) -- the GUI pipeline (perception/
# agent/pick_bridge) runs fine without it, so a missing overlay is a note,
# not a hard failure.
export DOOSAN_SETUP="${DOOSAN_SETUP:-$HOME/cobot2_ws/install/setup.bash}"
if [[ -f "$DOOSAN_SETUP" ]]; then
  source "$DOOSAN_SETUP"
else
  echo "note: Doosan overlay not found: $DOOSAN_SETUP (only table_homography_test needs it)" >&2
fi

# This workspace, once scripts/build.sh has run at least once.
if [[ -f "$_vla_root/install/setup.bash" ]]; then
  source "$_vla_root/install/setup.bash"
else
  echo "note: $_vla_root/install not built yet; run ./scripts/build.sh" >&2
fi

# Export the API keys so vla_agent finds them regardless of its working
# directory (load_dotenv() searches upward from CWD, which ros2 launch changes).
if [[ -f "$_vla_root/.env" ]]; then
  set -a
  source "$_vla_root/.env"
  set +a
fi

(( _vla_had_nounset )) && set -u
unset _vla_root _vla_had_nounset
