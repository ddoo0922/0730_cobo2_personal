#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble setup not found: /opt/ros/humble/setup.bash" >&2
  exit 1
fi
# ROS 2 / colcon setup scripts rely on internal vars (e.g. AMENT_TRACE_SETUP_FILES)
# that they unset before returning, which trips `set -u`. Relax nounset only
# around sourcing them.
set +u
source /opt/ros/humble/setup.bash

# Optional overlay containing dsr_common2 / dsr_msgs2.
if [[ -n "${DOOSAN_SETUP:-}" ]]; then
  if [[ ! -f "$DOOSAN_SETUP" ]]; then
    set -u
    echo "DOOSAN_SETUP does not exist: $DOOSAN_SETUP" >&2
    exit 1
  fi
  source "$DOOSAN_SETUP"
fi
set -u

colcon build \
  --symlink-install \
  --packages-select vla_interfaces vla_system
