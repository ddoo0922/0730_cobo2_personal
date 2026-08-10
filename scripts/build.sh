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

# .venv must be active *and* colcon must run as a module of its python3 --
# the apt-installed /usr/bin/colcon binary always runs under /usr/bin/python3
# regardless of venv activation, so a plain `colcon build` bakes
# /usr/bin/python3 into the generated console_scripts shebang. Every real
# node then fails at runtime with ModuleNotFoundError for torch/ultralytics/
# openai/pymodbus/sounddevice, which live only in .venv (CLAUDE.md #1 forbids
# --user installs). Reproduced 2026-08-10: `colcon build` -> perception_node
# crashed with "No module named 'torch'"; `python3 -m colcon build` (venv
# active) produced the correct .venv shebang.
if [[ -f "$ROOT/.venv/bin/activate" ]]; then
  source "$ROOT/.venv/bin/activate"
else
  echo "warn: $ROOT/.venv not found; run: python3 -m venv --system-site-packages .venv" >&2
fi

python3 -m colcon build \
  --symlink-install \
  --packages-select vla_interfaces vla_system
