#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$ROOT/src/vla_system:${PYTHONPATH:-}"

python3 -m compileall -q \
  "$ROOT/src/vla_system/vla_system" \
  "$ROOT/src/vla_system/launch"
python3 -m pytest -q "$ROOT/src/vla_system/test"
python3 - "$ROOT" <<'PY'
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import yaml

root = Path(sys.argv[1])
for path in root.glob("src/*/package.xml"):
    ET.parse(path)
for path in root.glob("src/*/config/*.yaml"):
    yaml.safe_load(path.read_text(encoding="utf-8"))
print("XML/YAML validation passed")
PY
