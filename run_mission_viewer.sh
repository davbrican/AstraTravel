#!/usr/bin/env bash
set -euo pipefail

export DYLD_LIBRARY_PATH="/opt/homebrew/opt/expat/lib:${DYLD_LIBRARY_PATH:-}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [[ ! -x ".venv/bin/python" ]]; then
  echo "No encuentro .venv/bin/python. Crea el entorno virtual antes de lanzar el visor." >&2
  exit 1
fi

if [[ "$#" -eq 0 ]]; then
  set -- --mission-json missions/artemis_ii.json
fi

exec .venv/bin/python mission_viewer_3d.py "$@"
