#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ISAACLAB_DIR="${ROOT_DIR}/submodules/IsaacLab"

if [ ! -d "${ISAACLAB_DIR}" ]; then
  echo "[ERROR] IsaacLab submodule not found at ${ISAACLAB_DIR}" >&2
  exit 1
fi
