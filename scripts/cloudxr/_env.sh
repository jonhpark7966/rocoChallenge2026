#!/usr/bin/env bash

# Avoid changing the caller's shell options when sourced.
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  set -euo pipefail
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ISAACLAB_DIR="${ROOT_DIR}/submodules/IsaacLab"

if [ ! -d "${ISAACLAB_DIR}" ]; then
  echo "[ERROR] IsaacLab submodule not found at ${ISAACLAB_DIR}" >&2
  if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    return 1
  fi
  exit 1
fi
