#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_env.sh
source "${SCRIPT_DIR}/_env.sh"

export XDG_RUNTIME_DIR="${ISAACLAB_DIR}/openxr/run"
export XR_RUNTIME_JSON="${ISAACLAB_DIR}/openxr/share/openxr/1/openxr_cloudxr.json"

printf "[INFO] XDG_RUNTIME_DIR=%s\n" "${XDG_RUNTIME_DIR}"
printf "[INFO] XR_RUNTIME_JSON=%s\n" "${XR_RUNTIME_JSON}"
