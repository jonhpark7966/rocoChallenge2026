#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_env.sh
source "${SCRIPT_DIR}/_env.sh"

cd "${ISAACLAB_DIR}"

./docker/container.py stop \
  --files docker-compose.cloudxr-runtime.patch.yaml \
  --env-file .env.cloudxr-runtime
