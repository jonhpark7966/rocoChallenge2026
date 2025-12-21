#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_env.sh
source "${SCRIPT_DIR}/_env.sh"

cd "${ISAACLAB_DIR}"

mkdir -p "${ISAACLAB_DIR}/openxr"

exec docker run -it --rm --name cloudxr-runtime \
  --user "$(id -u):$(id -g)" \
  --gpus=all \
  -e "ACCEPT_EULA=Y" \
  --mount type=bind,src="${ISAACLAB_DIR}/openxr",dst=/openxr \
  -p 48010:48010 \
  -p 47998:47998/udp \
  -p 47999:47999/udp \
  -p 48000:48000/udp \
  -p 48005:48005/udp \
  -p 48008:48008/udp \
  -p 48012:48012/udp \
  nvcr.io/nvidia/cloudxr-runtime:5.0.1
