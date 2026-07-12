#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
WORKSPACE_DIR="${ROOT_DIR}/.zephyr-workspace"
WEST="${ROOT_DIR}/.venv-zephyr/bin/west"
BOARD=${1:-native_sim}

case "${BOARD}" in
  native_sim|teensy41) ;;
  *)
    echo "Usage: $0 [native_sim|teensy41]" >&2
    exit 2
    ;;
esac

if [[ ! -x "${WEST}" || ! -d "${WORKSPACE_DIR}/zephyr" ]]; then
  echo "Run scripts/setup-zephyr.sh first." >&2
  exit 1
fi

if [[ -z "${ZEPHYR_SDK_INSTALL_DIR:-}" && -d "${HOME}/zephyr-sdk-0.17.4" ]]; then
  export ZEPHYR_SDK_INSTALL_DIR="${HOME}/zephyr-sdk-0.17.4"
fi
(
  cd "${WORKSPACE_DIR}"
  "${WEST}" build -p always -b "${BOARD}" \
    -d "${ROOT_DIR}/build/zephyr/${BOARD}" \
    "${ROOT_DIR}/firmware/zephyr"
)
