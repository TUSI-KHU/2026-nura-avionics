#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
VENV_DIR="${ROOT_DIR}/.venv-zephyr"
WORKSPACE_DIR="${ROOT_DIR}/.zephyr-workspace"
ZEPHYR_REVISION=v3.7.2

python3 -m venv "${VENV_DIR}"
"${VENV_DIR}/bin/pip" install --upgrade west

if [[ ! -f "${WORKSPACE_DIR}/.west/config" ]]; then
  "${VENV_DIR}/bin/west" init \
    -m https://github.com/zephyrproject-rtos/zephyr \
    --mr "${ZEPHYR_REVISION}" "${WORKSPACE_DIR}"
fi

(
  cd "${WORKSPACE_DIR}"
  "${VENV_DIR}/bin/west" update --narrow -o=--depth=1
  "${VENV_DIR}/bin/pip" install -r zephyr/scripts/requirements-base.txt
)

actual_revision=$(git -C "${WORKSPACE_DIR}/zephyr" describe --tags --exact-match)
if [[ "${actual_revision}" != "${ZEPHYR_REVISION}" ]]; then
  echo "Expected Zephyr ${ZEPHYR_REVISION}, found ${actual_revision}" >&2
  exit 1
fi

echo "Zephyr ${actual_revision} workspace ready: ${WORKSPACE_DIR}"
