#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_DIR="${INSTALL_DIR:-${ROOT_DIR}/install_runtime}"
DIST_DIR="${DIST_DIR:-${ROOT_DIR}/dist}"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build_runtime}"
DEVEL_DIR="${DEVEL_DIR:-${ROOT_DIR}/devel_runtime}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
JOBS="${JOBS:-$(nproc)}"
STRIP_ELF="${STRIP_ELF:-1}"
ARCH="${ARCH:-$(uname -m)}"
BUNDLE_NAME="${BUNDLE_NAME:-scan_planner_runtime_${ARCH}_$(date +%Y%m%d_%H%M%S)}"

# SCAN_Planner 源码在 src/ 下
if [ ! -d "${ROOT_DIR}/src" ]; then
  echo "ERROR: ${ROOT_DIR}/src not found."
  exit 1
fi

if [ ! -f /opt/ros/noetic/setup.bash ]; then
  echo "ERROR: /opt/ros/noetic/setup.bash not found."
  exit 1
fi

source /opt/ros/noetic/setup.bash

echo "[1/4] Cleaning runtime build dirs"
rm -rf "${INSTALL_DIR}" "${BUILD_DIR}" "${DEVEL_DIR}"
mkdir -p "${INSTALL_DIR}"

echo "[2/4] Building + installing runtime artifacts"
echo "Architecture: ${ARCH}"
cd "${ROOT_DIR}"
catkin_make \
  --source src \
  --build "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DCATKIN_DEVEL_PREFIX="${DEVEL_DIR}" \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
  install -j"${JOBS}"

if [ "${STRIP_ELF}" = "1" ]; then
  echo "[3/4] Stripping ELF symbols"
  if command -v strip >/dev/null 2>&1 && command -v file >/dev/null 2>&1; then
    while IFS= read -r -d '' f; do
      if file -b "${f}" | grep -q "^ELF"; then
        strip --strip-unneeded "${f}" || true
      fi
    done < <(find "${INSTALL_DIR}" -type f -print0)
  fi
else
  echo "[3/4] Skip stripping (STRIP_ELF=${STRIP_ELF})"
fi

echo "[4/4] Packaging tarball"
mkdir -p "${DIST_DIR}"
BUNDLE_PATH="${DIST_DIR}/${BUNDLE_NAME}.tar.gz"
tar -C "${INSTALL_DIR}" -czf "${BUNDLE_PATH}" .

echo
echo "Bundle created: ${BUNDLE_PATH}"
echo "Runtime launch:"
echo "  source /opt/ros/noetic/setup.bash"
echo "  source <deploy_dir>/setup.bash"
echo "  roslaunch scan_planner run.launch"
