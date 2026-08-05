#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-scan_planner:noetic-builder-arm64}"
PLATFORM="${PLATFORM:-linux/arm64}"
STAMP="$(date +%Y%m%d_%H%M%S)"
JOBS="${JOBS:-$(nproc)}"
SKIP_IMAGE_BUILD="${SKIP_IMAGE_BUILD:-0}"
CLEAN_BUILD="${CLEAN_BUILD:-0}"
USE_CCACHE="${USE_CCACHE:-1}"
BUNDLE_NAME="${BUNDLE_NAME:-scan_planner_runtime_aarch64_${STAMP}}"

cd "${ROOT_DIR}"

if [ "${SKIP_IMAGE_BUILD}" = "1" ]; then
  echo "[1/2] Reusing builder image: ${IMAGE}"
  if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "ERROR: ${IMAGE} does not exist; rerun without SKIP_IMAGE_BUILD=1."
    exit 1
  fi
else
  echo "[1/2] Building ${PLATFORM} ROS Noetic builder image: ${IMAGE}"
  if ! docker buildx version >/dev/null 2>&1; then
    echo "ERROR: docker buildx is required. Install: sudo apt-get install -y docker-buildx"
    exit 1
  fi

  # 确保本地有 arm64 版本的基础镜像（通过 Docker daemon 镜像加速器拉取）
  BASE_IMAGE="ros:noetic-ros-base-focal"
  if ! docker image inspect "${BASE_IMAGE}" 2>/dev/null | grep -q '"Architecture": "arm64"'; then
    echo "  Local arm64 base image not found, pulling via Docker daemon mirrors..."
    docker pull --platform "${PLATFORM}" "${BASE_IMAGE}"
  fi

  # 使用 default builder（docker driver）直接访问本地 Docker daemon 镜像缓存
  # --pull=false 避免从远程拉取，仅使用本地已有镜像
  docker buildx build \
    --builder default \
    --platform "${PLATFORM}" \
    --pull=false \
    --load \
    -f Dockerfile.noetic-builder \
    -t "${IMAGE}" \
    .
fi

echo "[2/2] Building arm64 runtime bundle under QEMU/binfmt"
DOCKER_TTY_ARGS=()
if [ -t 0 ] && [ -t 1 ]; then
  DOCKER_TTY_ARGS=(-it)
fi

docker run --rm "${DOCKER_TTY_ARGS[@]}" \
  --platform "${PLATFORM}" \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -e JOBS="${JOBS}" \
  -e ARCH=aarch64 \
  -e CLEAN_BUILD="${CLEAN_BUILD}" \
  -e USE_CCACHE="${USE_CCACHE}" \
  -e CCACHE_DIR=/ws/.ccache_arm64 \
  -e BUNDLE_NAME="${BUNDLE_NAME}" \
  -v "${ROOT_DIR}":/ws \
  -w /ws \
  "${IMAGE}" \
  bash -lc "source /opt/ros/noetic/setup.bash && ./scripts/build_runtime_bundle.sh"

echo
echo "Done."
echo "Bundle: ${ROOT_DIR}/dist/${BUNDLE_NAME}.tar.gz"
echo "Mode: $([ "${CLEAN_BUILD}" = "1" ] && echo clean || echo incremental)"
echo
echo "If Docker reports an exec format error, install arm64 binfmt once:"
echo "  docker run --privileged --rm tonistiigi/binfmt --install arm64"
