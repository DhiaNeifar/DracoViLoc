#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

SDK_DIR="${REPO_ROOT}/fairino-cpp-sdk/libfairino"
BUILD_DIR="${SDK_DIR}/build-native"
SDK_BIN_DIR="${SDK_DIR}/LinuxBuild/bin"
DEST_DIR="${REPO_ROOT}/src/fairino_hardware/libfairino"
DEST_INCLUDE_DIR="${DEST_DIR}/include"
DEST_LIB_DIR="${DEST_DIR}/lib"

case "$(uname -m)" in
  x86_64|amd64)
    ARCH_LABEL="x86_64"
    ;;
  aarch64|arm64)
    ARCH_LABEL="aarch64"
    ;;
  *)
    ARCH_LABEL="$(uname -m)"
    echo "Warning: unrecognized architecture '${ARCH_LABEL}'. Building native SDK library anyway." >&2
    ;;
esac

if [[ ! -f "${SDK_DIR}/CMakeLists.txt" ]]; then
  echo "Fairino SDK CMake project not found at: ${SDK_DIR}" >&2
  exit 1
fi

echo "Repository root: ${REPO_ROOT}"
echo "Fairino SDK:     ${SDK_DIR}"
echo "Architecture:    ${ARCH_LABEL}"

cmake -S "${SDK_DIR}" -B "${BUILD_DIR}"
cmake --build "${BUILD_DIR}" --target fairino

if [[ ! -f "${SDK_BIN_DIR}/libfairino.so.2.2.3" ]]; then
  echo "Expected SDK library was not produced: ${SDK_BIN_DIR}/libfairino.so.2.2.3" >&2
  exit 1
fi

mkdir -p "${DEST_INCLUDE_DIR}" "${DEST_LIB_DIR}"

cp -a "${SDK_BIN_DIR}"/libfairino.so* "${DEST_LIB_DIR}/"
cp -a "${SDK_DIR}/src/include/Robot-EN/"*.h "${DEST_INCLUDE_DIR}/"

echo
echo "Installed Fairino SDK artifacts:"
file "${DEST_LIB_DIR}/libfairino.so.2.2.3"
echo
echo "Libraries copied to: ${DEST_LIB_DIR}"
echo "Headers copied to:   ${DEST_INCLUDE_DIR}"
