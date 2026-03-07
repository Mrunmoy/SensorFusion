#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VERSION="$(head -n1 "${ROOT_DIR}/VERSION" | tr -d '[:space:]')"

if [[ ! "${VERSION}" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  echo "Invalid VERSION: '${VERSION}'" >&2
  exit 1
fi

TAG_NAME="${1:-v${VERSION}}"
OUT_DIR="${ROOT_DIR}/dist"
ARCHIVE_NAME="SensorFusion-${TAG_NAME}.tar.gz"
ARCHIVE_PATH="${OUT_DIR}/${ARCHIVE_NAME}"
SHA_PATH="${ARCHIVE_PATH}.sha256"

mkdir -p "${OUT_DIR}"
rm -f "${ARCHIVE_PATH}" "${SHA_PATH}"

git -C "${ROOT_DIR}" archive \
  --format=tar.gz \
  --prefix="SensorFusion-${TAG_NAME}/" \
  -o "${ARCHIVE_PATH}" \
  HEAD

sha256sum "${ARCHIVE_PATH}" > "${SHA_PATH}"

echo "Created:"
echo "  ${ARCHIVE_PATH}"
echo "  ${SHA_PATH}"
