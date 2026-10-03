#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_DIR"

DO_BUILD=1
if [ "${1:-}" = "--skip-build" ]; then
    DO_BUILD=0
    shift
fi

if [ "$DO_BUILD" -eq 1 ]; then
    source /opt/ros/humble/setup.bash
    colcon build \
        --symlink-install \
        "$@" \
        --cmake-args \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
fi

if ! command -v jq >/dev/null 2>&1; then
    echo "[ERROR] jq is required to merge compile_commands.json files." >&2
    exit 1
fi

mapfile -t DATABASES < <(
    find build \
        -type f \
        -name compile_commands.json \
        | sort
)

if [ "${#DATABASES[@]}" -eq 0 ]; then
    echo "[WARN] No build/*/compile_commands.json files found yet."
    exit 0
fi

TMP_FILE="$(mktemp)"
jq -s \
    'add | unique_by(.directory, .file, .command)' \
    "${DATABASES[@]}" \
    > "$TMP_FILE"
mv "$TMP_FILE" compile_commands.json

echo "[INFO] Updated ${WORKSPACE_DIR}/compile_commands.json"
echo "[INFO] Databases merged: ${#DATABASES[@]}"
