#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_DIR"

if [ ! -s compile_commands.json ]; then
    bash .devcontainer/update-compile-commands.sh --skip-build
fi

if [ ! -s compile_commands.json ]; then
    echo "[ERROR] compile_commands.json is missing. Run the VS Code build task first." >&2
    exit 1
fi

RUN_CLANG_TIDY="$(command -v run-clang-tidy || command -v run-clang-tidy-14 || true)"
if [ -z "$RUN_CLANG_TIDY" ]; then
    echo "[ERROR] run-clang-tidy is not installed." >&2
    exit 1
fi

JOBS="${CLANG_TIDY_JOBS:-$(nproc)}"
exec "$RUN_CLANG_TIDY" -p "$WORKSPACE_DIR" -j "$JOBS" "$@"
