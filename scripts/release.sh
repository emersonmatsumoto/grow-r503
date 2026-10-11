#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$SCRIPT_DIR"

LEVEL="${1:-patch}"
shift 2>/dev/null || true

# Obtém dinamicamente o nome do pacote a partir do Cargo.toml
CRATE_NAME=$(sed -n 's/^name *= *"*\([^"]*\)".*/\1/p' Cargo.toml | head -1)

if [ -z "$CRATE_NAME" ]; then
    echo "Error: Could not determine crate name from Cargo.toml" >&2
    exit 1
fi

echo "==> Releasing ${CRATE_NAME} (level: ${LEVEL})..."
cargo release --package "$CRATE_NAME" "$LEVEL" --execute "$@"

echo "==> Pushing commits and tags to remote..."
git push --follow-tags

echo "==> Release ${CRATE_NAME} executed and pushed successfully!"

