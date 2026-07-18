#!/bin/sh
# Sync source files from the mounted Hector directory into a git repo,
# commit any changes, then serve the repo via git daemon on port 9418.
#
# Pis clone/pull with:
#   git clone git://<WM_IP>/hector-agent ~/hector

set -e

REPO_DIR=/repos/hector-agent
SOURCE_DIR=/source

mkdir -p "$REPO_DIR"

# ── One-time repo initialisation ─────────────────────────────────────────────
if [ ! -d "$REPO_DIR/.git" ]; then
    echo "[git-daemon] Initialising repo at $REPO_DIR"
    git init "$REPO_DIR"
    git -C "$REPO_DIR" config user.email "deploy@warehouse"
    git -C "$REPO_DIR" config user.name  "Warehouse Deploy"
fi

# ── Sync source files and commit if anything changed ─────────────────────────
echo "[git-daemon] Syncing from $SOURCE_DIR"
cp "$SOURCE_DIR"/*.py "$REPO_DIR/"

git -C "$REPO_DIR" add -A

if ! git -C "$REPO_DIR" diff --cached --quiet; then
    STAMP=$(date -u +%Y-%m-%dT%H:%M:%SZ)
    git -C "$REPO_DIR" commit -m "deploy $STAMP"
    echo "[git-daemon] Committed new revision: $STAMP"
else
    echo "[git-daemon] No changes since last deploy"
fi

# ── Start git daemon ──────────────────────────────────────────────────────────
echo "[git-daemon] Serving on port 9418"
exec git daemon \
    --base-path=/repos \
    --export-all \
    --reuseaddr \
    --verbose \
    /repos
