#!/bin/bash
# Start the warehouse logistics system.
#
# Output is shown in the console AND saved to ./logs/warehouse_TIMESTAMP.log
# Press Ctrl+C to stop — the container shuts down cleanly.
#
# Usage:
#   ./start.sh               # normal start
#   ./start.sh --build       # rebuild image first
#   ./start.sh --build --no-cache   # full clean rebuild

set -e

mkdir -p logs

echo "Starting warehouse logistics..."
echo "Logs will be saved to ./logs/"
echo ""

docker compose up "$@"
