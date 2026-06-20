#!/usr/bin/env bash
set -euo pipefail

PI_HOST="${PI_HOST:-piolo@192.168.4.1}"
IMAGE_NAME="drone-vision"
ARCHIVE="drone-vision.tar.gz"

echo "=== Building Docker image for ARM64 ==="
docker build -t "$IMAGE_NAME" -f docker/Dockerfile .

echo "=== Saving image ==="
docker save "$IMAGE_NAME" | gzip > "$ARCHIVE"

echo "=== Transferring to Pi ($PI_HOST) ==="
scp "$ARCHIVE" "$PI_HOST":~/

echo "=== Loading image on Pi ==="
ssh "$PI_HOST" "docker load < ~/$ARCHIVE && rm ~/$ARCHIVE"

echo "=== Done! ==="
echo ""
echo "Run on the Pi with:"
echo "  # Dry run (camera only):"
echo "  docker run --rm --device /dev/video0 $IMAGE_NAME --dry-run"
echo ""
echo "  # With Pixhawk:"
echo "  docker run --rm --device /dev/video0 --device /dev/ttyACM0 -p 8080:8080 $IMAGE_NAME --port /dev/ttyACM0"

rm -f "$ARCHIVE"
