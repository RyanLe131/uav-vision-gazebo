#!/usr/bin/env bash
# upload_demo_video.sh
# Usage: ./scripts/upload_demo_video.sh path/to/demo.mp4
#
# Uploads a demo video to the latest GitHub Release and prints
# the download URL to embed in README.md.
#
# Requirements:
#   - gh CLI authenticated (gh auth login)
#   - repo: RyanLe131/uav-vision-gazebo

set -e

REPO="RyanLe131/uav-vision-gazebo"
RELEASE_TAG="demo-v1"
RELEASE_TITLE="Demo Video"
RELEASE_NOTES="Drone orbit detection demo video."

VIDEO_PATH="${1:-}"

# ── Validate input ────────────────────────────────────────────────────────────
if [[ -z "$VIDEO_PATH" ]]; then
    echo "Usage: $0 path/to/demo.mp4"
    exit 1
fi

if [[ ! -f "$VIDEO_PATH" ]]; then
    echo "Error: file not found: $VIDEO_PATH"
    exit 1
fi

VIDEO_FILE=$(basename "$VIDEO_PATH")

# ── Create release if it doesn't exist ───────────────────────────────────────
echo "Checking release '$RELEASE_TAG' on $REPO ..."
if ! gh release view "$RELEASE_TAG" --repo "$REPO" &>/dev/null; then
    echo "Creating release '$RELEASE_TAG' ..."
    gh release create "$RELEASE_TAG" \
        --repo "$REPO" \
        --title "$RELEASE_TITLE" \
        --notes "$RELEASE_NOTES"
else
    echo "Release '$RELEASE_TAG' already exists, uploading asset ..."
fi

# ── Upload video asset ────────────────────────────────────────────────────────
echo "Uploading $VIDEO_FILE ..."
gh release upload "$RELEASE_TAG" "$VIDEO_PATH" \
    --repo "$REPO" \
    --clobber   # overwrite if same filename already exists

# ── Print the download URL ────────────────────────────────────────────────────
URL=$(gh release view "$RELEASE_TAG" --repo "$REPO" --json assets \
    --jq ".assets[] | select(.name == \"$VIDEO_FILE\") | .url")

echo ""
echo "Upload complete!"
echo ""
echo "Direct URL:"
echo "  $URL"
echo ""
echo "Add to README.md:"
echo "  https://github.com/$REPO/releases/download/$RELEASE_TAG/$VIDEO_FILE"
