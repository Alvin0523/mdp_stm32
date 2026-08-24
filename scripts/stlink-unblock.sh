#!/usr/bin/env bash
# Reauthorize a previously-blocked ST-Link (see stlink-block.sh) so it can
# be used for pixi run probe/build/flash again.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source stlink-common.sh

AUTH_FILE=$(find_stlink_authorized) || {
    echo "ST-Link not found on the USB bus - is it physically plugged in?" >&2
    exit 1
}

echo 1 > "$AUTH_FILE"
echo "ST-Link reauthorized (unblocked) - ready for pixi run probe/build/flash."
