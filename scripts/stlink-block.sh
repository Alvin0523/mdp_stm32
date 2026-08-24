#!/usr/bin/env bash
# Deauthorize the ST-Link at the USB level - the kernel treats it as
# unplugged (no cable touching needed). Run this before resetting/power-
# cycling the board so it boots the flashed firmware instead of landing in
# the STM32 system bootloader. Requires scripts/install-stlink-udev-rule.sh
# to have been run once already (otherwise this needs sudo).
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source stlink-common.sh

AUTH_FILE=$(find_stlink_authorized) || {
    echo "ST-Link not found on the USB bus (already blocked, or truly unplugged?)." >&2
    exit 1
}

echo 0 > "$AUTH_FILE"
echo "ST-Link deauthorized (blocked) - board can now boot/run firmware normally."
