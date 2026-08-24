#!/usr/bin/env bash
# One-time setup (run with sudo): lets stlink-block.sh/stlink-unblock.sh toggle
# the ST-Link's USB authorization without sudo afterward. See docs/troubleshooting.md
# for why this exists - resetting the board while the ST-Link stays connected
# reliably lands it in the STM32's system bootloader instead of running the
# flashed firmware.
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "Run with sudo: sudo bash scripts/install-stlink-udev-rule.sh" >&2
    exit 1
fi

RULE_FILE=/etc/udev/rules.d/99-stlink-authorized.rules

cat > "$RULE_FILE" <<'EOF'
# ST-LINK/V2 (STMicroelectronics, VID:PID 0483:3748) - grant world-writable
# access to this device's own USB "authorized" attribute so mdp_stm32's
# stlink-block.sh/stlink-unblock.sh can toggle it without sudo. Scoped to
# this exact VID:PID only - no effect on any other USB device.
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="3748", RUN+="/bin/chmod 666 /sys%p/authorized"
EOF

udevadm control --reload-rules
udevadm trigger --subsystem-match=usb

echo "Installed $RULE_FILE"
echo "Replug the ST-Link (or run: sudo udevadm trigger) for it to take effect immediately."
