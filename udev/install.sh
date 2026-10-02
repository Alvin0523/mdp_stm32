#!/usr/bin/env bash
# Install udev/99-mdp-stm32.rules so the STM32 board shows up as /dev/stm32.
# Run once per machine: pixi run udev (asks for the sudo password).
set -euo pipefail
RULE=99-mdp-stm32.rules
cd "$(dirname "$0")"

sudo install -m 0644 "$RULE" "/etc/udev/rules.d/$RULE"
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty --action=add
sleep 1

if [ -e /dev/stm32 ]; then
    echo "OK: /dev/stm32 -> $(readlink -f /dev/stm32)"
else
    echo "Rule installed, but no /dev/stm32: is the STM32 plugged in by USB? (lsusb: 1a86:55d4)"
fi
