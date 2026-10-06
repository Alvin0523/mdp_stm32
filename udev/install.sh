#!/usr/bin/env bash
# Install udev/99-mdp-stm32.rules: /dev/stm32_serial (UART3, car link) and /dev/stm32_flash (UART1, flashing).
# Run once per machine: pixi run udev (asks for the sudo password).
set -euo pipefail
RULE=99-mdp-stm32.rules
cd "$(dirname "$0")"

sudo install -m 0644 "$RULE" "/etc/udev/rules.d/$RULE"
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty --action=add
sleep 1

for link in stm32_serial stm32_flash; do
    if [ -e /dev/$link ]; then
        echo "OK: /dev/$link -> $(readlink -f /dev/$link)"
    else
        echo "no /dev/$link: is that USB-C port plugged in? (lsusb: 1a86:55d4)"
    fi
done
