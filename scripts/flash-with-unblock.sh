#!/usr/bin/env bash
# Wraps `pio run -t upload` so the ST-Link is only "authorized" (see
# stlink-block.sh/stlink-unblock.sh) for the moment it's actually needed.
# Default resting state is blocked - unblock right before flashing, always
# re-block afterward (even if the flash itself fails), so the board is
# left in the state it needs to actually boot the flashed firmware instead
# of the STM32 system bootloader. See docs/troubleshooting.md.
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

bash scripts/stlink-unblock.sh || exit 1

pio run -t upload
exit_code=$?

bash scripts/stlink-block.sh

if [ "$exit_code" -ne 0 ]; then
    echo "Flash failed (exit $exit_code) - ST-Link re-blocked regardless." >&2
fi
exit "$exit_code"
