#!/usr/bin/env bash
# Upstream Zephyr v4.0.0 doesn't wire a pyocd runner into the stm32f4_disco
# board (only stm32cubeprogrammer/jlink are registered), but pyocd is what
# we actually flash with (see pixi.toml `flash` task). `west update` re-fetches
# zephyr/ from scratch, so this patch has to be re-applied after every setup
# instead of hand-edited once.
set -euo pipefail

BOARD_CMAKE="zephyr/boards/st/stm32f4_disco/board.cmake"

if [ ! -f "$BOARD_CMAKE" ]; then
    echo "error: $BOARD_CMAKE not found (did west update run?)" >&2
    exit 1
fi

if grep -q "board_runner_args(pyocd" "$BOARD_CMAKE"; then
    echo "pyocd runner already patched into $BOARD_CMAKE"
    exit 0
fi

sed -i \
    -e '/board_runner_args(jlink/a board_runner_args(pyocd "--target=stm32f407vetx")' \
    -e '/include(\${ZEPHYR_BASE}\/boards\/common\/jlink.board.cmake)/a include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)' \
    "$BOARD_CMAKE"

echo "patched pyocd runner into $BOARD_CMAKE"
