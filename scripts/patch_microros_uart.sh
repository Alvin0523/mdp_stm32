#!/usr/bin/env bash
# The micro-ROS Zephyr module's serial transport hardcodes USART1, but this
# board's micro-ROS link is USART3 (PD8/PD9, per docs/stm32/pinouts.md). West
# re-fetches this module from scratch on every setup, so this patch has to be
# re-applied instead of hand-edited once.
set -euo pipefail

TRANSPORT_C="micro_ros_zephyr_module/modules/libmicroros/microros_transports/serial/microros_transports.c"

if [ ! -f "$TRANSPORT_C" ]; then
    echo "error: $TRANSPORT_C not found (did west update run?)" >&2
    exit 1
fi

if grep -q "DT_NODELABEL(usart3)" "$TRANSPORT_C"; then
    echo "USART3 already patched into $TRANSPORT_C"
    exit 0
fi

if ! grep -q "#define UART_NODE DT_NODELABEL(usart1)" "$TRANSPORT_C"; then
    echo "error: expected line '#define UART_NODE DT_NODELABEL(usart1)' not found in $TRANSPORT_C — module source may have changed" >&2
    exit 1
fi

sed -i 's/#define UART_NODE DT_NODELABEL(usart1)/#define UART_NODE DT_NODELABEL(usart3)/' "$TRANSPORT_C"

echo "patched UART_NODE to usart3 in $TRANSPORT_C"
