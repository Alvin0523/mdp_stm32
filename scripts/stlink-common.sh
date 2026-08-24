# Shared helper: locate the ST-LINK/V2's USB "authorized" sysfs attribute.
# Sourced by stlink-block.sh / stlink-unblock.sh, not run directly.

find_stlink_authorized() {
    local dev vid pid
    for dev in /sys/bus/usb/devices/*/idVendor; do
        [ -e "$dev" ] || continue
        local d
        d=$(dirname "$dev")
        vid=$(cat "$d/idVendor" 2>/dev/null || true)
        pid=$(cat "$d/idProduct" 2>/dev/null || true)
        if [ "$vid" = "0483" ] && [ "$pid" = "3748" ]; then
            echo "$d/authorized"
            return 0
        fi
    done
    return 1
}
