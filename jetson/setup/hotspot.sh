#!/usr/bin/env bash
# Optional operator hotspot "GRAPPSTAR-ROVER" on a SECOND WiFi interface
# (a USB WiFi adapter), while the built-in WiFi stays joined to the dart AP "KEDGE".
#
#   bash setup/hotspot.sh list                 show WiFi interfaces and what they are doing
#   bash setup/hotspot.sh up [IFACE]           start the hotspot (and at every boot)
#   bash setup/hotspot.sh up IFACE --force     allow a non-USB interface (never the kedge one)
#   bash setup/hotspot.sh down                 stop it and stop it starting at boot
#   bash setup/hotspot.sh status               is it running, and the page address
#
# Change the name or password with environment variables, for example:
#   HOTSPOT_SSID=MYROVER HOTSPOT_PASS=longpassword bash setup/hotspot.sh up wlx00c0ca123456
# The password must be at least 8 characters.
#
# When it is up the Jetson is 10.42.0.1 on that network, so the operator page is
# http://10.42.0.1:8080 . The USB adapter must support AP (hotspot) mode on Linux.
# Only USB WiFi adapters are picked or allowed: never the built-in WiFi, and never the
# interface the "kedge" connection uses (that radio must stay on the dart).
set -euo pipefail

CON="grappstar-hotspot"
SSID="${HOTSPOT_SSID:-GRAPPSTAR-ROVER}"
PASS="${HOTSPOT_PASS:-grappstar}"

need_nmcli() {
    if ! command -v nmcli >/dev/null 2>&1; then
        echo "nmcli not found. Install it with: sudo apt-get install network-manager" >&2
        exit 1
    fi
}

wifi_devices() {
    nmcli -t -f DEVICE,TYPE device | awk -F: '$2 == "wifi" { print $1 }'
}

is_usb() {
    # True if the network interface sits on a USB bus (the built-in wlP1p1s0 is PCIe).
    local path
    path="$(readlink -f "/sys/class/net/$1/device" 2>/dev/null || true)"
    [[ "$path" == */usb* ]]
}

kedge_iface() {
    # Interface the saved "kedge" connection is tied to (empty if none).
    nmcli -g connection.interface-name connection show kedge 2>/dev/null || true
}

list_wifi() {
    local dev state conn bus
    while IFS=: read -r dev state conn; do
        if is_usb "$dev"; then bus="USB"; else bus="built-in"; fi
        printf "  %-18s %-9s %-14s %s\n" "$dev" "$bus" "$state" "$conn"
    done < <(nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device | awk -F: '$2 == "wifi" { print $1 ":" $3 ":" $4 }')
}

pick_iface() {
    # First USB WiFi adapter that is not the kedge interface. Never falls back to the built-in WiFi.
    local dev kedge
    kedge="$(kedge_iface)"
    while read -r dev; do
        [ -n "$dev" ] || continue
        [ "$dev" = "$kedge" ] && continue
        if is_usb "$dev"; then
            echo "$dev"
            return 0
        fi
    done < <(wifi_devices)
    return 0
}

cmd="${1:-status}"
need_nmcli
case "$cmd" in
    list)
        echo "WiFi interfaces:"
        list_wifi
        ;;
    up)
        shift
        FORCE=0
        IFACE=""
        for arg in "$@"; do
            case "$arg" in
                --force) FORCE=1 ;;
                -*) echo "Unknown option: $arg" >&2; exit 2 ;;
                *) IFACE="$arg" ;;
            esac
        done
        if [ -z "$IFACE" ]; then
            IFACE="$(pick_iface)"
        fi
        if [ -z "$IFACE" ]; then
            echo "No USB WiFi adapter found. Plug in the USB WiFi adapter, then run:" >&2
            echo "  bash setup/hotspot.sh list" >&2
            echo "and pass the adapter's name: bash setup/hotspot.sh up <name>" >&2
            echo "The built-in WiFi is never used: it stays on the dart (KEDGE)." >&2
            exit 1
        fi
        if [ ! -e "/sys/class/net/$IFACE" ]; then
            echo "No interface called $IFACE. See: bash setup/hotspot.sh list" >&2
            exit 1
        fi
        KEDGE_IF="$(kedge_iface)"
        if [ -n "$KEDGE_IF" ] && [ "$IFACE" = "$KEDGE_IF" ]; then
            echo "Refusing: $IFACE is the interface the kedge connection uses (the dart link)." >&2
            echo "Use the USB WiFi adapter instead (bash setup/hotspot.sh list). --force does not change this." >&2
            exit 1
        fi
        if ! is_usb "$IFACE" && [ "$FORCE" -eq 0 ]; then
            echo "Refusing: $IFACE is not a USB WiFi adapter (it is probably the built-in WiFi, which" >&2
            echo "must stay on KEDGE). Add --force only if you are sure." >&2
            exit 1
        fi
        if [ "${#PASS}" -lt 8 ]; then
            echo "HOTSPOT_PASS must be at least 8 characters." >&2
            exit 1
        fi
        echo "Starting hotspot $SSID on $IFACE"
        names="$(nmcli -t -f NAME connection show)"
        if grep -qx "$CON" <<<"$names"; then
            sudo nmcli connection modify "$CON" connection.interface-name "$IFACE" \
                802-11-wireless.ssid "$SSID" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PASS"
            sudo nmcli connection up "$CON"
        else
            sudo nmcli device wifi hotspot ifname "$IFACE" con-name "$CON" ssid "$SSID" password "$PASS"
        fi
        # Start at boot, and never send internet traffic through the hotspot.
        sudo nmcli connection modify "$CON" connection.autoconnect yes ipv4.never-default yes
        echo "Hotspot up. Operator: join WiFi \"$SSID\" (password: $PASS), open http://10.42.0.1:8080"
        ;;
    down)
        sudo nmcli connection modify "$CON" connection.autoconnect no 2>/dev/null || true
        sudo nmcli connection down "$CON" 2>/dev/null || true
        echo "Hotspot stopped."
        ;;
    status)
        active="$(nmcli -t -f NAME connection show --active)"
        if grep -qx "$CON" <<<"$active"; then
            echo "Hotspot $CON is UP. Page: http://10.42.0.1:8080"
        else
            echo "Hotspot $CON is down."
        fi
        echo "WiFi interfaces:"
        list_wifi
        ;;
    *)
        sed -n '2,19p' "$0"
        exit 2
        ;;
esac
