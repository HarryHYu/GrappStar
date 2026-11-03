#!/usr/bin/env bash
# Optional operator hotspot "GRAPPSTAR-ROVER" on a SECOND WiFi interface
# (a USB WiFi adapter), while the built-in WiFi stays joined to the dart AP "KEDGE".
#
#   bash setup/hotspot.sh list                 show WiFi interfaces and what they are doing
#   bash setup/hotspot.sh up [IFACE]           start the hotspot (and at every boot)
#   bash setup/hotspot.sh down                 stop it and stop it starting at boot
#   bash setup/hotspot.sh status               is it running, and the page address
#
# Change the name or password with environment variables, for example:
#   HOTSPOT_SSID=MYROVER HOTSPOT_PASS=longpassword bash setup/hotspot.sh up wlx00c0ca123456
# The password must be at least 8 characters.
#
# When it is up the Jetson is 10.42.0.1 on that network, so the operator page is
# http://10.42.0.1:8080 . The USB adapter must support AP (hotspot) mode on Linux.
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

list_wifi() {
    nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device | awk -F: '$2 == "wifi" { printf "  %-18s %-14s %s\n", $1, $3, $4 }'
}

pick_iface() {
    # First WiFi interface that is not connected to anything (the spare adapter).
    nmcli -t -f DEVICE,TYPE,STATE device | awk -F: '$2 == "wifi" && $3 == "disconnected" { print $1; exit }'
}

cmd="${1:-status}"
need_nmcli
case "$cmd" in
    list)
        echo "WiFi interfaces:"
        list_wifi
        ;;
    up)
        IFACE="${2:-$(pick_iface)}"
        if [ -z "$IFACE" ]; then
            echo "No free WiFi interface found. Plug in the USB WiFi adapter, then run:" >&2
            echo "  bash setup/hotspot.sh list" >&2
            echo "and pass the adapter's name: bash setup/hotspot.sh up <name>" >&2
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
        sed -n '2,15p' "$0"
        exit 2
        ;;
esac
