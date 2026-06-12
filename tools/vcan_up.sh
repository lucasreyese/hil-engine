#!/usr/bin/env bash
# Bring up a virtual CAN interface so the whole sim<->ECM loop runs with no
# hardware. Needs root (CAP_NET_ADMIN) for the ip/modprobe calls.
#
#   ./tools/vcan_up.sh [iface]     # default: vcan0
#
# For a real CANable in candleLight/gs_usb mode, you do NOT need this; instead:
#   sudo ip link set can0 up type can bitrate 500000
set -euo pipefail

IFACE="${1:-vcan0}"

sudo modprobe vcan
if ! ip link show "$IFACE" >/dev/null 2>&1; then
    sudo ip link add dev "$IFACE" type vcan
fi
sudo ip link set up "$IFACE"

echo "[vcan] '$IFACE' is up:"
ip -brief link show "$IFACE"
