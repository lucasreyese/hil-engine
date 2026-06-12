#!/usr/bin/env bash
# Tear down a virtual CAN interface created by vcan_up.sh.
#   ./tools/vcan_down.sh [iface]   # default: vcan0
set -euo pipefail

IFACE="${1:-vcan0}"
sudo ip link set down "$IFACE" 2>/dev/null || true
sudo ip link delete "$IFACE" 2>/dev/null || true
echo "[vcan] '$IFACE' removed"
