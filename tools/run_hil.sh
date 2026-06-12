#!/usr/bin/env bash
# Launch the full HIL loop: ECM (DUT) + engine simulator (plant) on one CAN
# interface. Works on a virtual bus (vcan0, see vcan_up.sh) or a real adapter
# (can0). Ctrl-C stops both.
#
#   ./tools/run_hil.sh [iface] [profile]
#       iface   : vcan0 (default) | can0 | ...
#       profile : drive (default) | idle | wot | blip
set -euo pipefail

IFACE="${1:-vcan0}"
PROFILE="${2:-drive}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

# Build the ECM if needed.
make -C "$ROOT/ecm" >/dev/null

# Use the locally-bootstrapped deps only if python-can isn't already installed.
if python3 -c 'import can' 2>/dev/null; then
    PYP=""
else
    PYP="$ROOT/.deps"
fi

echo "[hil] interface=$IFACE profile=$PROFILE"
"$ROOT/ecm/build/ecm_host" "$IFACE" &
ECM_PID=$!
trap 'kill $ECM_PID 2>/dev/null || true' EXIT INT TERM
sleep 0.3

PYTHONPATH="${PYP}${PYTHONPATH:+:$PYTHONPATH}" \
    python3 "$ROOT/simulator/main.py" --transport socketcan --channel "$IFACE" \
        --profile "$PROFILE"
