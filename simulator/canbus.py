"""
Pluggable CAN bus factory.

The simulator talks to whatever is on the other end of the wire through one of
these transports. The point of the indirection is that the *same* simulator runs
against real hardware or a hardware-free virtual bus with only a flag change:

  socketcan : real adapter as a SocketCAN netdev. This is the CANable running
              candleLight/gs_usb firmware -- it shows up as `can0`. Bring it up
              with:  sudo ip link set can0 up type can bitrate 500000
  vcan      : same SocketCAN driver, but a virtual interface (`vcan0`) for
              running the whole loop on one machine with no hardware. See
              tools/vcan_up.sh.
  slcan     : CANable running slcan firmware, exposed as a serial device
              (e.g. /dev/ttyACM0). python-can drives it directly.
  virtual   : in-process python-can bus. Only useful when both ends live in the
              same Python process (e.g. unit tests), not for the C ECM.
"""

import can

DEFAULT_BITRATE = 500000


def open_bus(transport: str = "socketcan", channel: str | None = None,
             bitrate: int = DEFAULT_BITRATE) -> can.BusABC:
    transport = transport.lower()

    if transport in ("socketcan", "vcan"):
        # Bitrate for SocketCAN is configured out-of-band via `ip link`, so it
        # is not passed here. vcan ignores bitrate entirely.
        if channel is None:
            channel = "vcan0" if transport == "vcan" else "can0"
        return can.Bus(interface="socketcan", channel=channel)

    if transport == "slcan":
        if channel is None:
            channel = "/dev/ttyACM0"
        return can.Bus(interface="slcan", channel=channel, bitrate=bitrate)

    if transport == "virtual":
        return can.Bus(interface="virtual", channel=channel or "hil")

    raise ValueError(
        f"unknown transport {transport!r} (use socketcan, vcan, slcan or virtual)"
    )
