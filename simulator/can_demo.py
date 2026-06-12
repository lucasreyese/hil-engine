import can
import time

# Candlelight firmware on Linux
# For setting the rate of the adapter, use the command `sudo ip link set can0 type can bitrate 500000 on`
bus = can.interface.Bus(
    interface='socketcan',
    channel='can0',
    bitrate=500000
)

msg = can.Message(arbitration_id=0xc0ffee,
                  data=[0, 25, 0, 1, 3, 1, 4, 1],
                  is_extended_id=True)

while True:
    try:
        bus.send(msg)
        print("Message sent on {}".format(bus.channel_info))
    except can.CanError as e:
        print("Message NOT sent: " + str(e))
    time.sleep(0.1)
