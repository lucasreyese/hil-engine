/*
 * can_socketcan.c -- Linux SocketCAN backend for can_iface.
 *
 * Works against a real CANable in candleLight/gs_usb mode (shows up as `can0`)
 * and against a virtual interface (`vcan0`) identically -- that is the whole
 * point of running it on a vcan: the firmware path is the same with or without
 * hardware. Bring the interface up before launching (see tools/vcan_up.sh or the
 * ecm/README).
 */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "can_iface.h"

bool can_open(can_iface_t *it, const char *channel, uint32_t bitrate) {
    (void)bitrate;  /* SocketCAN bitrate is configured out-of-band via ip link */
    it->fd = -1;
    it->ctx = NULL;

    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("socket(PF_CAN)");
        return false;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof ifr);
    strncpy(ifr.ifr_name, channel, IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        fprintf(stderr, "can_open: interface '%s' not found (is it up?): ", channel);
        perror("SIOCGIFINDEX");
        close(s);
        return false;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof addr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof addr) < 0) {
        perror("bind(can)");
        close(s);
        return false;
    }

    /* Non-blocking so can_recv() can poll without stalling the control loop. */
    int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);

    it->fd = s;
    return true;
}

bool can_send(can_iface_t *it, const can_frame_t *frame) {
    struct can_frame cf;
    memset(&cf, 0, sizeof cf);
    cf.can_id = frame->id & CAN_SFF_MASK;  /* standard 11-bit */
    cf.can_dlc = frame->dlc;
    memcpy(cf.data, frame->data, frame->dlc);
    return write(it->fd, &cf, sizeof cf) == (ssize_t)sizeof cf;
}

bool can_recv(can_iface_t *it, can_frame_t *frame) {
    struct can_frame cf;
    ssize_t n = read(it->fd, &cf, sizeof cf);
    if (n < (ssize_t)sizeof cf) {
        return false;  /* EAGAIN (empty) or a short read */
    }
    if (cf.can_id & (CAN_EFF_FLAG | CAN_ERR_FLAG)) {
        return false;  /* ignore extended/error frames: this bus is standard-ID */
    }
    frame->id = cf.can_id & CAN_SFF_MASK;
    frame->dlc = cf.can_dlc;
    memcpy(frame->data, cf.data, cf.can_dlc);
    return true;
}

void can_close(can_iface_t *it) {
    if (it->fd >= 0) {
        close(it->fd);
    }
    it->fd = -1;
}
