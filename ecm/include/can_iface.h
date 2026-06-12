/*
 * can_iface.h -- transport-agnostic CAN interface (the porting seam).
 *
 * The control core and message layer only ever touch this API. Swapping the
 * backend (SocketCAN on the host, bxCAN/FDCAN on an STM32) is a link-time choice
 * -- no control code changes. This is the same separation AUTOSAR draws between
 * the application and the CAN driver / MCAL.
 *
 * Classic CAN 2.0A only: 11-bit IDs, up to 8 data bytes (see docs/can_spec.md).
 */
#ifndef CAN_IFACE_H
#define CAN_IFACE_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t id;        /* 11-bit standard identifier */
    uint8_t  dlc;       /* data length, 0..8 */
    uint8_t  data[8];
} can_frame_t;

/* Backend handle. Each backend uses the field it needs; the other is unused. */
typedef struct {
    int   fd;           /* SocketCAN file descriptor (host backend) */
    void *ctx;          /* peripheral handle, e.g. FDCAN_HandleTypeDef* (STM32) */
} can_iface_t;

/* Open the bus. channel is "can0"/"vcan0" (SocketCAN) or backend-defined.
 * bitrate is informational for SocketCAN (set via `ip link`). Returns false on
 * error. */
bool can_open(can_iface_t *it, const char *channel, uint32_t bitrate);

/* Transmit one frame. Returns false on error. */
bool can_send(can_iface_t *it, const can_frame_t *frame);

/* Non-blocking receive. Returns true and fills *frame if one was available,
 * false if the RX queue was empty. */
bool can_recv(can_iface_t *it, can_frame_t *frame);

void can_close(can_iface_t *it);

#endif /* CAN_IFACE_H */
