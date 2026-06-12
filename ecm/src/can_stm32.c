/*
 * can_stm32.c -- STM32 backend for can_iface (the on-target seam).
 *
 * Compiled only for the STM32 firmware build (-DECM_TARGET_STM32); it is inert
 * on the host so the Linux build ignores it. The skeleton targets the FDCAN
 * peripheral (G0/G4/H7 -- e.g. a CANable 2.0 / STM32G431) running in CLASSIC
 * frame mode to match docs/can_spec.md. For an F1/F4/L4 with bxCAN, swap the
 * HAL_FDCAN_* calls for HAL_CAN_* equivalents; the can_iface signatures do not
 * change, and neither does any control code.
 *
 * CubeMX setup (the part that genuinely benefits from the generator):
 *   - Clocks: set the FDCAN kernel clock; size the nominal bit timing for
 *     500 kbit/s (e.g. 1 tq prescaler + seg1/seg2 for an 80% sample point).
 *   - FDCAN1: Classic mode, auto-retransmission on, 1 Tx FIFO/queue, RX FIFO0.
 *   - Accept ENG_* (0x100,0x110,0x120,0x130) into RX FIFO0; reject the rest.
 *   - NVIC: enable FDCAN1_IT0 if you prefer IRQ RX over the polling shown here.
 * Let CubeMX generate MX_FDCAN1_Init(); call ecm logic from your 10 ms tick.
 */
#ifdef ECM_TARGET_STM32

#include "can_iface.h"
#include "stm32g4xx_hal.h"   /* adjust to your device family */

/* Provided by the CubeMX-generated code. */
extern FDCAN_HandleTypeDef hfdcan1;

bool can_open(can_iface_t *it, const char *channel, uint32_t bitrate) {
    (void)channel;
    (void)bitrate;  /* bit timing comes from CubeMX/MX_FDCAN1_Init() */
    it->fd = -1;
    it->ctx = &hfdcan1;
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        return false;
    }
    return true;
}

bool can_send(can_iface_t *it, const can_frame_t *frame) {
    FDCAN_HandleTypeDef *h = (FDCAN_HandleTypeDef *)it->ctx;
    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier = frame->id & 0x7FFu;       /* standard 11-bit */
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = (uint32_t)frame->dlc << 16; /* FDCAN_DLC_BYTES_n encoding */
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(h, &tx, (uint8_t *)frame->data) == HAL_OK;
}

bool can_recv(can_iface_t *it, can_frame_t *frame) {
    FDCAN_HandleTypeDef *h = (FDCAN_HandleTypeDef *)it->ctx;
    if (HAL_FDCAN_GetRxFifoFillLevel(h, FDCAN_RX_FIFO0) == 0) {
        return false;
    }
    FDCAN_RxHeaderTypeDef rx;
    if (HAL_FDCAN_GetRxMessage(h, FDCAN_RX_FIFO0, &rx, frame->data) != HAL_OK) {
        return false;
    }
    frame->id = rx.Identifier;
    frame->dlc = (uint8_t)(rx.DataLength >> 16); /* classic: bytes == DLC */
    return true;
}

void can_close(can_iface_t *it) {
    FDCAN_HandleTypeDef *h = (FDCAN_HandleTypeDef *)it->ctx;
    if (h) {
        HAL_FDCAN_Stop(h);
    }
}

#endif /* ECM_TARGET_STM32 */
