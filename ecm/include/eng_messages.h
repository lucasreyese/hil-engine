/*
 * eng_messages.h -- CAN frame IDs and (de)serialization.
 *
 * Hand-written mirror of docs/engine.dbc. The DBC is the source of truth; if you
 * change scaling there, change it here too (the byte-compat cross-check in the
 * test suite will catch drift). All signals are little-endian; see can_spec.md.
 */
#ifndef ENG_MESSAGES_H
#define ENG_MESSAGES_H

#include <stdint.h>
#include "ecm.h"

/* Sensor frames (engine -> ECM) */
#define ID_ENG_SPEED       0x100u
#define ID_ENG_AIR         0x110u
#define ID_ENG_THERMAL     0x120u
#define ID_ENG_COMBUSTION  0x130u
/* Actuator/telemetry frames (ECM -> engine) */
#define ID_ECM_ACTUATORS   0x200u
#define ID_ECM_STATUS      0x210u

/* Decode a received sensor frame into the input snapshot. Unknown IDs are
 * ignored. Returns true if the frame was a recognized sensor frame. */
bool eng_msg_decode(uint32_t id, const uint8_t *data, uint8_t dlc, ecm_inputs_t *in);

/* Encode outgoing frames. Each returns the data length (DLC). */
uint8_t eng_msg_encode_actuators(const ecm_outputs_t *out, uint8_t data[8]);
uint8_t eng_msg_encode_status(const ecm_outputs_t *out, uint8_t data[8]);

#endif /* ENG_MESSAGES_H */
