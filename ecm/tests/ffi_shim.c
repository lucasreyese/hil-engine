/*
 * ffi_shim.c -- struct sizes for the ctypes test harness (the Python tests).
 *
 * Not part of the firmware. It lets the Python co-simulation allocate ecm_t /
 * ecm_config_t as opaque blobs of exactly the right size and assert that its
 * ctypes mirrors of ecm_inputs_t / ecm_outputs_t match the C layout.
 */
#include <stddef.h>
#include "ecm.h"

size_t ecm_sizeof(void)         { return sizeof(ecm_t); }
size_t ecm_config_sizeof(void)  { return sizeof(ecm_config_t); }
size_t ecm_inputs_sizeof(void)  { return sizeof(ecm_inputs_t); }
size_t ecm_outputs_sizeof(void) { return sizeof(ecm_outputs_t); }
