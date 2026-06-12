"""
ctypes bindings to build/libecm.so.

Lets the Python tests drive the *real compiled C ECM* -- the same object code
that links into ecm_host -- so the control logic and the on-wire packing are
exercised directly, no CAN bus required. ecm_config_t / ecm_t are treated as
opaque blobs (sized via the ffi_shim) so we never have to mirror their internal
layout; only the flat inputs/outputs structs are mirrored.
"""

import ctypes
import os

_HERE = os.path.dirname(os.path.abspath(__file__))
_LIB = os.path.join(_HERE, "..", "build", "libecm.so")

c_float = ctypes.c_float


class Inputs(ctypes.Structure):
    _fields_ = [
        ("rpm", c_float), ("rpm_instant", c_float), ("crank_angle_deg", c_float),
        ("maf_g_s", c_float), ("tps_pct", c_float), ("iat_c", c_float),
        ("ect_c", c_float), ("lambda_", c_float), ("knock", c_float),
    ]


class Outputs(ctypes.Structure):
    _fields_ = [
        ("ignition_advance_deg", c_float), ("injector_pw_ms", c_float),
        ("fuel_trim_pct", c_float), ("cold_start_enrich_pct", c_float),
        ("idle_target_rpm", c_float),
        ("state", ctypes.c_uint8), ("flags", ctypes.c_uint8),
        ("knock_retard_deg", c_float), ("target_lambda", c_float),
    ]


class ECM:
    """Thin wrapper around an opaque ecm_t + its config."""

    def __init__(self, lib_path: str = _LIB, dt_s: float = 0.010):
        self.lib = ctypes.CDLL(os.path.abspath(lib_path))
        self._bind()
        # Layout sanity: our ctypes mirrors must match the C structs exactly.
        assert ctypes.sizeof(Inputs) == self.lib.ecm_inputs_sizeof(), "Inputs layout mismatch"
        assert ctypes.sizeof(Outputs) == self.lib.ecm_outputs_sizeof(), "Outputs layout mismatch"

        self._cfg = ctypes.create_string_buffer(self.lib.ecm_config_sizeof())
        self._ecm = ctypes.create_string_buffer(self.lib.ecm_sizeof())
        self.lib.ecm_default_config(self._cfg, c_float(dt_s))
        self.lib.ecm_init(self._ecm, self._cfg)
        self.out = Outputs()

    def _bind(self):
        L = self.lib
        for name in ("ecm_sizeof", "ecm_config_sizeof", "ecm_inputs_sizeof",
                     "ecm_outputs_sizeof"):
            getattr(L, name).restype = ctypes.c_size_t
        L.ecm_default_config.argtypes = [ctypes.c_void_p, c_float]
        L.ecm_init.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        L.ecm_step.argtypes = [ctypes.c_void_p, ctypes.POINTER(Inputs),
                               ctypes.POINTER(Outputs)]
        L.eng_msg_encode_actuators.argtypes = [ctypes.POINTER(Outputs), ctypes.c_char_p]
        L.eng_msg_encode_actuators.restype = ctypes.c_uint8
        L.eng_msg_encode_status.argtypes = [ctypes.POINTER(Outputs), ctypes.c_char_p]
        L.eng_msg_encode_status.restype = ctypes.c_uint8
        L.eng_msg_decode.argtypes = [ctypes.c_uint32, ctypes.c_char_p, ctypes.c_uint8,
                                     ctypes.POINTER(Inputs)]
        L.eng_msg_decode.restype = ctypes.c_bool

    def step(self, inp: Inputs) -> Outputs:
        self.lib.ecm_step(self._ecm, ctypes.byref(inp), ctypes.byref(self.out))
        return self.out

    # -- direct access to the C packing layer (for byte-compat tests) -----
    def encode_actuators(self, out: Outputs) -> bytes:
        buf = ctypes.create_string_buffer(8)
        n = self.lib.eng_msg_encode_actuators(ctypes.byref(out), buf)
        return buf.raw[:n]

    def encode_status(self, out: Outputs) -> bytes:
        buf = ctypes.create_string_buffer(8)
        n = self.lib.eng_msg_encode_status(ctypes.byref(out), buf)
        return buf.raw[:n]

    def decode(self, can_id: int, data: bytes, into: Inputs = None) -> Inputs:
        inp = into if into is not None else Inputs()
        self.lib.eng_msg_decode(can_id, data, len(data), ctypes.byref(inp))
        return inp
