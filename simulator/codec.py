"""
DBC-driven CAN encode/decode for the simulator.

The DBC (docs/engine.dbc) is the single source of truth for IDs and signal
scaling. The simulator loads it directly with cantools; the C ECM mirrors the
same layout by hand. Keeping all scaling in the DBC means neither side carries
magic numbers.
"""

import os

import cantools

from engine.state import ControlInputs, EngineState

DEFAULT_DBC = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs", "engine.dbc"
)

# Sensor frames the engine publishes (message name -> EngineState attributes).
_SENSOR_MAP = {
    "ENG_SPEED": {
        "EngSpeedFiltered": "rpm_filtered",
        "EngSpeedInstant": "rpm",
        "CrankAngle": "crank_angle_deg",
    },
    "ENG_AIR": {
        "MassAirFlow": "maf_g_s",
        "ThrottlePosition": "tps_pct",
        "IntakeAirTemp": "iat_c",
    },
    "ENG_THERMAL": {
        "CoolantTemp": "ect_c",
    },
    "ENG_COMBUSTION": {
        "Lambda": "lambda_",
        "KnockIntensity": "knock_intensity",
    },
}

# ECM actuator frame -> ControlInputs attributes.
_ACTUATOR_MAP = {
    "IgnitionAdvance": "ignition_advance_deg",
    "InjectorPulseWidth": "injector_pw_ms",
    "FuelTrim": "fuel_trim_pct",
    "ColdStartEnrich": "cold_start_enrich_pct",
    "IdleTargetRpm": "idle_target_rpm",
}


def _clamp_to_signal(message, name, value):
    sig = message.get_signal_by_name(name)
    if sig.minimum is not None:
        value = max(value, sig.minimum)
    if sig.maximum is not None:
        value = min(value, sig.maximum)
    return value


class Codec:
    def __init__(self, dbc_path: str = DEFAULT_DBC):
        self.db = cantools.database.load_file(dbc_path)
        self._actuator_id = self.db.get_message_by_name("ECM_ACTUATORS").frame_id
        self._status_id = self.db.get_message_by_name("ECM_STATUS").frame_id

    # -- engine -> ECM ----------------------------------------------------
    def encode_sensor(self, name: str, state: EngineState):
        """Encode one sensor frame. Returns (arbitration_id, data: bytes)."""
        msg = self.db.get_message_by_name(name)
        signals = {
            sig: _clamp_to_signal(msg, sig, getattr(state, attr))
            for sig, attr in _SENSOR_MAP[name].items()
        }
        return msg.frame_id, msg.encode(signals)

    def sensor_message_names(self):
        return list(_SENSOR_MAP.keys())

    # -- ECM -> engine ----------------------------------------------------
    def is_actuator_frame(self, can_id: int) -> bool:
        return can_id == self._actuator_id

    def decode_actuators(self, data: bytes) -> ControlInputs:
        decoded = self.db.decode_message(self._actuator_id, data)
        ctrl = ControlInputs()
        for sig, attr in _ACTUATOR_MAP.items():
            setattr(ctrl, attr, float(decoded[sig]))
        return ctrl

    def decode_status(self, data: bytes) -> dict:
        """Decode ECM_STATUS for telemetry (EcmState comes back as a string)."""
        return self.db.decode_message(self._status_id, data)

    @property
    def status_id(self) -> int:
        return self._status_id
