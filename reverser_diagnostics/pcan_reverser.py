"""PCAN-based diagnostics logic for the reverser board (multi-step, with per-channel expectations)."""

import time
import logging
from typing import Dict, List, Optional, Tuple

from PCANBasic import *  # type: ignore

LOG = logging.getLogger(__name__)

# ---------- CAN / protocol constants (adapt to match your firmware) ----------

HLC_ID = 0x08          # our source (PC/HLC)
REVERSER_ID = 0x0A     # reverser board

PAYLOAD_DRIVER_STATE_COMMAND = 48   # 0x30
PAYLOAD_REVERSER_TELEMETRY   = 44   # 0x2C
PAYLOAD_ANALOG_TELEMETRY     = 62   # 0x3E
PAYLOAD_REVERSER_COMMAND     = 41   # 0x29
PAYLOAD_RESET_COMMAND        = 34   # 0x22

DRIVER_STATE_INIT        = 1
DRIVER_STATE_MANUAL      = 2
DRIVER_STATE_OPERATIONAL = 3
DRIVER_STATE_ERROR       = 4
DRIVER_STATE_EMERGENCY   = 5

GEAR_NEUTRAL = 1
GEAR_FORWARD = 2
GEAR_REVERSE = 3

MAX_ANALOG_CHANNELS = 8
MAX_CHANNELS_PER_MESSAGE = 3

ANALOG_CHANNEL_NAMES = [
    "CH_A_OUT_SNS",
    "CH_B_OUT_SNS",
    "CH_C_OUT_SNS",
    "CH_D_OUT_SNS",
    "CH_E_OUT_SNS",
    "CH_F_OUT_SNS",
    "CH_G_OUT_SNS",
    "CH_H_OUT_SNS",
]

STATUS_FLAG_NAMES = {
    0: "TRACTOR_HARNESS_OK",
    1: "MAIN_BYPASS_ENABLED",
    2: "AUX_BYPASS_ENABLED",
    3: "KA_BYPASS_ENABLED",
    4: "AUX_ALIVE",
    5: "EMERGENCY_SEQUENCE_ACTIVE",
    6: "FIRST_AUX_COMM",
    7: "AUX_SW_VERSION_OK",
    8: "DIGITAL_OUTPUT_VERIFICATION_ERROR",
    9: "CLUTCH_OUTPUT_ERROR",
    10: "LSD_VERIFICATION_ERROR",
    11: "DRIVE_GEAR_CONDITION_ERROR",
    12: "RESERVED_12",
    13: "RESERVED_13",
    14: "RESERVED_14",
    15: "RESERVED_15",
}

# Bits that are treated as "errors" when set.
ERROR_BITS = {8, 9, 10, 11}  # plus we also watch bit 5 (emergency)

# ---------- Expected analog ranges / values ----------

# Manual: 2.5V ± 5%
MANUAL_TARGET_MV = 2500
MANUAL_TOLERANCE_PCT = 5.0
MANUAL_TOL_MV = int(MANUAL_TARGET_MV * MANUAL_TOLERANCE_PCT / 100.0)
MANUAL_MIN_MV = MANUAL_TARGET_MV - MANUAL_TOL_MV
MANUAL_MAX_MV = MANUAL_TARGET_MV + MANUAL_TOL_MV

# Operational before Forward (your values)
OP_BASE_EXPECTED_MV: Dict[str, int] = {
    # A–E, H: 1.2V
    "CH_A_OUT_SNS": 1200,
    "CH_B_OUT_SNS": 1200,
    "CH_C_OUT_SNS": 1200,
    "CH_D_OUT_SNS": 1200,
    "CH_E_OUT_SNS": 1200,
    "CH_H_OUT_SNS": 1200,
    # F: 0V
    "CH_F_OUT_SNS": 0,
    # G: 3.69V
    "CH_G_OUT_SNS": 3690,
}

# Operational after Forward (your values)
OP_FWD_EXPECTED_MV: Dict[str, int] = {
    # A–E, H: 2.8V
    "CH_A_OUT_SNS": 2800,
    "CH_B_OUT_SNS": 2800,
    "CH_C_OUT_SNS": 2800,
    "CH_D_OUT_SNS": 2800,
    "CH_E_OUT_SNS": 2800,
    "CH_H_OUT_SNS": 2800,
    # F: 3.1V
    "CH_F_OUT_SNS": 3100,
    # G: 1.8V
    "CH_G_OUT_SNS": 1800,
}

# Operational after Reverse (your values; same as base)
OP_REV_EXPECTED_MV: Dict[str, int] = {
    # A–E, H: 1.2V
    "CH_A_OUT_SNS": 1200,
    "CH_B_OUT_SNS": 1200,
    "CH_C_OUT_SNS": 1200,
    "CH_D_OUT_SNS": 1200,
    "CH_E_OUT_SNS": 1200,
    "CH_H_OUT_SNS": 1200,
    # F: 0V
    "CH_F_OUT_SNS": 0,
    # G: 3.69V
    "CH_G_OUT_SNS": 3690,
}

OP_TOLERANCE_PCT = 10.0  # same ±10% for operational expectations


def build_can_id(source_id: int, dest_id: int, payload_type: int) -> int:
    """Compose a 29-bit extended CAN ID matching your existing firmware layout."""
    return (source_id |
            (dest_id << 8) |
            (payload_type << 16))  # priority/flags = 0


DRIVER_STATE_COMMAND_ID = build_can_id(HLC_ID, REVERSER_ID,
                                       PAYLOAD_DRIVER_STATE_COMMAND)
REVERSER_COMMAND_ID     = build_can_id(HLC_ID, REVERSER_ID,
                                       PAYLOAD_REVERSER_COMMAND)
RESET_COMMAND_ID        = build_can_id(HLC_ID, REVERSER_ID,
                                       PAYLOAD_RESET_COMMAND)

try:
    EXT_FLAG = int(PCAN_MESSAGE_EXTENDED.value)
except AttributeError:
    EXT_FLAG = int(PCAN_MESSAGE_EXTENDED)

DEFAULT_RANGE = (0, 5000)

ANALOG_EXPECTED_RANGE_MV: Dict[str, Tuple[int, int]] = {
    name: DEFAULT_RANGE for name in ANALOG_CHANNEL_NAMES
}


def _in_pct_range(mv: Optional[int], expected: int, pct: float) -> bool:
    """Check mv is within ±pct% of expected."""
    if mv is None:
        return False
    tol = int(expected * pct / 100.0)
    return (expected - tol) <= mv <= (expected + tol)


class ReverserDiagnostics:
    """Headless diagnostics for the reverser board, multi-step with per-channel expectations."""

    def __init__(self,
                 channel: TPCANHandle = PCAN_USBBUS1,
                 bitrate: TPCANBaudrate = PCAN_BAUD_250K):
        self.pcan = PCANBasic()
        self.channel = channel
        self.bitrate = bitrate

        # aggregated analog state
        self.analog_channels_mv: List[Optional[int]] = [None] * MAX_ANALOG_CHANNELS
        self.analog_supply_mv: Optional[int] = None
        self.last_analog_seq: Optional[int] = None
        self.last_active_mask: Optional[int] = None

        # latest telemetry snapshot
        self.last_status_flags: int = 0
        self.status_flags_agg: int = 0
        self.last_driver_state: Optional[int] = None
        self.last_desired_gear: Optional[int] = None
        self.last_actual_gear: Optional[int] = None

        # connectivity flags
        self.got_reverser_telemetry: bool = False
        self.got_analog_telemetry: bool = False

        # current driver-state command (keep-alive value)
        self.current_driver_state_cmd: int = DRIVER_STATE_OPERATIONAL

        self.connected: bool = False

    # ----- CAN connection -----

    def connect(self) -> None:
        if self.connected:
            return
        result = self.pcan.Initialize(self.channel, self.bitrate)
        if result != PCAN_ERROR_OK:
            # Get error text for better diagnostics
            error_code = int(result)
            error_msg = f"Error code: 0x{error_code:08X}"
            
            # Try to get descriptive error text
            try:
                error_text_result = self.pcan.GetErrorText(result, 0)
                if error_text_result and len(error_text_result) >= 2:
                    if int(error_text_result[0]) == int(PCAN_ERROR_OK):
                        if isinstance(error_text_result[1], bytes):
                            error_msg = error_text_result[1].decode('utf-8', errors='ignore')
                        else:
                            error_msg = str(error_text_result[1])
            except Exception as e:
                # If GetErrorText fails, use the error code
                LOG.debug("Could not get error text: %s", e)
            
            # Provide helpful suggestions based on common error codes
            suggestions = []
            # PCAN_ERROR_INITIALIZE = 0x4000000 (67108864 decimal)
            if error_code in (0x4000000, 0x04000000, 67108864):  
                suggestions.append("Channel may already be initialized or hardware not available")
                suggestions.append("Try disconnecting other applications using the PCAN device")
                suggestions.append("Check if PCAN device is physically connected")
                suggestions.append("Verify the channel number is correct (default: PCAN_USBBUS1)")
            elif error_code == 0x00200:  # PCAN_ERROR_NODRIVER
                suggestions.append("PCAN driver may not be installed or loaded")
            elif error_code == 0x01400:  # PCAN_ERROR_ILLHW
                suggestions.append("Invalid hardware handle - check if PCAN device is connected")
            
            full_error = f"PCAN init failed: {error_msg}"
            if suggestions:
                full_error += "\nSuggestions:\n  - " + "\n  - ".join(suggestions)
            
            raise RuntimeError(full_error)
        self.connected = True
        LOG.info("PCAN connected on channel %s, bitrate %s", self.channel, self.bitrate)

    def disconnect(self) -> None:
        if not self.connected:
            return
        self.pcan.Uninitialize(self.channel)
        self.connected = False
        LOG.info("PCAN disconnected")

    # ----- Transmit helpers -----

    def send_driver_state(self) -> None:
        """Send the current driver state command as a keep-alive."""
        if not self.connected:
            return

        msg = TPCANMsg()
        msg.ID = DRIVER_STATE_COMMAND_ID
        msg.MSGTYPE = PCAN_MESSAGE_EXTENDED
        msg.LEN = 1
        msg.DATA[0] = self.current_driver_state_cmd
        for i in range(1, 8):
            msg.DATA[i] = 0

        res = self.pcan.Write(self.channel, msg)
        if res != PCAN_ERROR_OK:
            LOG.error("Write error (driver state): 0x%08X", int(res))

    def send_gear_command(self, gear_mode: int) -> None:
        """Send a simple gear command.

        NOTE: This is a best-guess placeholder assuming the first data byte
        is the desired gear. Adjust to match your real ReverserCommand struct.
        """
        if not self.connected:
            return

        msg = TPCANMsg()
        msg.ID = REVERSER_COMMAND_ID
        msg.MSGTYPE = PCAN_MESSAGE_EXTENDED
        msg.LEN = 1
        msg.DATA[0] = gear_mode
        for i in range(1, 8):
            msg.DATA[i] = 0

        res = self.pcan.Write(self.channel, msg)
        if res != PCAN_ERROR_OK:
            LOG.error("Write error (gear command): 0x%08X", int(res))

    def send_reset(self) -> None:
        """Send a reset command to the reverser board."""
        if not self.connected:
            return

        msg = TPCANMsg()
        msg.ID = RESET_COMMAND_ID
        msg.MSGTYPE = PCAN_MESSAGE_EXTENDED
        msg.LEN = 1
        msg.DATA[0] = 0  # Reset command (adjust if needed)
        for i in range(1, 8):
            msg.DATA[i] = 0

        res = self.pcan.Write(self.channel, msg)
        if res != PCAN_ERROR_OK:
            LOG.error("Write error (reset command): 0x%08X", int(res))

    # ----- Receive + decode -----

    def _apply_analog_message(self, active_mask: int, msg_index: int,
                              values: List[int]) -> None:
        values_filled = 0
        channel_position = 0
        start_position = msg_index * MAX_CHANNELS_PER_MESSAGE

        for bit in range(MAX_ANALOG_CHANNELS):
            if active_mask & (1 << bit):
                if (channel_position >= start_position and
                        values_filled < MAX_CHANNELS_PER_MESSAGE):
                    mv = values[values_filled]
                    self.analog_channels_mv[bit] = mv
                    values_filled += 1
                channel_position += 1

        # TRC_Supply_SNS is the 9th value (trc_5v_sns from firmware)
        # With 8 channels + 1 supply = 9 values total, sent in 3 messages:
        # seq=0: channels 0,1,2 (positions 0,1,2)
        # seq=1: channels 3,4,5 (positions 3,4,5)
        # seq=2: channels 6,7, supply (positions 6,7,8)
        # So TRC_Supply_SNS is always values[2] (the last value, index -1) in seq=2
        if msg_index == 2 and len(values) >= 3:
            self.analog_supply_mv = values[2]  # Always the 3rd value (index 2) in seq=2
            LOG.debug("TRC_Supply_SNS captured: seq=%d, values=%s, supply=%d", 
                     msg_index, values, self.analog_supply_mv)
        elif msg_index == 2:
            LOG.debug("TRC_Supply_SNS: seq=2 but len(values)=%d < 3", len(values))

    def _handle_analog_telemetry(self, can_id: int, data: List[int]) -> None:
        if len(data) < 8:
            return

        active_mask = data[0]
        seq = data[1]
        v0 = data[2] | (data[3] << 8)
        v1 = data[4] | (data[5] << 8)
        v2 = data[6] | (data[7] << 8)
        values = [v0, v1, v2]

        self.last_active_mask = active_mask
        self.got_analog_telemetry = True

        if self.last_analog_seq is None or seq == 0:
            self.analog_channels_mv = [None] * MAX_ANALOG_CHANNELS
            self.analog_supply_mv = None

        self._apply_analog_message(active_mask, seq, values)
        self.last_analog_seq = seq

    def _handle_reverser_telemetry(self, can_id: int, data: List[int]) -> None:
        if len(data) < 7:
            return

        desired_gear = data[0]
        actual_gear  = data[1]
        driver_state = data[2]
        status_flags = data[3] | (data[4] << 8)

        self.last_desired_gear = desired_gear
        self.last_actual_gear = actual_gear
        self.last_driver_state = driver_state
        self.last_status_flags = status_flags
        self.status_flags_agg |= status_flags

        self.got_reverser_telemetry = True

        LOG.debug(
            "ReverserTelemetry: desired=%d actual=%d state=%d flags=0x%04X",
            desired_gear, actual_gear, driver_state, status_flags
        )

    def _read_can_once(self) -> None:
        if not self.connected:
            return

        while True:
            result, msg, timestamp = self.pcan.Read(self.channel)

            if result == PCAN_ERROR_QRCVEMPTY:
                break
            if result != PCAN_ERROR_OK:
                LOG.error("CAN read error: 0x%08X", int(result))
                break

            msg_type = int(msg.MSGTYPE)
            msg_len  = int(msg.LEN)
            can_id   = int(msg.ID)

            if not (msg_type & EXT_FLAG):
                continue

            source_id    = can_id & 0xFF
            payload_type = (can_id >> 16) & 0xFF

            if source_id != REVERSER_ID:
                continue

            data = [int(msg.DATA[i]) for i in range(msg_len)]

            if payload_type == PAYLOAD_REVERSER_TELEMETRY:
                self._handle_reverser_telemetry(can_id, data)
            elif payload_type == PAYLOAD_ANALOG_TELEMETRY:
                self._handle_analog_telemetry(can_id, data)

    # ----- Utility loops -----

    def _poll_for(self, duration_s: float, keepalive: bool = True) -> None:
        """Run read loop for given duration, optionally sending keepalive."""
        start = time.time()
        last_keepalive = 0.0
        while time.time() - start < duration_s:
            now = time.time()
            if keepalive and (now - last_keepalive) >= 0.2:
                self.send_driver_state()
                last_keepalive = now
            self._read_can_once()
            time.sleep(0.01)

    def _sample_analog_values(self, duration_s: float, num_samples: int = 10) -> Dict[str, List[Optional[int]]]:
        """Sample analog channel values multiple times and return lists of samples.
        
        Returns a dict mapping channel names to lists of sampled values.
        """
        samples: Dict[str, List[Optional[int]]] = {
            name: [] for name in ANALOG_CHANNEL_NAMES
        }
        
        if num_samples <= 0:
            num_samples = 1
        
        interval = duration_s / num_samples
        start = time.time()
        last_keepalive = start - 0.1  # Allow immediate keepalive
        next_sample_time = interval
        
        while len(samples[ANALOG_CHANNEL_NAMES[0]]) < num_samples:
            now = time.time()
            elapsed = now - start
            
            # Send keepalive periodically
            if now - last_keepalive >= 0.1:
                self.send_driver_state()
                last_keepalive = now
            
            # Read CAN messages continuously
            self._read_can_once()
            
            # Sample at regular intervals
            if elapsed >= next_sample_time:
                for idx, name in enumerate(ANALOG_CHANNEL_NAMES):
                    samples[name].append(self.analog_channels_mv[idx])
                next_sample_time += interval
            
            # Small sleep to prevent CPU spinning
            time.sleep(0.001)
        
        return samples

    def _get_averaged_values(self, samples: Dict[str, List[Optional[int]]]) -> Dict[str, Optional[int]]:
        """Calculate averaged values from samples, ignoring None values."""
        averaged = {}
        for name, sample_list in samples.items():
            valid_samples = [s for s in sample_list if s is not None]
            if valid_samples:
                avg = int(sum(valid_samples) / len(valid_samples))
                averaged[name] = avg
            else:
                averaged[name] = None
        return averaged

    # ----- Individual steps -----

    def step_reset(self, timeout_s: float = 5.0) -> Dict:
        """Step 0: Reset and wait for reconnection.

        Sends reset command, stops all telemetry (including keep-alive) for 5 seconds,
        then waits for both ReverserTelemetry and AnalogTelemetry to resume.
        """
        # Clear telemetry flags
        self.got_reverser_telemetry = False
        self.got_analog_telemetry = False

        # Send reset command
        self.send_reset()

        # Stop all telemetry for 5 seconds - no keep-alive, no CAN reading
        time.sleep(5.0)

        # Now wait for reconnection (after the 5 second silence period)
        start = time.time()
        end_time = start + timeout_s
        last_keepalive = start - 0.1  # Initialize to allow immediate keepalive
        while time.time() < end_time:
            now = time.time()
            if now - last_keepalive >= 0.1:
                self.send_driver_state()
                last_keepalive = now
            self._read_can_once()
            if self.got_reverser_telemetry and self.got_analog_telemetry:
                break

        ok = self.got_reverser_telemetry and self.got_analog_telemetry
        reason = "OK" if ok else "Missing telemetry after reset: " + ", ".join(
            name for ok_flag, name in [
                (self.got_reverser_telemetry, "ReverserTelemetry"),
                (self.got_analog_telemetry, "AnalogTelemetry"),
            ] if not ok_flag
        )
        return {"pass": ok, "reason": reason}

    def step_connectivity(self, timeout_s: float = 3.0) -> Dict:
        """Step 1: Connectivity check.

        Expect both ReverserTelemetry and AnalogTelemetry within timeout.
        If telemetry was already received from reset step, verify it continues to flow.
        Handles cases where board may have lost power (e.g., TRC supply went to 0V).
        """
        # First, read some CAN messages to check TRC supply voltage
        # This helps detect if board is powered down
        for _ in range(10):  # Read a few messages to get TRC supply reading
            self._read_can_once()
            time.sleep(0.05)
        
        # Check if TRC supply is low (board may be powered down)
        # If so, wait longer for board to power up
        trc_supply_low = False
        if self.analog_supply_mv is not None and self.analog_supply_mv < 100:
            trc_supply_low = True
            # Wait for board to power up (TRC supply to come back)
            LOG.info("TRC supply is low (%dmV), waiting for board to power up...", self.analog_supply_mv)
            power_up_start = time.time()
            power_up_timeout = 5.0  # Wait up to 5 seconds for power up
            while time.time() < power_up_start + power_up_timeout:
                self._read_can_once()
                if self.analog_supply_mv is not None and self.analog_supply_mv >= 100:
                    LOG.info("TRC supply restored (%dmV), board powered up", self.analog_supply_mv)
                    break
                time.sleep(0.1)
        
        # Clear flags to ensure we get fresh telemetry
        # This handles the case where board may have reset due to power loss
        self.got_reverser_telemetry = False
        self.got_analog_telemetry = False
        
        # If we waited for power up, give extra time for telemetry to start
        if trc_supply_low:
            timeout_s = max(timeout_s, 5.0)  # At least 5 seconds after power up
        
        start = time.time()
        end_time = start + timeout_s
        last_keepalive = start - 0.1
        
        while time.time() < end_time:
            now = time.time()
            if now - last_keepalive >= 0.1:
                self.send_driver_state()
                last_keepalive = now
            self._read_can_once()
            
            # If we receive both types of telemetry, we're good
            if self.got_reverser_telemetry and self.got_analog_telemetry:
                return {"pass": True, "reason": "OK"}

        ok = self.got_reverser_telemetry and self.got_analog_telemetry
        reason = "OK" if ok else "Missing telemetry: " + ", ".join(
            name for ok_flag, name in [
                (self.got_reverser_telemetry, "ReverserTelemetry"),
                (self.got_analog_telemetry, "AnalogTelemetry"),
            ] if not ok_flag
        )
        return {"pass": ok, "reason": reason}

    def step_configuration(self) -> Dict:
        """Step 2: Test station configuration (active_mask == 0xFF)."""
        mask = self.last_active_mask
        if mask is None:
            return {"pass": False, "active_mask": None,
                    "reason": "No AnalogTelemetry received yet"}
        ok = (mask == 0xFF)
        reason = "OK" if ok else f"Active mask is 0x{mask:02X}, expected 0xFF"
        return {"pass": ok, "active_mask": mask, "reason": reason}

    def step_manual(self, duration_s: float = 1.0) -> Dict:
        """Step 3: Manual mode: all channels ~2.5 V ± 5%."""
        self.current_driver_state_cmd = DRIVER_STATE_MANUAL
        self._poll_for(duration_s, keepalive=True)

        manual_values = {name: self.analog_channels_mv[idx]
                         for idx, name in enumerate(ANALOG_CHANNEL_NAMES)}

        per_channel_ok: Dict[str, bool] = {}
        failed: List[str] = []

        for ch_name, mv in manual_values.items():
            if mv is None:
                per_channel_ok[ch_name] = False
                failed.append(ch_name)
                continue
            ok = (MANUAL_MIN_MV <= mv <= MANUAL_MAX_MV)
            per_channel_ok[ch_name] = ok
            if not ok:
                failed.append(ch_name)

        overall = len(failed) == 0
        return {
            "pass": overall,
            "failed_channels": failed,
            "per_channel_ok": per_channel_ok,
            "values": manual_values,
        }

    def _decode_error_bits(self, flags: int) -> List[str]:
        errors: List[str] = []
        for bit in ERROR_BITS.union({5}):
            if flags & (1 << bit):
                errors.append(STATUS_FLAG_NAMES.get(bit, f"BIT_{bit}"))
        return errors

    def step_operational(self, duration_s: float = 1.0) -> Dict:
        """Step 4: Operational mode: check status flags AND per-channel op baseline."""
        self.current_driver_state_cmd = DRIVER_STATE_OPERATIONAL
        self.status_flags_agg = 0
        self._poll_for(duration_s, keepalive=True)

        # Status flags
        errors = self._decode_error_bits(self.status_flags_agg)
        flags_ok = len(errors) == 0

        # Per-channel baseline op values
        op_values = {name: self.analog_channels_mv[idx]
                     for idx, name in enumerate(ANALOG_CHANNEL_NAMES)}
        per_channel_ok: Dict[str, bool] = {}
        analog_failed: List[str] = []

        for ch_name in ANALOG_CHANNEL_NAMES:
            mv = op_values.get(ch_name)
            expected = OP_BASE_EXPECTED_MV[ch_name]
            ok = _in_pct_range(mv, expected, OP_TOLERANCE_PCT)
            per_channel_ok[ch_name] = ok
            if not ok:
                analog_failed.append(ch_name)

        analog_ok = len(analog_failed) == 0

        ok = flags_ok and analog_ok

        reasons = []
        if not flags_ok:
            reasons.append("Errors: " + ", ".join(errors))
        if not analog_ok:
            reasons.append("Bad op baseline on: " + ", ".join(analog_failed))
        reason = "OK" if ok else " | ".join(reasons)

        return {
            "pass": ok,
            "error_bits_set": errors,
            "flags": self.status_flags_agg,
            "reason": reason,
            "values": op_values,
            "per_channel_ok": per_channel_ok,
        }

    def step_forward(self, manual_values: Dict[str, Optional[int]],
                     duration_s: float = 2.0) -> Dict:
        """Step 5: All channels ON.

        - Send FORWARD command
        - Wait until actual_gear == GEAR_FORWARD (or timeout)
        - Then wait a settling window and check per-channel forward expectations.
        """
        # 1) Command forward
        self.send_gear_command(GEAR_FORWARD)

        # 2) Wait for gear to actually become forward
        gear_ok = self._wait_for_gear(GEAR_FORWARD, timeout_s=3.0)

        # 3) Sample values multiple times during settling for stability
        samples = self._sample_analog_values(duration_s, num_samples=10)
        forward_values = self._get_averaged_values(samples)

        per_channel_changed: Dict[str, bool] = {}
        failed: List[str] = []

        for ch_name in ANALOG_CHANNEL_NAMES:
            mv = forward_values.get(ch_name)
            expected = OP_FWD_EXPECTED_MV[ch_name]
            ok = _in_pct_range(mv, expected, OP_TOLERANCE_PCT)
            per_channel_changed[ch_name] = ok
            if not ok:
                failed.append(ch_name)

        ok_all = gear_ok and (len(failed) == 0)

        reasons = []
        if not gear_ok:
            reasons.append("gear did not reach FORWARD")
        if failed:
            reasons.append("Forward mismatch on: " + ", ".join(failed))

        reason = "OK" if ok_all else " | ".join(reasons)

        return {
            "pass": ok_all,
            "failed_channels": failed,
            "per_channel_changed": per_channel_changed,
            "values": forward_values,
            "reason": reason,
        }

    def step_reverse(self, manual_values: Dict[str, Optional[int]],
                     duration_s: float = 2.0) -> Dict:
        """Step 6: All channels OFF.

        - Send REVERSE command
        - Wait until actual_gear == GEAR_REVERSE (or timeout)
        - Then wait settling window and check per-channel reverse expectations.
        """
        # 1) Command reverse
        self.send_gear_command(GEAR_REVERSE)

        # 2) Wait for gear to actually become reverse
        gear_ok = self._wait_for_gear(GEAR_REVERSE, timeout_s=3.0)

        # 3) Sample values multiple times during settling for stability
        samples = self._sample_analog_values(duration_s, num_samples=10)
        reverse_values = self._get_averaged_values(samples)

        per_channel_back: Dict[str, bool] = {}
        failed: List[str] = []

        for ch_name in ANALOG_CHANNEL_NAMES:
            mv = reverse_values.get(ch_name)
            expected = OP_REV_EXPECTED_MV[ch_name]
            ok = _in_pct_range(mv, expected, OP_TOLERANCE_PCT)
            per_channel_back[ch_name] = ok
            if not ok:
                failed.append(ch_name)

        ok_all = gear_ok and (len(failed) == 0)

        reasons = []
        if not gear_ok:
            reasons.append("gear did not reach REVERSE")
        if failed:
            reasons.append("Reverse mismatch on: " + ", ".join(failed))

        reason = "OK" if ok_all else " | ".join(reasons)

        return {
            "pass": ok_all,
            "failed_channels": failed,
            "per_channel_back": per_channel_back,
            "values": reverse_values,
            "reason": reason,
        }

    def step_trc_supply_switch(self, timeout_s: float = 15.0) -> Dict:
        """Step: TRC_Supply_SNS switch toggle test.
        
        Prompts user to toggle TRC supply switch and monitors for voltage change.
        Expects voltage to change from 5V ±10% (4500-5500mV) to 0V or vice versa.
        
        Args:
            timeout_s: Maximum time to wait for change (default: 15.0 seconds)
        """
        import sys
        
        # Voltage thresholds (10% tolerance)
        HIGH_VOLTAGE_MIN_MV = 4500  # 5V - 10% = 4.5V
        HIGH_VOLTAGE_MAX_MV = 5500  # 5V + 10% = 5.5V
        LOW_VOLTAGE_MAX_MV = 100   # Consider <100mV as 0V (with some noise margin)
        
        # Get initial voltage state from supply voltage
        initial_voltage = self.analog_supply_mv
        
        # Determine initial state: "HIGH" (~5V), "LOW" (~0V), or "UNKNOWN"
        initial_state = None
        if initial_voltage is not None:
            if HIGH_VOLTAGE_MIN_MV <= initial_voltage <= HIGH_VOLTAGE_MAX_MV:
                initial_state = "HIGH"
            elif initial_voltage <= LOW_VOLTAGE_MAX_MV:
                initial_state = "LOW"
            else:
                initial_state = "UNKNOWN"
        
        # Prompt user
        print("\n" + "="*60)
        print("TRC_Supply_SNS Switch Test")
        print("="*60)
        if initial_state:
            print(f"Current TRC_Supply_SNS voltage: {initial_voltage}mV ({initial_state})")
        else:
            print("Waiting for initial TRC_Supply_SNS voltage reading...")
        print("Please toggle the TRC supply switch now...")
        print("Waiting for voltage change (timeout: {}s)...".format(timeout_s))
        sys.stdout.flush()
        
        # Poll for changes
        start = time.time()
        end_time = start + timeout_s
        last_keepalive = start - 0.1
        change_detected = False
        initial_state_captured = False
        
        while time.time() < end_time:
            now = time.time()
            if now - last_keepalive >= 0.1:
                self.send_driver_state()
                last_keepalive = now
            
            self._read_can_once()
            
            current_voltage = self.analog_supply_mv
            
            # Capture initial state once we have a valid reading
            if not initial_state_captured and current_voltage is not None:
                if HIGH_VOLTAGE_MIN_MV <= current_voltage <= HIGH_VOLTAGE_MAX_MV:
                    initial_state = "HIGH"
                    initial_voltage = current_voltage
                    initial_state_captured = True
                elif current_voltage <= LOW_VOLTAGE_MAX_MV:
                    initial_state = "LOW"
                    initial_voltage = current_voltage
                    initial_state_captured = True
                # If voltage is in between, wait for it to settle
            
            # Check for change
            if initial_state_captured and current_voltage is not None:
                current_state = None
                if HIGH_VOLTAGE_MIN_MV <= current_voltage <= HIGH_VOLTAGE_MAX_MV:
                    current_state = "HIGH"
                elif current_voltage <= LOW_VOLTAGE_MAX_MV:
                    current_state = "LOW"
                
                if current_state and current_state != initial_state:
                    change_detected = True
                    # If voltage went from HIGH to LOW, board may reset - stop telemetry
                    if initial_state == "HIGH" and current_state == "LOW":
                        # Stop all telemetry for 5 seconds to allow board to reset
                        time.sleep(5.0)
                        # Clear telemetry flags and wait for reconnection
                        self.got_reverser_telemetry = False
                        self.got_analog_telemetry = False
                        reconnect_start = time.time()
                        reconnect_timeout = 5.0
                        reconnect_end = reconnect_start + reconnect_timeout
                        reconnect_keepalive = reconnect_start - 0.1
                        while time.time() < reconnect_end:
                            now = time.time()
                            if now - reconnect_keepalive >= 0.1:
                                self.send_driver_state()
                                reconnect_keepalive = now
                            self._read_can_once()
                            if self.got_reverser_telemetry and self.got_analog_telemetry:
                                break
                    break
            
            time.sleep(0.01)
        
        ok = change_detected
        final_voltage = self.analog_supply_mv
        final_state = None
        if final_voltage is not None:
            if HIGH_VOLTAGE_MIN_MV <= final_voltage <= HIGH_VOLTAGE_MAX_MV:
                final_state = "HIGH"
            elif final_voltage <= LOW_VOLTAGE_MAX_MV:
                final_state = "LOW"
        
        if ok:
            if initial_state == "LOW":
                reason = f"OK - TRC_Supply_SNS voltage changed from LOW ({initial_voltage}mV) to HIGH ({final_voltage}mV)"
            else:
                reason = f"OK - TRC_Supply_SNS voltage changed from HIGH ({initial_voltage}mV) to LOW ({final_voltage}mV)"
                # If we went from HIGH to LOW, verify connectivity was restored
                if not (self.got_reverser_telemetry and self.got_analog_telemetry):
                    reason += " (board reset detected, waiting for reconnection...)"
        else:
            reason = f"FAIL - No TRC_Supply_SNS voltage change detected within {timeout_s}s timeout"
            if not initial_state_captured:
                reason += " (no valid initial state captured)"
            elif final_voltage is not None:
                reason += f" (still {initial_state} at {final_voltage}mV)"
        
        return {
            "pass": ok,
            "reason": reason,
            "initial_state": initial_state,
            "initial_voltage_mv": initial_voltage,
            "final_state": final_state,
            "final_voltage_mv": final_voltage,
            "change_detected": change_detected,
        }

    # ----- High-level orchestration -----

    def run_full_diagnostics(self) -> Dict:
        """Run all 8 steps and return a rich result dict."""
        if not self.connected:
            self.connect()

        # 0) Connectivity
        connectivity = self.step_connectivity()

        # 1) Reset and wait for reconnection
        reset = self.step_reset()

        # 2) Configuration
        configuration = self.step_configuration()

        # 3) Manual
        manual = self.step_manual()
        manual_values = manual["values"]

        # 4) Operational baseline
        operational = self.step_operational()

        # 5) Forward
        forward = self.step_forward(manual_values)

        # 6) Reverse
        reverse = self.step_reverse(manual_values)

        # 7) TRC_Supply_SNS Switch toggle test
        trc_supply_switch = self.step_trc_supply_switch()

        final_pass = all(step["pass"] for step in
                         [connectivity, reset, configuration, manual, operational, forward, reverse, trc_supply_switch])

        return {
            "final_pass": final_pass,
            "connectivity": connectivity,
            "reset": reset,
            "configuration": configuration,
            "manual": manual,
            "operational": operational,
            "forward": forward,
            "reverse": reverse,
            "trc_supply_switch": trc_supply_switch,
            "manual_values": manual_values,
            "forward_values": forward.get("values", {}),
            "reverse_values": reverse.get("values", {}),
        }


    def _wait_for_gear(self, expected_gear: int, timeout_s: float = 3.0) -> bool:
        """Wait until actual_gear equals expected_gear or timeout.

        Returns True if gear reached, False on timeout.
        """
        start = time.time()
        last_keepalive = 0.0
        while time.time() - start < timeout_s:
            now = time.time()
            # keep sending current driver state
            if now - last_keepalive >= 0.2:
                self.send_driver_state()
                last_keepalive = now

            self._read_can_once()

            if self.last_actual_gear == expected_gear:
                return True

            time.sleep(0.01)

        LOG.warning(
            "Timeout waiting for gear %d (last_actual_gear=%s)",
            expected_gear,
            self.last_actual_gear,
        )
        return False

