import math
import time
import threading
import serial

try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False
    print("WARNING: python-can not installed. CAN mode unavailable.")


# ── CAN command bytes ─────────────────────────────────────────────────────────
CMD_STOP           = 0x01
CMD_GOTO_ABS       = 0x02
CMD_GOTO_REL       = 0x03
CMD_ZERO           = 0x04
CMD_SET_ID         = 0x05
CMD_SET_KP         = 0x06
CMD_SET_MAX_SPEED  = 0x07
CMD_SET_DEADBAND   = 0x08
CMD_SET_VEL_LIMIT  = 0x09
CMD_SAVE           = 0x0A
CMD_SET_CAN_ENABLE = 0x0B


def _pack_int16_le(value: int) -> tuple[int, int]:
    value = max(-32768, min(32767, value))
    unsigned = value & 0xFFFF
    return unsigned & 0xFF, (unsigned >> 8) & 0xFF


class ActuatorController:
    def __init__(self, mode="uart", wheelbase_m=1.3, lookahead_index=5,
                 can_channel="can0", can_bustype="socketcan",
                 can_bitrate=500_000, can_arb_id=0x201):
        self.mode      = mode.lower()
        self.WHEELBASE = wheelbase_m
        self.LOOKAHEAD = lookahead_index

        self.serial_conn = None
        self.can_bus     = None
        self.lock        = threading.Lock()
        self.remote_cmd  = {}
        self.last_update_time = 0.0
        self.running     = False

        self.max_speed_var    = 15.0
        self.HZ               = 50
        self.DT               = 1.0 / self.HZ
        self.WATCHDOG_TIMEOUT = 6.0

        self.can_channel = can_channel
        self.can_bustype = can_bustype
        self.can_bitrate = can_bitrate
        self.can_arb_id  = can_arb_id

        self._setup_connection()

    # ── connection setup ───────────────────────────────────────────────────────

    def _setup_connection(self):
        try:
            if self.mode == "uart":
                self._setup_uart()
            elif self.mode == "can":
                self._setup_can()
            else:
                raise ValueError(f"Unknown mode '{self.mode}'. Use 'uart' or 'can'.")
        except Exception as e:
            print(f"ERROR Hardware setup failed: {e}")

    def _setup_uart(self):
        self.serial_conn = serial.Serial(
            port="/dev/tty_steering", baudrate=115200, timeout=0.1
        )
        print("Initializing Actuator (UART)...")
        time.sleep(2)
        self.serial_conn.write(b"\r\nN0\r\n")
        time.sleep(0.5)
        self.serial_conn.write(b"T1\r\n")
        time.sleep(0.5)

    def _setup_can(self):
        if not CAN_AVAILABLE:
            raise RuntimeError("python-can is not installed.")
        print(f"Initializing Actuator (CAN) — channel={self.can_channel} "
              f"arb_id=0x{self.can_arb_id:03X} bitrate={self.can_bitrate}")
        self.can_bus = can.interface.Bus(
            channel=self.can_channel,
            bustype=self.can_bustype,
            bitrate=self.can_bitrate,
        )

    # ── public API — callers use these, never touch transport directly ─────────

    def ingest_kappa(self, cmd):
        with self.lock:
            self.remote_cmd = cmd
            self.last_update_time = time.time()

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._control_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if hasattr(self, "thread"):
            self.thread.join()
        self.cmd_stop()
        self._close_connections()

    def cmd_stop(self):
        self._dispatch("stop")

    def cmd_set_max_speed(self, speed: float = None):
        self._dispatch("set_max_speed", speed or self.max_speed_var)

    def cmd_goto_abs(self, angle_deg: float):
        self._dispatch("goto_abs", angle_deg)

    def cmd_goto_rel(self, delta_deg: float):
        self._dispatch("goto_rel", delta_deg)

    def cmd_zero(self):
        self._dispatch("zero")

    def cmd_set_device_id(self, new_id: int):
        self._dispatch("set_device_id", new_id)

    def cmd_set_kp(self, kp: float):
        self._dispatch("set_kp", kp)

    def cmd_set_deadband(self, deadband_deg: float):
        self._dispatch("set_deadband", deadband_deg)

    def cmd_set_velocity_limit(self, vl: float):
        self._dispatch("set_velocity_limit", vl)

    def cmd_save(self):
        self._dispatch("save")

    def cmd_set_can_enabled(self, enabled: bool):
        self._dispatch("set_can_enabled", enabled)

    # ── dispatcher — the only place that knows about transport ────────────────

    def _dispatch(self, command: str, *args):
        if self.mode == "uart":
            self._uart_send(command, *args)
        elif self.mode == "can":
            self._can_send(command, *args)

    def _uart_send(self, command: str, *args):
        table = {
            "stop":              lambda: "S",
            "set_max_speed":     lambda s: f"M{s}",
            "goto_abs":          lambda a: f"A{a:.2f}",
            # ── UART has no equivalent for these; log and skip ──
            "goto_rel":          None,
            "zero":              None,
            "set_device_id":     None,
            "set_kp":            None,
            "set_deadband":      None,
            "set_velocity_limit":None,
            "save":              None,
            "set_can_enabled":   None,
        }
        builder = table.get(command)
        if builder is None:
            print(f"UART: '{command}' is not supported over UART — skipped.")
            return
        raw = builder(*args)
        if not self.serial_conn:
            print("UART: not connected.")
            return
        self.serial_conn.write((raw + "\r\n").encode("ascii"))

    def _can_send(self, command: str, *args):
        table = {
            "stop":              lambda:    [CMD_STOP],
            "goto_abs":          lambda a:  [CMD_GOTO_ABS,      *_pack_int16_le(int(round(a * 10)))],
            "goto_rel":          lambda a:  [CMD_GOTO_REL,      *_pack_int16_le(int(round(a * 10)))],
            "zero":              lambda:    [CMD_ZERO],
            "set_device_id":     lambda i:  [CMD_SET_ID,        int(i)],
            "set_kp":            lambda k:  [CMD_SET_KP,        *_pack_int16_le(int(round(k   * 100)))],
            "set_max_speed":     lambda s:  [CMD_SET_MAX_SPEED, *_pack_int16_le(int(round(s   * 100)))],
            "set_deadband":      lambda d:  [CMD_SET_DEADBAND,  *_pack_int16_le(int(round(d   * 10)))],
            "set_velocity_limit":lambda v:  [CMD_SET_VEL_LIMIT, *_pack_int16_le(int(round(v   * 100)))],
            "save":              lambda:    [CMD_SAVE],
            "set_can_enabled":   lambda e:  [CMD_SET_CAN_ENABLE, 0x01 if e else 0x00],
        }
        builder = table.get(command)
        if builder is None:
            print(f"CAN: unknown command '{command}' — dropped.")
            return
        data = builder(*args)
        if not self.can_bus:
            print("CAN: bus not initialised.")
            return
        msg = can.Message(
            arbitration_id=self.can_arb_id,
            data=bytearray(data),
            is_extended_id=False,
        )
        try:
            self.can_bus.send(msg)
        except can.CanError as e:
            print(f"CAN TX error: {e}")

    # ── control loop ───────────────────────────────────────────────────────────

    def _control_loop(self):
        self.cmd_set_max_speed()

        while self.running:
            loop_start = time.time()

            with self.lock:
                age          = time.time() - self.last_update_time
                cmd_snapshot = dict(self.remote_cmd)

            if self.last_update_time > 0 and age > self.WATCHDOG_TIMEOUT:
                self.cmd_stop()
                print(f"Watchdog: no update for {age:.1f}s — stopping.")
            elif cmd_snapshot:
                target_angle = (cmd_snapshot["angle"] / 100) * 120
                if target_angle == 0:
                    self.cmd_stop()
                else:
                    self.cmd_goto_abs(target_angle * 3)

            if self.mode == "uart" and self.serial_conn:
                if self.serial_conn.in_waiting > 500:
                    self.serial_conn.reset_input_buffer()

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, self.DT - elapsed))

    # ── misc ───────────────────────────────────────────────────────────────────

    def _calculate_angle(self, kappa_val: float) -> float:
        steering_rad = math.atan(self.WHEELBASE * kappa_val)
        steering_deg = math.degrees(steering_rad)
        return max(-45.0, min(45.0, steering_deg))

    def _close_connections(self):
        if self.serial_conn:
            self.serial_conn.close()
            self.serial_conn = None
        if self.can_bus:
            self.can_bus.shutdown()
            self.can_bus = None
