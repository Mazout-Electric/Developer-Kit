import threading
import queue
import time
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports
import can


TX_STATUS_ID = 0x101
RX_COMMAND_ID = 0x201


class MotorControlUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Motor Controller UI - Serial + CAN")
        self.root.geometry("1100x760")

        # connection state
        self.conn_mode = tk.StringVar(value="serial")
        self.serial_obj = None
        self.can_bus = None
        self.can_notifier_thread = None
        self.reader_thread = None
        self.stop_event = threading.Event()
        self.rx_queue = queue.Queue()

        # runtime status
        self.connected = False
        self.tracking_enabled = False

        # display vars
        self.conn_status_var = tk.StringVar(value="Disconnected")
        self.mode_status_var = tk.StringVar(value="Mode: -")
        self.pos_var = tk.StringVar(value="0.0")
        self.mech_var = tk.StringVar(value="0.0")
        self.target_var = tk.StringVar(value="0.0")
        self.vel_var = tk.StringVar(value="0.0")
        self.cmd_var = tk.StringVar(value="0.0")
        self.last_rx_var = tk.StringVar(value="-")

        # serial settings
        self.serial_port_var = tk.StringVar(value="COM6")
        self.serial_baud_var = tk.StringVar(value="115200")

        # can settings
        self.can_interface_var = tk.StringVar(value="slcan")
        self.can_channel_var = tk.StringVar(value="COM5")
        self.can_bitrate_var = tk.StringVar(value="500000")
        self.can_serial_baud_var = tk.StringVar(value="115200")

        # command vars
        self.abs_angle_var = tk.StringVar(value="180")
        self.inc_angle_var = tk.StringVar(value="30")
        self.kp_var = tk.StringVar(value="4.0")
        self.max_speed_var = tk.StringVar(value="5.0")
        self.deadband_var = tk.StringVar(value="2.0")
        self.vel_limit_var = tk.StringVar(value="6.0")

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(100, self._process_rx_queue)

    # =========================================================
    # UI
    # =========================================================
    def _build_ui(self):
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill="both", expand=True)

        self._build_connection_frame(main)
        self._build_status_frame(main)
        self._build_command_frame(main)
        self._build_quick_frame(main)
        self._build_log_frame(main)

    def _build_connection_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="Connection", padding=10)
        frame.pack(fill="x", pady=5)

        ttk.Label(frame, text="Mode").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        ttk.Radiobutton(frame, text="Serial", variable=self.conn_mode, value="serial", command=self._update_connection_mode_ui).grid(row=0, column=1, padx=5, pady=5, sticky="w")
        ttk.Radiobutton(frame, text="CAN", variable=self.conn_mode, value="can", command=self._update_connection_mode_ui).grid(row=0, column=2, padx=5, pady=5, sticky="w")

        ttk.Button(frame, text="Refresh Ports", command=self.refresh_ports).grid(row=0, column=3, padx=5, pady=5)

        # serial section
        self.serial_section = ttk.LabelFrame(frame, text="Serial Settings", padding=8)
        self.serial_section.grid(row=1, column=0, columnspan=5, sticky="ew", padx=5, pady=5)

        ttk.Label(self.serial_section, text="Port").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.serial_port_combo = ttk.Combobox(self.serial_section, textvariable=self.serial_port_var, width=18)
        self.serial_port_combo.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(self.serial_section, text="Baud").grid(row=0, column=2, padx=5, pady=5, sticky="w")
        ttk.Entry(self.serial_section, textvariable=self.serial_baud_var, width=12).grid(row=0, column=3, padx=5, pady=5)

        # can section
        self.can_section = ttk.LabelFrame(frame, text="CAN Settings", padding=8)
        self.can_section.grid(row=2, column=0, columnspan=5, sticky="ew", padx=5, pady=5)

        ttk.Label(self.can_section, text="Interface").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        ttk.Combobox(
            self.can_section,
            textvariable=self.can_interface_var,
            width=16,
            state="readonly",
            values=["slcan", "socketcan", "pcan", "kvaser", "vector", "gs_usb", "usb2can"]
        ).grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(self.can_section, text="Channel").grid(row=0, column=2, padx=5, pady=5, sticky="w")
        ttk.Entry(self.can_section, textvariable=self.can_channel_var, width=16).grid(row=0, column=3, padx=5, pady=5)

        ttk.Label(self.can_section, text="Bitrate").grid(row=0, column=4, padx=5, pady=5, sticky="w")
        ttk.Entry(self.can_section, textvariable=self.can_bitrate_var, width=12).grid(row=0, column=5, padx=5, pady=5)

        ttk.Label(self.can_section, text="Serial Baud (slcan)").grid(row=0, column=6, padx=5, pady=5, sticky="w")
        ttk.Entry(self.can_section, textvariable=self.can_serial_baud_var, width=12).grid(row=0, column=7, padx=5, pady=5)

        self.connect_btn = ttk.Button(frame, text="Connect", command=self.connect)
        self.connect_btn.grid(row=0, column=6, padx=8, pady=5)

        self.disconnect_btn = ttk.Button(frame, text="Disconnect", command=self.disconnect, state="disabled")
        self.disconnect_btn.grid(row=0, column=7, padx=8, pady=5)

        ttk.Label(frame, textvariable=self.conn_status_var).grid(row=0, column=8, padx=10, pady=5, sticky="w")

        self.refresh_ports()
        self._update_connection_mode_ui()

    def _build_status_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="Live Status", padding=10)
        frame.pack(fill="x", pady=5)

        fields = [
            ("Mode", self.mode_status_var),
            ("Output Position (deg)", self.pos_var),
            ("Mechanical Angle (deg)", self.mech_var),
            ("Target (deg)", self.target_var),
            ("Velocity (rad/s)", self.vel_var),
            ("Motor Cmd (rad/s)", self.cmd_var),
            ("Last RX", self.last_rx_var),
        ]

        for i, (label, var) in enumerate(fields):
            ttk.Label(frame, text=label).grid(row=0, column=i * 2, padx=4, pady=5, sticky="w")
            ttk.Entry(frame, textvariable=var, width=15, state="readonly").grid(row=0, column=i * 2 + 1, padx=4, pady=5)

    def _build_command_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="Commands", padding=10)
        frame.pack(fill="x", pady=5)

        ttk.Button(frame, text="Set Zero (Z)", command=self.cmd_zero, width=16).grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(frame, text="Stop (S)", command=self.cmd_stop, width=16).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(frame, text="Read (R/P)", command=self.cmd_read_status, width=16).grid(row=0, column=2, padx=5, pady=5)

        ttk.Label(frame, text="Absolute Angle").grid(row=1, column=0, padx=5, pady=5, sticky="e")
        ttk.Entry(frame, textvariable=self.abs_angle_var, width=12).grid(row=1, column=1, padx=5, pady=5, sticky="w")
        ttk.Button(frame, text="Go Absolute", command=self.cmd_absolute, width=16).grid(row=1, column=2, padx=5, pady=5)

        ttk.Label(frame, text="Increment Angle").grid(row=1, column=3, padx=5, pady=5, sticky="e")
        ttk.Entry(frame, textvariable=self.inc_angle_var, width=12).grid(row=1, column=4, padx=5, pady=5, sticky="w")
        ttk.Button(frame, text="Move Increment", command=self.cmd_increment, width=16).grid(row=1, column=5, padx=5, pady=5)

        ttk.Label(frame, text="Kp").grid(row=2, column=0, padx=5, pady=5, sticky="e")
        ttk.Entry(frame, textvariable=self.kp_var, width=12).grid(row=2, column=1, padx=5, pady=5, sticky="w")
        ttk.Button(frame, text="Set Kp", command=self.cmd_set_kp, width=16).grid(row=2, column=2, padx=5, pady=5)

        ttk.Label(frame, text="Max Speed").grid(row=2, column=3, padx=5, pady=5, sticky="e")
        ttk.Entry(frame, textvariable=self.max_speed_var, width=12).grid(row=2, column=4, padx=5, pady=5, sticky="w")
        ttk.Button(frame, text="Set Max Speed", command=self.cmd_set_max_speed, width=16).grid(row=2, column=5, padx=5, pady=5)

        ttk.Label(frame, text="Deadband (deg)").grid(row=3, column=0, padx=5, pady=5, sticky="e")
        ttk.Entry(frame, textvariable=self.deadband_var, width=12).grid(row=3, column=1, padx=5, pady=5, sticky="w")
        ttk.Button(frame, text="Set Deadband", command=self.cmd_set_deadband, width=16).grid(row=3, column=2, padx=5, pady=5)

        ttk.Label(frame, text="Velocity Limit").grid(row=3, column=3, padx=5, pady=5, sticky="e")
        ttk.Entry(frame, textvariable=self.vel_limit_var, width=12).grid(row=3, column=4, padx=5, pady=5, sticky="w")
        ttk.Button(frame, text="Set Vel Limit", command=self.cmd_set_vel_limit, width=16).grid(row=3, column=5, padx=5, pady=5)

        ttk.Button(frame, text="Speed + (U)", command=self.cmd_speed_up, width=16).grid(row=4, column=0, padx=5, pady=5)
        ttk.Button(frame, text="Speed - (J)", command=self.cmd_speed_down, width=16).grid(row=4, column=1, padx=5, pady=5)
        ttk.Button(frame, text="Tracking ON (T1)", command=lambda: self.send_text_command("T1"), width=16).grid(row=4, column=2, padx=5, pady=5)
        ttk.Button(frame, text="Tracking OFF (T0)", command=lambda: self.send_text_command("T0"), width=16).grid(row=4, column=3, padx=5, pady=5)

    def _build_quick_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="Quick Move", padding=10)
        frame.pack(fill="x", pady=5)

        abs_vals = [-180, -90, -45, 0, 45, 90, 180]
        for i, val in enumerate(abs_vals):
            ttk.Button(
                frame,
                text=f"A{val}",
                width=10,
                command=lambda v=val: self.send_absolute(v)
            ).grid(row=0, column=i, padx=4, pady=4)

        inc_vals = [-30, -10, 10, 30]
        for i, val in enumerate(inc_vals):
            ttk.Button(
                frame,
                text=f"I{val}",
                width=10,
                command=lambda v=val: self.send_increment(v)
            ).grid(row=1, column=i, padx=4, pady=4)

    def _build_log_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="Log", padding=10)
        frame.pack(fill="both", expand=True, pady=5)

        self.log_text = tk.Text(frame, wrap="word", height=18)
        self.log_text.pack(side="left", fill="both", expand=True)

        scroll = ttk.Scrollbar(frame, orient="vertical", command=self.log_text.yview)
        scroll.pack(side="right", fill="y")
        self.log_text.configure(yscrollcommand=scroll.set)

    def _update_connection_mode_ui(self):
        mode = self.conn_mode.get()
        if mode == "serial":
            self.serial_section.grid()
            self.can_section.grid_remove()
        else:
            self.can_section.grid()
            self.serial_section.grid_remove()

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if not ports:
            ports = ["COM1", "COM2", "COM3", "COM4", "COM5", "COM6"]
        self.serial_port_combo["values"] = ports
        if self.serial_port_var.get() not in ports and ports:
            self.serial_port_var.set(ports[0])

    # =========================================================
    # Logging
    # =========================================================
    def log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.log_text.insert("end", f"[{ts}] {msg}\n")
        self.log_text.see("end")

    # =========================================================
    # Connection
    # =========================================================
    def connect(self):
        if self.connected:
            return

        try:
            self.stop_event.clear()
            mode = self.conn_mode.get()

            if mode == "serial":
                port = self.serial_port_var.get().strip()
                baud = int(self.serial_baud_var.get().strip())
                self.serial_obj = serial.Serial(port=port, baudrate=baud, timeout=0.1)
                self.reader_thread = threading.Thread(target=self._serial_reader_worker, daemon=True)
                self.reader_thread.start()
                self.conn_status_var.set(f"Serial connected: {port} @ {baud}")
                self.log(f"Serial connected: {port} @ {baud}")

            else:
                interface = self.can_interface_var.get().strip()
                channel = self.can_channel_var.get().strip()
                bitrate = int(self.can_bitrate_var.get().strip())
                kwargs = {"interface": interface, "channel": channel, "bitrate": bitrate}
                if interface == "slcan":
                    kwargs["tty_baudrate"] = int(self.can_serial_baud_var.get().strip())
                self.can_bus = can.Bus(**kwargs)
                self.reader_thread = threading.Thread(target=self._can_reader_worker, daemon=True)
                self.reader_thread.start()
                self.conn_status_var.set(f"CAN connected: {interface} / {channel} / {bitrate}")
                self.log(f"CAN connected: interface={interface}, channel={channel}, bitrate={bitrate}")

            self.connected = True
            self.connect_btn.config(state="disabled")
            self.disconnect_btn.config(state="normal")

        except Exception as e:
            messagebox.showerror("Connection Error", str(e))
            self.log(f"Connection failed: {e}")
            self.disconnect()

    def disconnect(self):
        self.stop_event.set()

        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)

        if self.serial_obj:
            try:
                self.serial_obj.close()
            except Exception:
                pass
            self.serial_obj = None

        if self.can_bus:
            try:
                self.can_bus.shutdown()
            except Exception:
                pass
            self.can_bus = None

        self.connected = False
        self.reader_thread = None
        self.conn_status_var.set("Disconnected")
        self.connect_btn.config(state="normal")
        self.disconnect_btn.config(state="disabled")
        self.log("Disconnected")

    # =========================================================
    # Serial / CAN workers
    # =========================================================
    def _serial_reader_worker(self):
        while not self.stop_event.is_set():
            try:
                if self.serial_obj is None:
                    time.sleep(0.1)
                    continue

                line = self.serial_obj.readline()
                if line:
                    text = line.decode(errors="ignore").strip()
                    if text:
                        self.rx_queue.put(("serial", text))
            except Exception as e:
                self.rx_queue.put(("error", f"Serial read error: {e}"))
                time.sleep(0.2)

    def _can_reader_worker(self):
        while not self.stop_event.is_set():
            try:
                if self.can_bus is None:
                    time.sleep(0.1)
                    continue

                msg = self.can_bus.recv(timeout=0.1)
                if msg is not None:
                    self.rx_queue.put(("can", msg))
            except Exception as e:
                self.rx_queue.put(("error", f"CAN read error: {e}"))
                time.sleep(0.2)

    def _process_rx_queue(self):
        while not self.rx_queue.empty():
            kind, payload = self.rx_queue.get()

            if kind == "error":
                self.log(str(payload))

            elif kind == "serial":
                self._handle_serial_line(payload)

            elif kind == "can":
                self._handle_can_msg(payload)

        self.root.after(100, self._process_rx_queue)

    # =========================================================
    # RX handling
    # =========================================================
    def _handle_serial_line(self, line: str):
        self.last_rx_var.set(time.strftime("%H:%M:%S"))
        self.log(f"SER RX: {line}")

        if "Mode" in line:
            self.mode_status_var.set(line.split(":", 1)[-1].strip())

        if "Output Position(deg)" in line:
            try:
                val = float(line.split(":")[-1].strip())
                self.pos_var.set(f"{val:.3f}")
            except Exception:
                pass

        elif "Output Mechanical(deg)" in line:
            try:
                val = float(line.split(":")[-1].strip())
                self.mech_var.set(f"{val:.3f}")
            except Exception:
                pass

        elif "Target Output(deg)" in line or "Target output angle set (deg)" in line:
            try:
                val = float(line.split(":")[-1].strip())
                self.target_var.set(f"{val:.3f}")
            except Exception:
                pass

        elif "Output Velocity(rad/s)" in line:
            try:
                val = float(line.split(":")[-1].strip())
                self.vel_var.set(f"{val:.4f}")
            except Exception:
                pass

        elif "Motor Cmd(rad/s)" in line:
            try:
                val = float(line.split(":")[-1].strip())
                self.cmd_var.set(f"{val:.4f}")
            except Exception:
                pass

        elif "Reached target near output angle (deg)" in line:
            try:
                val = float(line.split(":")[-1].strip())
                self.pos_var.set(f"{val:.3f}")
            except Exception:
                pass

    def _handle_can_msg(self, msg: can.Message):
        self.last_rx_var.set(time.strftime("%H:%M:%S"))

        if msg.arbitration_id == TX_STATUS_ID and len(msg.data) >= 8:
            pos_deg = int.from_bytes(msg.data[0:2], "little", signed=True) / 10.0
            vel = int.from_bytes(msg.data[2:4], "little", signed=True) / 100.0
            tgt_deg = int.from_bytes(msg.data[4:6], "little", signed=True) / 10.0
            cmd = int.from_bytes(msg.data[6:8], "little", signed=True) / 100.0

            self.mode_status_var.set("CAN Status")
            self.pos_var.set(f"{pos_deg:.3f}")
            self.target_var.set(f"{tgt_deg:.3f}")
            self.vel_var.set(f"{vel:.4f}")
            self.cmd_var.set(f"{cmd:.4f}")
            self.log(f"CAN RX 0x{msg.arbitration_id:X}: pos={pos_deg:.2f}, vel={vel:.2f}, tgt={tgt_deg:.2f}, cmd={cmd:.2f}")
        else:
            self.log(f"CAN RX ID=0x{msg.arbitration_id:X} DATA={msg.data.hex(' ')}")

    # =========================================================
    # TX helpers
    # =========================================================
    def send_text_command(self, cmd: str):
        if not self.connected:
            messagebox.showwarning("Not Connected", "Connect Serial or CAN first.")
            return

        cmd = cmd.strip()
        if not cmd:
            return

        try:
            if self.conn_mode.get() == "serial":
                if not self.serial_obj:
                    raise RuntimeError("Serial not connected")
                self.serial_obj.write((cmd + "\n").encode())
                self.log(f"SER TX: {cmd}")
            else:
                self._send_can_equivalent(cmd)
        except Exception as e:
            messagebox.showerror("Send Error", str(e))
            self.log(f"Send failed: {e}")

    def _send_can_equivalent(self, cmd: str):
        if not self.can_bus:
            raise RuntimeError("CAN not connected")

        c = cmd.strip().upper()

        # Supported over CAN in this UI:
        # S, Z, Axxx, Ixxx
        if c == "S":
            data = bytes([0x01])

        elif c == "Z":
            data = bytes([0x04])

        elif c.startswith("A"):
            deg = float(c[1:])
            val = int(round(deg * 10.0))
            data = bytes([0x02]) + val.to_bytes(2, "little", signed=True)

        elif c.startswith("I"):
            deg = float(c[1:])
            val = int(round(deg * 10.0))
            data = bytes([0x03]) + val.to_bytes(2, "little", signed=True)

        else:
            raise RuntimeError("This CAN mapping currently supports only: S, Z, Axxx, Ixxx")

        msg = can.Message(arbitration_id=RX_COMMAND_ID, data=data, is_extended_id=False)
        self.can_bus.send(msg)
        self.log(f"CAN TX ID=0x{RX_COMMAND_ID:X} DATA={data.hex(' ')} CMD={c}")

    # =========================================================
    # Command wrappers
    # =========================================================
    def cmd_zero(self):
        self.send_text_command("Z")

    def cmd_stop(self):
        self.send_text_command("S")

    def cmd_read_status(self):
        # serial firmware supports R/P; CAN firmware sends periodic status
        if self.conn_mode.get() == "serial":
            self.send_text_command("P")
        else:
            self.log("CAN status is periodic; waiting for ID 0x101 frames")

    def send_absolute(self, value: float):
        self.send_text_command(f"A{value}")

    def send_increment(self, value: float):
        self.send_text_command(f"I{value}")

    def cmd_absolute(self):
        self.send_absolute(self._get_float(self.abs_angle_var, "Absolute Angle"))

    def cmd_increment(self):
        self.send_increment(self._get_float(self.inc_angle_var, "Increment Angle"))

    def cmd_set_kp(self):
        self.send_text_command(f"K{self._get_float(self.kp_var, 'Kp')}")

    def cmd_set_max_speed(self):
        self.send_text_command(f"M{self._get_float(self.max_speed_var, 'Max Speed')}")

    def cmd_set_deadband(self):
        self.send_text_command(f"D{self._get_float(self.deadband_var, 'Deadband')}")

    def cmd_set_vel_limit(self):
        self.send_text_command(f"V{self._get_float(self.vel_limit_var, 'Velocity Limit')}")

    def cmd_speed_up(self):
        self.send_text_command("U")

    def cmd_speed_down(self):
        self.send_text_command("J")

    def _get_float(self, var: tk.StringVar, name: str) -> float:
        try:
            return float(var.get().strip())
        except Exception:
            raise RuntimeError(f"Invalid {name}")

    # =========================================================
    # Close
    # =========================================================
    def on_close(self):
        self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    style = ttk.Style()
    try:
        style.theme_use("clam")
    except Exception:
        pass
    app = MotorControlUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()