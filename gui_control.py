import threading
import queue
import time
import math
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports
import can


CAN_STATUS_BASE_ID = 0x100
CAN_COMMAND_BASE_ID = 0x200
DEFAULT_GEAR_RATIO = 15.0


class DeviceState:
    def __init__(self, device_id: int):
        self.device_id = device_id
        self.module_name = f"Device_{device_id}"
        self.mode = "-"
        self.output_pos_deg = 0.0
        self.output_mech_deg = 0.0
        self.output_target_deg = 0.0
        self.output_vel_rad_s = 0.0
        self.output_rpm = 0.0
        self.motor_cmd_rad_s = 0.0
        self.motor_pos_deg = 0.0
        self.motor_mech_deg = 0.0
        self.last_rx = "-"
        self.last_source = "-"
        self.last_raw = ""


class MultiDeviceMotorUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Multi Device Motor Controller UI - Serial + CAN")
        self.root.geometry("1450x980")

        self.conn_mode = tk.StringVar(value="serial")
        self.serial_obj = None
        self.can_bus = None
        self.reader_thread = None
        self.stop_event = threading.Event()
        self.rx_queue = queue.Queue()
        self.connected = False

        self.serial_port_var = tk.StringVar(value="COM6")
        self.serial_baud_var = tk.StringVar(value="115200")

        self.can_interface_var = tk.StringVar(value="slcan")
        self.can_channel_var = tk.StringVar(value="COM5")
        self.can_bitrate_var = tk.StringVar(value="500000")
        self.can_serial_baud_var = tk.StringVar(value="115200")

        self.conn_status_var = tk.StringVar(value="Disconnected")

        self.device_id_var = tk.StringVar(value="1")
        self.module_name_var = tk.StringVar(value="")
        self.abs_angle_var = tk.StringVar(value="180")
        self.inc_angle_var = tk.StringVar(value="30")
        self.kp_var = tk.StringVar(value="4.0")
        self.max_speed_var = tk.StringVar(value="5.0")
        self.deadband_var = tk.StringVar(value="2.0")
        self.vel_limit_var = tk.StringVar(value="6.0")
        self.rpm_set_var = tk.StringVar(value="60")
        self.can_tx_enable_var = tk.StringVar(value="1")
        self.gear_ratio_var = tk.StringVar(value=str(DEFAULT_GEAR_RATIO))

        self.status_device_var = tk.StringVar(value="1")

        self.mode_status_var = tk.StringVar(value="-")
        self.output_pos_var = tk.StringVar(value="0.0")
        self.output_mech_var = tk.StringVar(value="0.0")
        self.output_target_var = tk.StringVar(value="0.0")
        self.output_vel_var = tk.StringVar(value="0.0")
        self.output_rpm_var = tk.StringVar(value="0.0")
        self.motor_cmd_var = tk.StringVar(value="0.0")
        self.motor_pos_var = tk.StringVar(value="0.0")
        self.motor_mech_var = tk.StringVar(value="0.0")
        self.last_rx_var = tk.StringVar(value="-")
        self.last_source_var = tk.StringVar(value="-")
        self.module_name_status_var = tk.StringVar(value="-")

        self.devices = {}

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(100, self._process_rx_queue)
        self.root.after(150, self._draw_visuals)

    # =========================================================
    # SCROLL HELPERS
    # =========================================================
    def _on_frame_configure(self, event=None):
        self.main_canvas.configure(scrollregion=self.main_canvas.bbox("all"))

    def _on_canvas_configure(self, event):
        self.main_canvas.itemconfig(self.canvas_window, width=event.width)

    def _on_mousewheel(self, event):
        self.main_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def _on_shiftwheel(self, event):
        self.main_canvas.xview_scroll(int(-1 * (event.delta / 120)), "units")

    # =========================================================
    # UI
    # =========================================================
    def _build_ui(self):
        outer = ttk.Frame(self.root)
        outer.pack(fill="both", expand=True)

        self.main_canvas = tk.Canvas(outer, highlightthickness=0)
        self.main_canvas.pack(side="left", fill="both", expand=True)

        v_scroll = ttk.Scrollbar(outer, orient="vertical", command=self.main_canvas.yview)
        v_scroll.pack(side="right", fill="y")

        h_scroll = ttk.Scrollbar(self.root, orient="horizontal", command=self.main_canvas.xview)
        h_scroll.pack(side="bottom", fill="x")

        self.main_canvas.configure(yscrollcommand=v_scroll.set, xscrollcommand=h_scroll.set)

        self.scrollable_frame = ttk.Frame(self.main_canvas, padding=10)
        self.canvas_window = self.main_canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")

        self.scrollable_frame.bind("<Configure>", self._on_frame_configure)
        self.main_canvas.bind("<Configure>", self._on_canvas_configure)

        self.main_canvas.bind_all("<MouseWheel>", self._on_mousewheel)
        self.main_canvas.bind_all("<Shift-MouseWheel>", self._on_shiftwheel)
        self.main_canvas.bind_all("<Button-4>", lambda e: self.main_canvas.yview_scroll(-1, "units"))
        self.main_canvas.bind_all("<Button-5>", lambda e: self.main_canvas.yview_scroll(1, "units"))

        main = self.scrollable_frame

        self._build_connection_frame(main)
        self._build_device_selector_frame(main)
        self._build_status_frame(main)
        self._build_command_frame(main)
        self._build_quick_frame(main)
        self._build_visual_frame(main)
        self._build_device_table_frame(main)
        self._build_log_frame(main)

    def _build_connection_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="Connection", padding=10)
        frame.pack(fill="x", pady=5)

        ttk.Label(frame, text="Mode").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        ttk.Radiobutton(frame, text="Serial", variable=self.conn_mode, value="serial", command=self._update_connection_mode_ui).grid(row=0, column=1, padx=5, pady=5, sticky="w")
        ttk.Radiobutton(frame, text="CAN", variable=self.conn_mode, value="can", command=self._update_connection_mode_ui).grid(row=0, column=2, padx=5, pady=5, sticky="w")

        ttk.Button(frame, text="Refresh Ports", command=self.refresh_ports).grid(row=0, column=3, padx=5, pady=5)

        self.serial_section = ttk.LabelFrame(frame, text="Serial Settings", padding=8)
        self.serial_section.grid(row=1, column=0, columnspan=6, sticky="ew", padx=5, pady=5)

        ttk.Label(self.serial_section, text="Port").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.serial_port_combo = ttk.Combobox(self.serial_section, textvariable=self.serial_port_var, width=18)
        self.serial_port_combo.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(self.serial_section, text="Baud").grid(row=0, column=2, padx=5, pady=5, sticky="w")
        ttk.Entry(self.serial_section, textvariable=self.serial_baud_var, width=12).grid(row=0, column=3, padx=5, pady=5)

        self.can_section = ttk.LabelFrame(frame, text="CAN Settings", padding=8)
        self.can_section.grid(row=2, column=0, columnspan=10, sticky="ew", padx=5, pady=5)

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

    def _build_device_selector_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="Target Device", padding=10)
        frame.pack(fill="x", pady=5)

        ttk.Label(frame, text="Command Device ID").grid(row=0, column=0, padx=5, pady=5, sticky="e")
        ttk.Entry(frame, textvariable=self.device_id_var, width=10).grid(row=0, column=1, padx=5, pady=5, sticky="w")

        ttk.Label(frame, text="Status Device ID").grid(row=0, column=2, padx=5, pady=5, sticky="e")
        self.status_device_combo = ttk.Combobox(frame, textvariable=self.status_device_var, width=10)
        self.status_device_combo.grid(row=0, column=3, padx=5, pady=5, sticky="w")

        ttk.Button(frame, text="Use Command ID as Status ID", command=self._sync_status_device).grid(row=0, column=4, padx=5, pady=5)

        ttk.Label(frame, text="Module Name").grid(row=0, column=5, padx=5, pady=5, sticky="e")
        ttk.Entry(frame, textvariable=self.module_name_var, width=20).grid(row=0, column=6, padx=5, pady=5, sticky="w")

        ttk.Button(frame, text="Query Status Now", command=self.cmd_read_status).grid(row=0, column=7, padx=5, pady=5)

    def _build_status_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="Selected Device Live Status", padding=10)
        frame.pack(fill="x", pady=5)

        fields = [
            ("Module Name", self.module_name_status_var),
            ("Mode", self.mode_status_var),
            ("Output Position (deg)", self.output_pos_var),
            ("Output Mechanical (deg)", self.output_mech_var),
            ("Target (deg)", self.output_target_var),
            ("Output Velocity (rad/s)", self.output_vel_var),
            ("Output RPM", self.output_rpm_var),
            ("Motor Cmd (rad/s)", self.motor_cmd_var),
            ("Motor Position (deg)", self.motor_pos_var),
            ("Motor Mechanical (deg)", self.motor_mech_var),
            ("Last RX", self.last_rx_var),
            ("Source", self.last_source_var),
        ]

        for i, (label, var) in enumerate(fields):
            row = i // 4
            col = (i % 4) * 2
            ttk.Label(frame, text=label).grid(row=row, column=col, padx=4, pady=5, sticky="w")
            ttk.Entry(frame, textvariable=var, width=18, state="readonly").grid(row=row, column=col + 1, padx=4, pady=5)

    def _build_command_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="Commands", padding=10)
        frame.pack(fill="x", pady=5)

        ttk.Button(frame, text="Set Zero (Z)", command=self.cmd_zero, width=16).grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(frame, text="Stop (S)", command=self.cmd_stop, width=16).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(frame, text="Read Status", command=self.cmd_read_status, width=16).grid(row=0, column=2, padx=5, pady=5)
        ttk.Button(frame, text="INFO", command=self.cmd_info, width=16).grid(row=0, column=3, padx=5, pady=5)
        ttk.Button(frame, text="SAVE", command=self.cmd_save, width=16).grid(row=0, column=4, padx=5, pady=5)
        ttk.Button(frame, text="LOAD", command=self.cmd_load, width=16).grid(row=0, column=5, padx=5, pady=5)
        ttk.Button(frame, text="DEFAULT", command=self.cmd_default, width=16).grid(row=0, column=6, padx=5, pady=5)

        ttk.Label(frame, text="Absolute Angle").grid(row=1, column=0, padx=5, pady=5, sticky="e")
        ttk.Entry(frame, textvariable=self.abs_angle_var, width=12).grid(row=1, column=1, padx=5, pady=5, sticky="w")
        ttk.Button(frame, text="Go Absolute", command=self.cmd_absolute, width=16).grid(row=1, column=2, padx=5, pady=5)

        ttk.Label(frame, text="Increment Angle").grid(row=1, column=3, padx=5, pady=5, sticky="e")
        ttk.Entry(frame, textvariable=self.inc_angle_var, width=12).grid(row=1, column=4, padx=5, pady=5, sticky="w")
        ttk.Button(frame, text="Move Increment", command=self.cmd_increment, width=16).grid(row=1, column=5, padx=5, pady=5)

        ttk.Label(frame, text="New Device ID").grid(row=2, column=0, padx=5, pady=5, sticky="e")
        ttk.Entry(frame, textvariable=self.device_id_var, width=12).grid(row=2, column=1, padx=5, pady=5, sticky="w")
        ttk.Button(frame, text="Set Device ID", command=self.cmd_set_device_id, width=16).grid(row=2, column=2, padx=5, pady=5)

        ttk.Label(frame, text="Module Name").grid(row=2, column=3, padx=5, pady=5, sticky="e")
        ttk.Entry(frame, textvariable=self.module_name_var, width=18).grid(row=2, column=4, padx=5, pady=5, sticky="w")
        ttk.Button(frame, text="Set Module Name", command=self.cmd_set_module_name, width=16).grid(row=2, column=5, padx=5, pady=5)

        ttk.Label(frame, text="Kp").grid(row=3, column=0, padx=5, pady=5, sticky="e")
        ttk.Entry(frame, textvariable=self.kp_var, width=12).grid(row=3, column=1, padx=5, pady=5, sticky="w")
        ttk.Button(frame, text="Set Kp", command=self.cmd_set_kp, width=16).grid(row=3, column=2, padx=5, pady=5)

        ttk.Label(frame, text="Max Speed").grid(row=3, column=3, padx=5, pady=5, sticky="e")
        ttk.Entry(frame, textvariable=self.max_speed_var, width=12).grid(row=3, column=4, padx=5, pady=5, sticky="w")
        ttk.Button(frame, text="Set Max Speed", command=self.cmd_set_max_speed, width=16).grid(row=3, column=5, padx=5, pady=5)

        ttk.Label(frame, text="Deadband (deg)").grid(row=4, column=0, padx=5, pady=5, sticky="e")
        ttk.Entry(frame, textvariable=self.deadband_var, width=12).grid(row=4, column=1, padx=5, pady=5, sticky="w")
        ttk.Button(frame, text="Set Deadband", command=self.cmd_set_deadband, width=16).grid(row=4, column=2, padx=5, pady=5)

        ttk.Label(frame, text="Velocity Limit (rad/s)").grid(row=4, column=3, padx=5, pady=5, sticky="e")
        ttk.Entry(frame, textvariable=self.vel_limit_var, width=12).grid(row=4, column=4, padx=5, pady=5, sticky="w")
        ttk.Button(frame, text="Set Velocity", command=self.cmd_set_vel_limit, width=16).grid(row=4, column=5, padx=5, pady=5)

        ttk.Label(frame, text="Input RPM").grid(row=5, column=0, padx=5, pady=5, sticky="e")
        ttk.Entry(frame, textvariable=self.rpm_set_var, width=12).grid(row=5, column=1, padx=5, pady=5, sticky="w")
        ttk.Button(frame, text="Set RPM", command=self.cmd_set_rpm, width=16).grid(row=5, column=2, padx=5, pady=5)

        ttk.Label(frame, text="CAN TX Enable").grid(row=5, column=3, padx=5, pady=5, sticky="e")
        ttk.Entry(frame, textvariable=self.can_tx_enable_var, width=12).grid(row=5, column=4, padx=5, pady=5, sticky="w")
        ttk.Button(frame, text="Set CAN TX", command=self.cmd_set_can_tx_enable, width=16).grid(row=5, column=5, padx=5, pady=5)

        ttk.Button(frame, text="Speed + (U)", command=self.cmd_speed_up, width=16).grid(row=6, column=0, padx=5, pady=5)
        ttk.Button(frame, text="Speed - (J)", command=self.cmd_speed_down, width=16).grid(row=6, column=1, padx=5, pady=5)
        ttk.Button(frame, text="Tracking ON (T1)", command=lambda: self.send_text_command("T1"), width=16).grid(row=6, column=2, padx=5, pady=5)
        ttk.Button(frame, text="Tracking OFF (T0)", command=lambda: self.send_text_command("T0"), width=16).grid(row=6, column=3, padx=5, pady=5)
        ttk.Button(frame, text="Help (H)", command=lambda: self.send_text_command("H"), width=16).grid(row=6, column=4, padx=5, pady=5)

    def _build_quick_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="Quick Move", padding=10)
        frame.pack(fill="x", pady=5)

        abs_vals = [-180, -90, -45, 0, 45, 90, 180]
        for i, val in enumerate(abs_vals):
            ttk.Button(frame, text=f"A{val}", width=10, command=lambda v=val: self.send_absolute(v)).grid(row=0, column=i, padx=4, pady=4)

        inc_vals = [-30, -10, 10, 30]
        for i, val in enumerate(inc_vals):
            ttk.Button(frame, text=f"I{val}", width=10, command=lambda v=val: self.send_increment(v)).grid(row=1, column=i, padx=4, pady=4)

    def _build_visual_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="2D Visualisation", padding=10)
        frame.pack(fill="x", pady=5)

        self.output_canvas = tk.Canvas(frame, width=360, height=280, bg="white", highlightthickness=1, highlightbackground="#888")
        self.output_canvas.pack(side="left", padx=10, pady=5)

        self.motor_canvas = tk.Canvas(frame, width=360, height=280, bg="white", highlightthickness=1, highlightbackground="#888")
        self.motor_canvas.pack(side="left", padx=10, pady=5)

        info = ttk.Frame(frame)
        info.pack(side="left", fill="both", expand=True, padx=10)

        ttk.Label(info, text="Left dial: Output shaft position", font=("Arial", 11, "bold")).pack(anchor="w", pady=4)
        ttk.Label(info, text="Blue arrow = current output mechanical angle").pack(anchor="w", pady=2)
        ttk.Label(info, text="Red mark = output target angle").pack(anchor="w", pady=2)
        ttk.Label(info, text="Right dial: Motor shaft position").pack(anchor="w", pady=8)
        ttk.Label(info, text="Blue arrow = motor mechanical angle").pack(anchor="w", pady=2)
        ttk.Label(info, text=f"Gear ratio = {DEFAULT_GEAR_RATIO}:1").pack(anchor="w", pady=8)

    def _build_device_table_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="All Devices Seen", padding=10)
        frame.pack(fill="both", expand=False, pady=5)

        columns = ("device_id", "name", "mode", "out_pos", "out_mech", "target", "vel", "cmd", "last_rx", "source")
        self.device_tree = ttk.Treeview(frame, columns=columns, show="headings", height=8)

        headings = {
            "device_id": "Device ID",
            "name": "Module Name",
            "mode": "Mode",
            "out_pos": "Output Pos",
            "out_mech": "Output Mech",
            "target": "Target",
            "vel": "Vel(rad/s)",
            "cmd": "Cmd(rad/s)",
            "last_rx": "Last RX",
            "source": "Source",
        }

        widths = {
            "device_id": 70,
            "name": 150,
            "mode": 90,
            "out_pos": 100,
            "out_mech": 100,
            "target": 100,
            "vel": 100,
            "cmd": 100,
            "last_rx": 100,
            "source": 80,
        }

        for col in columns:
            self.device_tree.heading(col, text=headings[col])
            self.device_tree.column(col, width=widths[col], anchor="center")

        self.device_tree.pack(side="left", fill="both", expand=True)
        scroll = ttk.Scrollbar(frame, orient="vertical", command=self.device_tree.yview)
        scroll.pack(side="right", fill="y")
        self.device_tree.configure(yscrollcommand=scroll.set)

        self.device_tree.bind("<<TreeviewSelect>>", self._on_device_tree_select)

    def _build_log_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="Log", padding=10)
        frame.pack(fill="both", expand=True, pady=5)

        self.log_text = tk.Text(frame, wrap="word", height=14)
        self.log_text.pack(side="left", fill="both", expand=True)

        scroll = ttk.Scrollbar(frame, orient="vertical", command=self.log_text.yview)
        scroll.pack(side="right", fill="y")
        self.log_text.configure(yscrollcommand=scroll.set)

    def _draw_dial(self, canvas, title, current_deg, target_deg=None, show_target=True):
        canvas.delete("all")
        w = int(canvas["width"])
        cx = w // 2
        cy = 125
        r = 90

        canvas.create_text(cx, 20, text=title, font=("Arial", 12, "bold"))
        canvas.create_oval(cx - r, cy - r, cx + r, cy + r, width=2, outline="black")

        for deg in range(0, 360, 30):
            ang = math.radians(deg - 90)
            x1 = cx + (r - 8) * math.cos(ang)
            y1 = cy + (r - 8) * math.sin(ang)
            x2 = cx + r * math.cos(ang)
            y2 = cy + r * math.sin(ang)
            canvas.create_line(x1, y1, x2, y2, fill="#666", width=2)

        if show_target and target_deg is not None:
            ta = math.radians(target_deg - 90)
            tx = cx + (r + 12) * math.cos(ta)
            ty = cy + (r + 12) * math.sin(ta)
            canvas.create_oval(tx - 5, ty - 5, tx + 5, ty + 5, fill="red", outline="red")
            canvas.create_text(tx, ty - 14, text="T", fill="red", font=("Arial", 10, "bold"))

        ca = math.radians(current_deg - 90)
        ax = cx + (r - 18) * math.cos(ca)
        ay = cy + (r - 18) * math.sin(ca)
        canvas.create_line(cx, cy, ax, ay, fill="blue", width=4, arrow="last")

        canvas.create_oval(cx - 6, cy - 6, cx + 6, cy + 6, fill="black")
        canvas.create_text(cx, 235, text=f"Current: {current_deg:.2f}°", font=("Arial", 11, "bold"), fill="blue")
        if show_target and target_deg is not None:
            canvas.create_text(cx, 255, text=f"Target: {target_deg:.2f}°", font=("Arial", 10), fill="red")

    def _draw_visuals(self):
        dev = self._get_selected_state()
        if dev is None:
            out_cur = 0.0
            out_tgt = 0.0
            mot_cur = 0.0
        else:
            out_cur = dev.output_mech_deg % 360.0
            out_tgt = dev.output_target_deg % 360.0
            mot_cur = dev.motor_mech_deg % 360.0

        self._draw_dial(self.output_canvas, "Output Shaft", out_cur, out_tgt, True)
        self._draw_dial(self.motor_canvas, "Motor Shaft", mot_cur, None, False)
        self.root.after(150, self._draw_visuals)

    # =========================================================
    # Connection / ports
    # =========================================================
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
    # Device helpers
    # =========================================================
    def _ensure_device(self, device_id: int) -> DeviceState:
        if device_id not in self.devices:
            self.devices[device_id] = DeviceState(device_id)
            self._refresh_device_tree()
            self._refresh_status_device_options()
        return self.devices[device_id]

    def _refresh_device_tree(self):
        items = {self.device_tree.item(item, "values")[0]: item for item in self.device_tree.get_children()}
        for dev_id, state in sorted(self.devices.items()):
            values = (
                str(state.device_id),
                state.module_name,
                state.mode,
                f"{state.output_pos_deg:.2f}",
                f"{state.output_mech_deg:.2f}",
                f"{state.output_target_deg:.2f}",
                f"{state.output_vel_rad_s:.3f}",
                f"{state.motor_cmd_rad_s:.3f}",
                state.last_rx,
                state.last_source,
            )
            key = str(dev_id)
            if key in items:
                self.device_tree.item(items[key], values=values)
            else:
                self.device_tree.insert("", "end", values=values)

    def _refresh_status_device_options(self):
        ids = [str(k) for k in sorted(self.devices.keys())]
        if not ids:
            ids = [self.status_device_var.get() or "1"]
        self.status_device_combo["values"] = ids
        if self.status_device_var.get() not in ids:
            self.status_device_var.set(ids[0])

    def _sync_status_device(self):
        self.status_device_var.set(self.device_id_var.get().strip())
        self._update_selected_status_panel()

    def _on_device_tree_select(self, _event=None):
        sel = self.device_tree.selection()
        if not sel:
            return
        values = self.device_tree.item(sel[0], "values")
        if values:
            self.status_device_var.set(str(values[0]))
            self._update_selected_status_panel()

    def _get_selected_device_id(self):
        try:
            return int(self.status_device_var.get().strip())
        except Exception:
            return None

    def _get_selected_state(self):
        dev_id = self._get_selected_device_id()
        if dev_id is None:
            return None
        return self.devices.get(dev_id)

    def _update_selected_status_panel(self):
        state = self._get_selected_state()
        if state is None:
            return

        self.module_name_status_var.set(state.module_name)
        self.mode_status_var.set(state.mode)
        self.output_pos_var.set(f"{state.output_pos_deg:.3f}")
        self.output_mech_var.set(f"{state.output_mech_deg:.3f}")
        self.output_target_var.set(f"{state.output_target_deg:.3f}")
        self.output_vel_var.set(f"{state.output_vel_rad_s:.4f}")
        self.output_rpm_var.set(f"{state.output_rpm:.3f}")
        self.motor_cmd_var.set(f"{state.motor_cmd_rad_s:.4f}")
        self.motor_pos_var.set(f"{state.motor_pos_deg:.3f}")
        self.motor_mech_var.set(f"{state.motor_mech_deg:.3f}")
        self.last_rx_var.set(state.last_rx)
        self.last_source_var.set(state.last_source)
        self.module_name_var.set(state.module_name)

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
    # Readers
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
    # Serial parsing
    # =========================================================
    def _get_or_create_serial_device(self):
        try:
            dev_id = int(self.device_id_var.get().strip())
        except Exception:
            dev_id = 1
        return self._ensure_device(dev_id)

    def _handle_serial_line(self, line: str):
        self.log(f"SER RX: {line}")
        state = self._get_or_create_serial_device()
        state.last_rx = time.strftime("%H:%M:%S")
        state.last_source = "Serial"
        state.last_raw = line

        if line.startswith("Module Name"):
            try:
                state.module_name = line.split(":", 1)[1].strip()
            except Exception:
                pass

        elif line.startswith("Device ID"):
            try:
                new_id = int(line.split(":", 1)[1].strip())
                if new_id != state.device_id:
                    self.devices.pop(state.device_id, None)
                    state.device_id = new_id
                    self.devices[new_id] = state
                    self.device_id_var.set(str(new_id))
                    self.status_device_var.set(str(new_id))
            except Exception:
                pass

        elif line.startswith("Mode"):
            try:
                state.mode = line.split(":", 1)[1].strip()
            except Exception:
                pass

        elif "Output Position(deg)" in line:
            try:
                state.output_pos_deg = float(line.split(":")[-1].strip())
            except Exception:
                pass

        elif "Output Mechanical(deg)" in line:
            try:
                state.output_mech_deg = float(line.split(":")[-1].strip())
            except Exception:
                pass

        elif "Target Output(deg)" in line or "Target output angle set (deg)" in line:
            try:
                state.output_target_deg = float(line.split(":")[-1].strip())
            except Exception:
                pass

        elif "Output Velocity(rad/s)" in line:
            try:
                state.output_vel_rad_s = float(line.split(":")[-1].strip())
                state.output_rpm = state.output_vel_rad_s * 60.0 / (2.0 * math.pi)
            except Exception:
                pass

        elif "Motor Cmd(rad/s)" in line:
            try:
                state.motor_cmd_rad_s = float(line.split(":")[-1].strip())
            except Exception:
                pass

        elif "Motor Angle(deg cumulative)" in line:
            try:
                state.motor_pos_deg = float(line.split(":")[-1].strip())
                state.motor_mech_deg = state.motor_pos_deg % 360.0
            except Exception:
                pass

        elif "Motor Mechanical(deg)" in line:
            try:
                state.motor_mech_deg = float(line.split(":")[-1].strip())
            except Exception:
                pass

        elif "Reached target near output angle (deg)" in line:
            try:
                state.output_pos_deg = float(line.split(":")[-1].strip())
            except Exception:
                pass

        elif line.startswith("OK: module name ="):
            try:
                state.module_name = line.split("=", 1)[1].strip()
            except Exception:
                pass

        elif line.startswith("OK: device ID ="):
            try:
                new_id = int(line.split("=", 1)[1].strip())
                if new_id != state.device_id:
                    self.devices.pop(state.device_id, None)
                    state.device_id = new_id
                    self.devices[new_id] = state
                    self.device_id_var.set(str(new_id))
                    self.status_device_var.set(str(new_id))
            except Exception:
                pass

        self._refresh_device_tree()
        self._refresh_status_device_options()
        self._update_selected_status_panel()

    # =========================================================
    # CAN parsing
    # =========================================================
    def _handle_can_msg(self, msg: can.Message):
        arb = msg.arbitration_id

        if CAN_STATUS_BASE_ID <= arb <= CAN_STATUS_BASE_ID + 127 and len(msg.data) >= 8:
            device_id = arb - CAN_STATUS_BASE_ID
            state = self._ensure_device(device_id)
            state.last_rx = time.strftime("%H:%M:%S")
            state.last_source = "CAN"
            state.last_raw = msg.data.hex(" ")

            state.output_pos_deg = int.from_bytes(msg.data[0:2], "little", signed=True) / 10.0
            state.output_vel_rad_s = int.from_bytes(msg.data[2:4], "little", signed=True) / 100.0
            state.output_target_deg = int.from_bytes(msg.data[4:6], "little", signed=True) / 10.0
            state.motor_cmd_rad_s = int.from_bytes(msg.data[6:8], "little", signed=True) / 100.0

            state.output_mech_deg = state.output_pos_deg % 360.0
            state.output_rpm = state.output_vel_rad_s * 60.0 / (2.0 * math.pi)
            state.motor_pos_deg = state.output_pos_deg * DEFAULT_GEAR_RATIO
            state.motor_mech_deg = state.motor_pos_deg % 360.0
            state.mode = "CAN Status"

            self.log(
                f"CAN RX Dev{device_id} ID=0x{arb:X}: "
                f"pos={state.output_pos_deg:.2f}, "
                f"vel={state.output_vel_rad_s:.2f}, "
                f"tgt={state.output_target_deg:.2f}, "
                f"cmd={state.motor_cmd_rad_s:.2f}"
            )

            self._refresh_device_tree()
            self._refresh_status_device_options()
            self._update_selected_status_panel()
        else:
            self.log(f"CAN RX ID=0x{arb:X} DATA={msg.data.hex(' ')}")

    # =========================================================
    # Send helpers
    # =========================================================
    def _get_command_device_id(self):
        try:
            dev_id = int(self.device_id_var.get().strip())
        except Exception:
            raise RuntimeError("Invalid Device ID")
        if dev_id < 1 or dev_id > 127:
            raise RuntimeError("Device ID must be 1..127")
        return dev_id

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

    def _send_can_msg(self, device_id: int, payload: bytes):
        if not self.can_bus:
            raise RuntimeError("CAN not connected")
        arb = CAN_COMMAND_BASE_ID + device_id
        msg = can.Message(arbitration_id=arb, data=payload, is_extended_id=False)
        self.can_bus.send(msg)
        self.log(f"CAN TX Dev{device_id} ID=0x{arb:X} DATA={payload.hex(' ')}")

    def _send_can_equivalent(self, cmd: str):
        device_id = self._get_command_device_id()
        c = cmd.strip()
        upper = c.upper()

        if upper == "S":
            self._send_can_msg(device_id, bytes([0x01]))
            return

        if upper == "Z":
            self._send_can_msg(device_id, bytes([0x04]))
            return

        if upper.startswith("A"):
            deg = float(c[1:])
            val = int(round(deg * 10.0))
            self._send_can_msg(device_id, bytes([0x02]) + val.to_bytes(2, "little", signed=True))
            return

        if upper.startswith("I"):
            deg = float(c[1:])
            val = int(round(deg * 10.0))
            self._send_can_msg(device_id, bytes([0x03]) + val.to_bytes(2, "little", signed=True))
            return

        if upper.startswith("ID"):
            new_id = int(c[2:])
            if new_id < 1 or new_id > 127:
                raise RuntimeError("New Device ID must be 1..127")
            self._send_can_msg(device_id, bytes([0x05, new_id & 0xFF]))
            return

        if upper.startswith("K"):
            v = float(c[1:])
            val = int(round(v * 100.0))
            self._send_can_msg(device_id, bytes([0x06]) + val.to_bytes(2, "little", signed=True))
            return

        if upper.startswith("M"):
            v = float(c[1:])
            val = int(round(v * 100.0))
            self._send_can_msg(device_id, bytes([0x07]) + val.to_bytes(2, "little", signed=True))
            return

        if upper.startswith("D"):
            v = float(c[1:])
            val = int(round(v * 10.0))
            self._send_can_msg(device_id, bytes([0x08]) + val.to_bytes(2, "little", signed=True))
            return

        if upper.startswith("V"):
            v = float(c[1:])
            val = int(round(v * 100.0))
            self._send_can_msg(device_id, bytes([0x09]) + val.to_bytes(2, "little", signed=True))
            return

        if upper == "SAVE":
            self._send_can_msg(device_id, bytes([0x0A]))
            return

        if upper.startswith("N"):
            v = int(c[1:])
            self._send_can_msg(device_id, bytes([0x0B, 1 if v != 0 else 0]))
            return

        raise RuntimeError("This CAN mapping supports: S, Z, Axxx, Ixxx, IDx, Kx, Mx, Dx, Vx, SAVE, N0/N1")

    # =========================================================
    # Command wrappers
    # =========================================================
    def cmd_zero(self):
        self.send_text_command("Z")

    def cmd_stop(self):
        self.send_text_command("S")

    def cmd_read_status(self):
        if self.conn_mode.get() == "serial":
            self.send_text_command("R")
            self.send_text_command("P")
            self.send_text_command("INFO")
        else:
            self.log("CAN status is periodic; waiting for device status frame")

    def cmd_info(self):
        self.send_text_command("INFO")

    def cmd_save(self):
        self.send_text_command("SAVE")

    def cmd_load(self):
        self.send_text_command("LOAD")

    def cmd_default(self):
        self.send_text_command("DEF")

    def send_absolute(self, value: float):
        self.send_text_command(f"A{value}")

    def send_increment(self, value: float):
        self.send_text_command(f"I{value}")

    def cmd_absolute(self):
        self.send_absolute(self._get_float(self.abs_angle_var, "Absolute Angle"))

    def cmd_increment(self):
        self.send_increment(self._get_float(self.inc_angle_var, "Increment Angle"))

    def cmd_set_device_id(self):
        current_target = self._get_command_device_id()
        self.send_text_command(f"ID{current_target}")

    def cmd_set_module_name(self):
        name = self.module_name_var.get().strip()
        if not name:
            messagebox.showwarning("Input Error", "Module name cannot be empty.")
            return
        self.send_text_command(f"NAME{name}")

    def cmd_set_kp(self):
        self.send_text_command(f"K{self._get_float(self.kp_var, 'Kp')}")

    def cmd_set_max_speed(self):
        self.send_text_command(f"M{self._get_float(self.max_speed_var, 'Max Speed')}")

    def cmd_set_deadband(self):
        self.send_text_command(f"D{self._get_float(self.deadband_var, 'Deadband')}")

    def cmd_set_vel_limit(self):
        value = self._get_float(self.vel_limit_var, "Velocity Limit")
        self.send_text_command(f"V{value}")

    def cmd_set_rpm(self):
        rpm = self._get_float(self.rpm_set_var, "RPM")
        rad_s = rpm * (2.0 * math.pi) / 60.0
        self.vel_limit_var.set(f"{rad_s:.4f}")
        self.send_text_command(f"V{rad_s:.4f}")
        self.log(f"RPM {rpm:.3f} converted to {rad_s:.4f} rad/s")

    def cmd_set_can_tx_enable(self):
        try:
            v = int(self.can_tx_enable_var.get().strip())
        except Exception:
            raise RuntimeError("Invalid CAN TX value; use 0 or 1")
        self.send_text_command(f"N{1 if v != 0 else 0}")

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
    MultiDeviceMotorUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()