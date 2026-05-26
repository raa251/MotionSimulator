"""Assetto Corsa Competizione 2DOF motion simulator with GUI and Arduino serial support."""

from __future__ import annotations

import argparse
import json
import math
import logging
import queue
import socket
import struct
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import messagebox, simpledialog, ttk
from typing import Optional

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import serial
import serial.tools.list_ports

PROFILE_PATH = Path(__file__).resolve().parent / "axis_profiles.json"
TELEMETRY_SOURCE_OPTIONS = [
    "None",
    "Pitch",
    "Roll",
    "Speed",
    "G Longitudinal",
    "G Lateral",
    "G Vertical",
]


@dataclass(frozen=True)
class TelemetryFrame:
    timestamp: float
    pitch_deg: float
    roll_deg: float
    g_force_longitudinal: float
    g_force_lateral: float
    g_force_vertical: float
    speed_kmh: float


@dataclass
class PlatformCommand:
    pitch_deg: float
    roll_deg: float
    timestamp: float


@dataclass(frozen=True)
class AxisProfile:
    name: str
    pitch_scale: float
    roll_scale: float
    pitch_accel_gain: float
    roll_accel_gain: float
    pitch_limit: float
    roll_limit: float
    pitch_type: str
    roll_type: str
    smoothing: float
    pitch_mapping_sources: list[str]
    pitch_mapping_percents: list[float]
    roll_mapping_sources: list[str]
    roll_mapping_percents: list[float]

    def to_dict(self) -> dict[str, object]:
        return {
            "pitch_scale": self.pitch_scale,
            "roll_scale": self.roll_scale,
            "pitch_accel_gain": self.pitch_accel_gain,
            "roll_accel_gain": self.roll_accel_gain,
            "pitch_limit": self.pitch_limit,
            "roll_limit": self.roll_limit,
            "pitch_type": self.pitch_type,
            "roll_type": self.roll_type,
            "pitch_mapping_sources": self.pitch_mapping_sources,
            "pitch_mapping_percents": self.pitch_mapping_percents,
            "roll_mapping_sources": self.roll_mapping_sources,
            "roll_mapping_percents": self.roll_mapping_percents,
            "smoothing": self.smoothing,
        }

    @classmethod
    def from_dict(cls, name: str, data: dict[str, str | float]) -> "AxisProfile":
        pitch_limit = data.get("pitch_limit")
        roll_limit = data.get("roll_limit")
        if pitch_limit is None or roll_limit is None:
            pitch_limit = float(data.get("max_angle", 15.0))
            roll_limit = float(data.get("max_angle", 15.0))

        # Helper to coerce mapping lists safely
        def _coerce_str_list(value, default):
            if isinstance(value, list):
                return [str(v) for v in value][:10]
            return default

        def _coerce_float_list(value, default):
            if isinstance(value, list):
                out = []
                for v in value[:10]:
                    try:
                        out.append(float(v))
                    except Exception:
                        out.append(0.0)
                return out
            return default

        pitch_mapping_sources = _coerce_str_list(data.get("pitch_mapping_sources"), ["None"] * 10)
        pitch_mapping_percents = _coerce_float_list(data.get("pitch_mapping_percents"), [0.0] * 10)
        roll_mapping_sources = _coerce_str_list(data.get("roll_mapping_sources"), ["None"] * 10)
        roll_mapping_percents = _coerce_float_list(data.get("roll_mapping_percents"), [0.0] * 10)

        return cls(
            name=name,
            pitch_scale=float(data.get("pitch_scale", 1.0)),
            roll_scale=float(data.get("roll_scale", 1.0)),
            pitch_accel_gain=float(data.get("pitch_accel_gain", 4.0)),
            roll_accel_gain=float(data.get("roll_accel_gain", 4.0)),
            pitch_limit=float(pitch_limit),
            roll_limit=float(roll_limit),
            pitch_type=str(data.get("pitch_type", "rotational")),
            roll_type=str(data.get("roll_type", "rotational")),
            smoothing=float(data.get("smoothing", 0.92)),
            pitch_mapping_sources=pitch_mapping_sources,
            pitch_mapping_percents=pitch_mapping_percents,
            roll_mapping_sources=roll_mapping_sources,
            roll_mapping_percents=roll_mapping_percents,
        )


class ProfileManager:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.profiles: dict[str, AxisProfile] = self._load_profiles()

    def _load_profiles(self) -> dict[str, AxisProfile]:
        if not self.path.exists():
            return {}
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            return {
                name: AxisProfile.from_dict(name, profile)
                for name, profile in data.items()
                if isinstance(profile, dict)
            }
        except Exception as exc:
            logging.debug("Failed to load profiles: %s", exc)
            return {}

    def save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        data = {name: profile.to_dict() for name, profile in self.profiles.items()}
        self.path.write_text(json.dumps(data, indent=2), encoding="utf-8")

    def save_profile(self, profile: AxisProfile) -> None:
        self.profiles[profile.name] = profile
        self.save()

    def delete_profile(self, name: str) -> None:
        if name in self.profiles:
            del self.profiles[name]
            self.save()

    def profile_names(self) -> list[str]:
        return sorted(self.profiles.keys())


class SerialActuatorOutput:
    def __init__(self, serial_controller: "SerialController") -> None:
        self.serial_controller = serial_controller

    def send_raw(self, payload: str, terminator: str = "") -> None:
        self.serial_controller.send(payload, terminator=terminator)

    def send_positions(self, pitch_target: int, roll_target: int) -> None:
        payload = f"[A{pitch_target}][B{roll_target}]"
        self.serial_controller.send(payload, terminator="")


class AccTelemetryReceiver(threading.Thread):
    """Receive Assetto Corsa Competizione UDP telemetry packets."""

    HEADER_STRUCT = struct.Struct("<HHIQfIBB")
    CAR_MOTION_STRUCT = struct.Struct("<44f")
    CAR_MOTION_SIZE = CAR_MOTION_STRUCT.size
    HEADER_SIZE = HEADER_STRUCT.size

    def __init__(self, host: str = "0.0.0.0", port: int = 9996, queue_size: int = 64) -> None:
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.host, self.port))
        self.frames: queue.Queue[TelemetryFrame] = queue.Queue(maxsize=queue_size)
        self.running = threading.Event()
        self.running.set()
        logging.info("ACC telemetry receiver listening on %s:%d", self.host, self.port)

    def run(self) -> None:
        while self.running.is_set():
            try:
                packet, _ = self.socket.recvfrom(65535)
                frame = self._parse_packet(packet)
                if frame is not None:
                    try:
                        self.frames.put_nowait(frame)
                    except queue.Full:
                        _ = self.frames.get_nowait()
                        self.frames.put_nowait(frame)
            except OSError:
                break
            except Exception as exc:  # pragma: no cover
                logging.debug("ACC packet parse failed: %s", exc)

    def stop(self) -> None:
        self.running.clear()
        try:
            self.socket.close()
        except OSError:
            pass

    def get_latest(self) -> Optional[TelemetryFrame]:
        frame: Optional[TelemetryFrame] = None
        while not self.frames.empty():
            frame = self.frames.get()
        return frame

    def _parse_packet(self, data: bytes) -> Optional[TelemetryFrame]:
        if len(data) < self.HEADER_SIZE + self.CAR_MOTION_SIZE:
            return None

        packet_id, packet_version, packet_size, session_uid, session_time, frame_id, player_index, _ = self.HEADER_STRUCT.unpack_from(data, 0)
        if packet_id != 0:
            return None

        if packet_size > len(data):
            logging.debug("Incomplete telemetry packet: expected %d bytes got %d", packet_size, len(data))
            return None

        if player_index >= 64:
            logging.debug("Unrealistic player index %d", player_index)
            player_index = 0

        offset = self.HEADER_SIZE + player_index * self.CAR_MOTION_SIZE
        if offset + self.CAR_MOTION_SIZE > len(data):
            return None

        motion = self.CAR_MOTION_STRUCT.unpack_from(data, offset)
        g_force_lateral = motion[12]
        g_force_longitudinal = motion[13]
        g_force_vertical = motion[14]
        pitch_rad = motion[15]
        yaw_rad = motion[16]
        roll_rad = motion[17]
        speed_kmh = self._extract_speed_from_motion(motion)

        return TelemetryFrame(
            timestamp=time.time(),
            pitch_deg=math.degrees(pitch_rad),
            roll_deg=math.degrees(roll_rad),
            g_force_longitudinal=g_force_longitudinal,
            g_force_lateral=g_force_lateral,
            g_force_vertical=g_force_vertical,
            speed_kmh=speed_kmh,
        )

    @staticmethod
    def _extract_speed_from_motion(motion: tuple[float, ...]) -> float:
        world_vel_x, world_vel_y, world_vel_z = motion[3], motion[4], motion[5]
        speed = math.sqrt(world_vel_x**2 + world_vel_y**2 + world_vel_z**2)
        return max(speed * 3.6, 0.0)


class TwoDoFMotionCue:
    """Convert ACC telemetry into 2DOF platform pitch and roll commands."""

    def __init__(
        self,
        pitch_scale: float = 1.0,
        roll_scale: float = 1.0,
        pitch_accel_gain: float = 4.0,
        roll_accel_gain: float = 4.0,
        max_angle: float = 15.0,
        smoothing: float = 0.92,
    ) -> None:
        self.pitch_scale = pitch_scale
        self.roll_scale = roll_scale
        self.pitch_accel_gain = pitch_accel_gain
        self.roll_accel_gain = roll_accel_gain
        self.max_angle = max_angle
        self.smoothing = smoothing
        self.last_pitch = 0.0
        self.last_roll = 0.0

    def compute(self, frame: TelemetryFrame) -> PlatformCommand:
        target_pitch = frame.pitch_deg * self.pitch_scale + frame.g_force_longitudinal * self.pitch_accel_gain
        target_roll = frame.roll_deg * self.roll_scale + frame.g_force_lateral * self.roll_accel_gain

        pitch = self._lowpass(target_pitch, self.last_pitch)
        roll = self._lowpass(target_roll, self.last_roll)
        self.last_pitch = pitch
        self.last_roll = roll

        return PlatformCommand(
            pitch_deg=pitch,
            roll_deg=roll,
            timestamp=frame.timestamp,
        )

    def _lowpass(self, value: float, previous: float) -> float:
        return previous * self.smoothing + value * (1.0 - self.smoothing)

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(min(value, maximum), minimum)


class SerialController(threading.Thread):
    """Manage Arduino serial communication in the background."""

    def __init__(self) -> None:
        super().__init__(daemon=True)
        self.serial_port: Optional[serial.Serial] = None
        self.port_name = ""
        self.baud_rate = 115200
        self.connected = threading.Event()
        self.running = threading.Event()
        self.read_callback = None
        self.running.set()
        self.start()

    def connect(self, port_name: str, baud_rate: int) -> None:
        self.disconnect()
        self.port_name = port_name
        self.baud_rate = baud_rate
        try:
            self.serial_port = serial.Serial(port_name, baud_rate, timeout=0.1)
            self.connected.set()
            logging.info("Serial connected to %s at %d", port_name, baud_rate)
        except serial.SerialException as exc:
            self.disconnect()
            raise RuntimeError(f"Unable to open serial port {port_name}: {exc}") from exc

    def disconnect(self) -> None:
        self.connected.clear()
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except OSError:
                pass
            self.serial_port = None

    def stop(self) -> None:
        self.running.clear()
        self.disconnect()

    def run(self) -> None:
        while self.running.is_set():
            if self.serial_port is not None and self.serial_port.in_waiting:
                try:
                    line = self.serial_port.readline().decode("utf-8", errors="replace").strip()
                    if self.read_callback is not None and line:
                        self.read_callback(line)
                except Exception:
                    pass
            time.sleep(0.05)

    def send(self, payload: str, terminator: str = "\n") -> None:
        if self.serial_port is None or not self.connected.is_set():
            raise RuntimeError("Serial port is not connected")
        message = payload.strip() + terminator
        self.serial_port.write(message.encode("utf-8"))
        logging.debug("Sent serial payload: %s", message)

    def is_connected(self) -> bool:
        return self.connected.is_set()


class MotionSimulatorApp:
    """GUI app for ACC telemetry, motion cues, and Arduino serial control."""

    def __init__(self, receiver: AccTelemetryReceiver, cue: TwoDoFMotionCue) -> None:
        self.receiver = receiver
        self.cue = cue
        self.serial_controller = SerialController()
        self.profile_manager = ProfileManager(PROFILE_PATH)
        self.actuator_output = SerialActuatorOutput(self.serial_controller)
        self.latest_frame: Optional[TelemetryFrame] = None
        self.platform_command: Optional[PlatformCommand] = None
        self.telemetry_history: list[TelemetryFrame] = []
        self.max_history = 120
        self.root = tk.Tk()
        self.root.title("ACC 2DOF Motion Simulator")
        self.root.geometry("1140x780")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.chart_mode_var = tk.StringVar(value="One graph")
        self.sample_start_time = time.time()
        self.pitch_mapping_source_vars = [tk.StringVar(value="None") for _ in range(10)]
        self.pitch_mapping_percent_vars = [tk.DoubleVar(value=0.0) for _ in range(10)]
        self.roll_mapping_source_vars = [tk.StringVar(value="None") for _ in range(10)]
        self.roll_mapping_percent_vars = [tk.DoubleVar(value=0.0) for _ in range(10)]
        self.axis_output_history: list[tuple[float, float, float]] = []
        self.pitch_output_var = tk.StringVar(value="-")
        self.roll_output_var = tk.StringVar(value="-")
        self.selected_profile: Optional[AxisProfile] = None
        # Per-telemetry range values (±range). Keys correspond to the internal value keys
        # used in the UI: 'pitch','roll','speed','g_long','g_lat','g_vert'
        self.telemetry_range_vars: dict[str, tk.DoubleVar] = {
            "pitch": tk.DoubleVar(value=15.0),
            "roll": tk.DoubleVar(value=15.0),
            "speed": tk.DoubleVar(value=100.0),
            "g_long": tk.DoubleVar(value=3.0),
            "g_lat": tk.DoubleVar(value=3.0),
            "g_vert": tk.DoubleVar(value=2.0),
        }

        self._create_widgets()
        self._apply_initial_settings()
        self._schedule_update()

    def _create_widgets(self) -> None:
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        self.serial_tab = ttk.Frame(notebook)
        self.game_tab = ttk.Frame(notebook)
        self.axis_tab = ttk.Frame(notebook)
        self.axis_output_tab = ttk.Frame(notebook)

        notebook.add(self.serial_tab, text="Serial Communication")
        notebook.add(self.game_tab, text="Game Output")
        notebook.add(self.axis_tab, text="Axis Adjustments")
        notebook.add(self.axis_output_tab, text="Axis Output")

        self._build_serial_tab()
        self._build_game_tab()
        self._build_axis_tab()
        self._build_axis_output_tab()

    def _apply_initial_settings(self) -> None:
        """Capture current UI values as the applied settings used for output."""
        def _list_get(vars_list):
            return [v.get() for v in vars_list]

        # Ensure mapping lists have sensible defaults
        pitch_sources = _list_get(self.pitch_mapping_source_vars)
        pitch_percents = [float(v.get()) for v in self.pitch_mapping_percent_vars]
        roll_sources = _list_get(self.roll_mapping_source_vars)
        roll_percents = [float(v.get()) for v in self.roll_mapping_percent_vars]

        self.applied_settings = {
            "pitch_mapping_sources": pitch_sources,
            "pitch_mapping_percents": pitch_percents,
            "roll_mapping_sources": roll_sources,
            "roll_mapping_percents": roll_percents,
            "telemetry_ranges": {k: float(v.get()) for k, v in self.telemetry_range_vars.items()},
            "pitch_limit": float(self.pitch_limit_var.get()),
            "roll_limit": float(self.roll_limit_var.get()),
            "pitch_type": self.pitch_type_var.get(),
            "roll_type": self.roll_type_var.get(),
            "pitch_scale": float(self.pitch_scale_var.get()),
            "roll_scale": float(self.roll_scale_var.get()),
            "pitch_accel_gain": float(self.pitch_accel_gain_var.get()),
            "roll_accel_gain": float(self.roll_accel_gain_var.get()),
            "smoothing": float(self.smoothing_var.get()),
        }

    def _build_serial_tab(self) -> None:
        frame = ttk.Frame(self.serial_tab, padding=12)
        frame.pack(fill=tk.BOTH, expand=True)

        ttk.Label(frame, text="Arduino UNO Serial Settings", font=(None, 12, "bold")).grid(row=0, column=0, columnspan=2, sticky=tk.W)

        ttk.Label(frame, text="COM Port:").grid(row=1, column=0, sticky=tk.W, pady=6)
        self.port_var = tk.StringVar(value="COM3")
        self.port_entry = ttk.Combobox(frame, textvariable=self.port_var, values=self._available_serial_ports(), width=20)
        self.port_entry.grid(row=1, column=1, sticky=tk.W, pady=6)

        ttk.Label(frame, text="Baud Rate:").grid(row=2, column=0, sticky=tk.W, pady=6)
        self.baud_var = tk.StringVar(value="115200")
        self.baud_entry = ttk.Combobox(frame, textvariable=self.baud_var, values=["9600", "115200", "250000"], width=20)
        self.baud_entry.grid(row=2, column=1, sticky=tk.W, pady=6)

        self.connect_button = ttk.Button(frame, text="Connect", command=self._toggle_serial_connection)
        self.connect_button.grid(row=3, column=0, pady=12, sticky=tk.W)

        self.serial_status_var = tk.StringVar(value="Disconnected")
        ttk.Label(frame, textvariable=self.serial_status_var).grid(row=3, column=1, sticky=tk.W, pady=12)

        ttk.Separator(frame, orient=tk.HORIZONTAL).grid(row=4, column=0, columnspan=2, sticky="ew", pady=12)

        ttk.Label(frame, text="Send custom command:").grid(row=5, column=0, sticky=tk.W, pady=6)
        self.custom_command_var = tk.StringVar(value="[A512][B512]")
        ttk.Entry(frame, textvariable=self.custom_command_var, width=40).grid(row=5, column=1, sticky=tk.W, pady=6)

        self.send_button = ttk.Button(frame, text="Send", command=self._send_custom_serial)
        self.send_button.grid(row=6, column=0, sticky=tk.W, pady=6)

        self.auto_send_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            frame,
            text="Send computed platform command to Arduino",
            variable=self.auto_send_var,
        ).grid(row=6, column=1, sticky=tk.W, pady=6)

        ttk.Label(frame, text="Arduino receive log:", font=(None, 10, "bold")).grid(row=7, column=0, columnspan=2, sticky=tk.W, pady=(16, 6))
        self.serial_log = tk.Text(frame, width=72, height=10, state=tk.DISABLED)
        self.serial_log.grid(row=8, column=0, columnspan=2, pady=4)

        self.serial_controller.read_callback = self._append_serial_log

    def _build_game_tab(self) -> None:
        frame = ttk.Frame(self.game_tab, padding=12)
        frame.pack(fill=tk.BOTH, expand=True)

        ttk.Label(frame, text="Live ACC Telemetry", font=(None, 12, "bold")).grid(row=0, column=0, columnspan=4, sticky=tk.W)

        self.use_sample_data_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            frame,
            text="Use simulated telemetry",
            variable=self.use_sample_data_var,
            command=self._update_charts,
        ).grid(row=1, column=0, columnspan=2, sticky=tk.W, pady=6)

        ttk.Label(frame, text="Display charts:").grid(row=1, column=2, sticky=tk.W, pady=6, padx=(20, 0))
        self.chart_mode_combo = ttk.Combobox(
            frame,
            textvariable=self.chart_mode_var,
            values=["One graph", "Separate graphs"],
            state="readonly",
            width=16,
        )
        self.chart_mode_combo.grid(row=1, column=3, sticky=tk.W, pady=6)
        self.chart_mode_combo.bind("<<ComboboxSelected>>", lambda _: self._update_charts())

        labels = [
            ("Pitch [deg]", "pitch"),
            ("Roll [deg]", "roll"),
            ("Speed [km/h]", "speed"),
            ("G Longitudinal", "g_long"),
            ("G Lateral", "g_lat"),
            ("G Vertical", "g_vert"),
            ("Last update", "updated"),
        ]
        self.value_vars = {key: tk.StringVar(value="-") for _, key in labels}

        left_value_frame = ttk.Frame(frame)
        left_value_frame.grid(row=2, column=0, rowspan=7, columnspan=3, sticky="nsew")
        left_value_frame.grid_rowconfigure(0, weight=1)
        left_value_frame.grid_rowconfigure(len(labels) + 1, weight=1)
        left_value_frame.grid_columnconfigure(0, weight=1)
        left_value_frame.grid_columnconfigure(1, weight=1)
        left_value_frame.grid_columnconfigure(2, weight=1)

        for index, (label, key) in enumerate(labels, start=1):
            ttk.Label(left_value_frame, text=label + ":").grid(row=index, column=0, sticky=tk.W, pady=4)
            ttk.Label(left_value_frame, textvariable=self.value_vars[key], width=18, anchor="center").grid(row=index, column=1, sticky="ew", pady=4)
            # Add a range input for live clamping/normalization for all telemetry keys except the timestamp
            if key != "updated":
                range_var = self.telemetry_range_vars.get(key)
                if range_var is None:
                    range_var = tk.DoubleVar(value=0.0)
                    self.telemetry_range_vars[key] = range_var
                ttk.Entry(left_value_frame, textvariable=range_var, width=10).grid(row=index, column=2, sticky=tk.W, pady=4)
            else:
                ttk.Label(left_value_frame, text="", width=10).grid(row=index, column=2, sticky=tk.W, pady=4)

        chart_frame = ttk.Frame(frame)
        chart_frame.grid(row=2, column=3, rowspan=7, columnspan=1, sticky="nsew", padx=(20, 0), pady=(0, 4))
        frame.grid_columnconfigure(0, weight=1)
        frame.grid_columnconfigure(1, weight=1)
        frame.grid_columnconfigure(2, weight=1)
        frame.grid_columnconfigure(3, weight=1)
        frame.grid_rowconfigure(2, weight=1)

        self.figure = Figure(figsize=(6, 4), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.figure, master=chart_frame)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(fill=tk.BOTH, expand=True)
        self._update_charts()

    def _build_axis_tab(self) -> None:
        frame = ttk.Frame(self.axis_tab, padding=12)
        frame.pack(fill=tk.BOTH, expand=True)

        ttk.Label(frame, text="Axis Adjustments", font=(None, 12, "bold")).grid(row=0, column=0, columnspan=4, sticky=tk.W)

        ttk.Label(frame, text="Profile:").grid(row=1, column=0, sticky=tk.W, pady=6)
        self.profile_var = tk.StringVar()
        self.profile_combo = ttk.Combobox(
            frame,
            textvariable=self.profile_var,
            values=self.profile_manager.profile_names(),
            state="readonly",
            width=28,
        )
        self.profile_combo.grid(row=1, column=1, sticky=tk.W, pady=6)
        # populate UI fields when a profile is selected, but do not apply to cue until Apply pressed
        self.profile_combo.bind("<<ComboboxSelected>>", lambda _: self._on_profile_selected())
        button_frame = ttk.Frame(frame)
        button_frame.grid(row=1, column=2, columnspan=2, sticky="ew", pady=6)
        # center the Save/Delete buttons inside the frame
        button_frame.grid_columnconfigure(0, weight=1)
        button_frame.grid_columnconfigure(3, weight=1)
        save_btn = ttk.Button(button_frame, text="Save", command=self._save_profile)
        delete_btn = ttk.Button(button_frame, text="Delete", command=self._delete_profile)
        save_btn.grid(row=0, column=1, padx=6)
        delete_btn.grid(row=0, column=2, padx=6)

        ttk.Separator(frame, orient=tk.HORIZONTAL).grid(row=3, column=0, columnspan=4, sticky="ew", pady=12)

        pitch_frame = ttk.LabelFrame(frame, text="Axis 1", padding=10)
        roll_frame = ttk.LabelFrame(frame, text="Axis 2", padding=10)
        pitch_frame.grid(row=4, column=0, columnspan=2, sticky="nsew", padx=(0, 4), pady=4)
        roll_frame.grid(row=4, column=2, columnspan=2, sticky="nsew", padx=(4, 0), pady=4)
        frame.grid_columnconfigure(0, weight=1)
        frame.grid_columnconfigure(1, weight=1)
        frame.grid_columnconfigure(2, weight=1)
        frame.grid_columnconfigure(3, weight=1)
        frame.grid_rowconfigure(4, weight=1)

        self.pitch_scale_var = tk.DoubleVar(value=self.cue.pitch_scale)
        self.roll_scale_var = tk.DoubleVar(value=self.cue.roll_scale)
        self.pitch_accel_gain_var = tk.DoubleVar(value=self.cue.pitch_accel_gain)
        self.roll_accel_gain_var = tk.DoubleVar(value=self.cue.roll_accel_gain)
        self.pitch_limit_var = tk.DoubleVar(value=15.0)
        self.roll_limit_var = tk.DoubleVar(value=15.0)
        self.pitch_type_var = tk.StringVar(value="rotational")
        self.roll_type_var = tk.StringVar(value="rotational")
        self.smoothing_var = tk.DoubleVar(value=self.cue.smoothing)

        ttk.Label(pitch_frame, text="Type:").grid(row=0, column=0, sticky=tk.W, pady=4)
        self.pitch_type_select = ttk.Combobox(
            pitch_frame,
            textvariable=self.pitch_type_var,
            values=["rotational", "linear"],
            state="readonly",
            width=16,
        )
        self.pitch_type_select.grid(row=0, column=1, sticky=tk.W, pady=4)
        self.pitch_type_var.trace_add("write", self._update_axis_limit_labels)

        self.pitch_limit_label = ttk.Label(pitch_frame, text="Limit (±deg):")
        self.pitch_limit_label.grid(row=1, column=0, sticky=tk.W, pady=4)
        ttk.Entry(pitch_frame, textvariable=self.pitch_limit_var, width=12).grid(row=1, column=1, sticky=tk.W, pady=4)

        ttk.Label(pitch_frame, text="Telemetry source:").grid(row=2, column=0, sticky=tk.W, pady=(8, 2))
        ttk.Label(pitch_frame, text="Max %:").grid(row=2, column=1, sticky=tk.W, pady=(8, 2))
        for index in range(10):
            ttk.Combobox(
                pitch_frame,
                textvariable=self.pitch_mapping_source_vars[index],
                values=TELEMETRY_SOURCE_OPTIONS,
                state="readonly",
                width=20,
            ).grid(row=index + 3, column=0, sticky=tk.W, pady=2)
            ttk.Spinbox(
                pitch_frame,
                from_=0,
                to=100,
                textvariable=self.pitch_mapping_percent_vars[index],
                width=8,
                increment=1,
            ).grid(row=index + 3, column=1, sticky=tk.W, pady=2)

        ttk.Label(roll_frame, text="Type:").grid(row=0, column=0, sticky=tk.W, pady=4)
        self.roll_type_select = ttk.Combobox(
            roll_frame,
            textvariable=self.roll_type_var,
            values=["rotational", "linear"],
            state="readonly",
            width=16,
        )
        self.roll_type_select.grid(row=0, column=1, sticky=tk.W, pady=4)
        self.roll_type_var.trace_add("write", self._update_axis_limit_labels)

        self.roll_limit_label = ttk.Label(roll_frame, text="Limit (±deg):")
        self.roll_limit_label.grid(row=1, column=0, sticky=tk.W, pady=4)
        ttk.Entry(roll_frame, textvariable=self.roll_limit_var, width=12).grid(row=1, column=1, sticky=tk.W, pady=4)

        ttk.Label(roll_frame, text="Telemetry source:").grid(row=2, column=0, sticky=tk.W, pady=(8, 2))
        ttk.Label(roll_frame, text="Max %:").grid(row=2, column=1, sticky=tk.W, pady=(8, 2))
        for index in range(10):
            ttk.Combobox(
                roll_frame,
                textvariable=self.roll_mapping_source_vars[index],
                values=TELEMETRY_SOURCE_OPTIONS,
                state="readonly",
                width=20,
            ).grid(row=index + 3, column=0, sticky=tk.W, pady=2)
            ttk.Spinbox(
                roll_frame,
                from_=0,
                to=100,
                textvariable=self.roll_mapping_percent_vars[index],
                width=8,
                increment=1,
            ).grid(row=index + 3, column=1, sticky=tk.W, pady=2)

        ttk.Button(frame, text="Apply current settings", command=self._set_cue_from_vars).grid(row=5, column=0, sticky=tk.W, pady=14)
        ttk.Label(
            frame,
            text="Current axis output is visible in the Axis Output tab.",
            foreground="gray",
        ).grid(row=5, column=1, columnspan=3, sticky=tk.W, pady=(8, 0))

    def _build_axis_output_tab(self) -> None:
        frame = ttk.Frame(self.axis_output_tab, padding=12)
        frame.pack(fill=tk.BOTH, expand=True)

        ttk.Label(frame, text="Axis Output", font=(None, 12, "bold")).grid(row=0, column=0, columnspan=4, sticky=tk.W)

        ttk.Label(frame, text="Axis 1 output:").grid(row=1, column=0, sticky=tk.W, pady=6)
        ttk.Label(frame, textvariable=self.pitch_output_var, width=16, anchor="center").grid(row=1, column=1, sticky="w", pady=6)
        ttk.Label(frame, text="Axis 2 output:").grid(row=1, column=2, sticky=tk.W, pady=6)
        ttk.Label(frame, textvariable=self.roll_output_var, width=16, anchor="center").grid(row=1, column=3, sticky="w", pady=6)

        graph_frame = ttk.Frame(frame)
        graph_frame.grid(row=2, column=0, columnspan=4, sticky="nsew", pady=(8, 0))
        frame.grid_columnconfigure(0, weight=1)
        frame.grid_columnconfigure(1, weight=1)
        frame.grid_columnconfigure(2, weight=1)
        frame.grid_columnconfigure(3, weight=1)
        frame.grid_rowconfigure(2, weight=1)

        self.output_figure = Figure(figsize=(8, 5), dpi=100)
        self.output_canvas = FigureCanvasTkAgg(self.output_figure, master=graph_frame)
        self.output_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._update_axis_output_graph()

    def _build_axis_control(
        self,
        frame: ttk.Frame,
        row: int,
        label: str,
        var: tk.DoubleVar,
        minimum: float,
        maximum: float,
        step: float = 0.01,
    ) -> None:
        ttk.Label(frame, text=f"{label}:").grid(row=row, column=0, sticky=tk.W, pady=4)
        ttk.Entry(frame, textvariable=var, width=12).grid(row=row, column=1, sticky=tk.W, pady=4)
        ttk.Scale(frame, variable=var, from_=minimum, to=maximum, orient=tk.HORIZONTAL, length=220).grid(row=row, column=2, sticky=tk.W, pady=4)

    def _telemetry_value(self, frame: TelemetryFrame, source: str) -> float:
        return {
            "Pitch": frame.pitch_deg,
            "Roll": frame.roll_deg,
            "Speed": frame.speed_kmh,
            "G Longitudinal": frame.g_force_longitudinal,
            "G Lateral": frame.g_force_lateral,
            "G Vertical": frame.g_force_vertical,
        }.get(source, 0.0)

    def _normalized_telemetry_value(self, frame: TelemetryFrame, source: str, range_override: Optional[float] = None) -> float:
        """Return telemetry value normalized to [-1, +1] based on the provided range or user-defined range.

        If `range_override` is given it is used; otherwise the UI range is used with sensible defaults.
        """
        raw = self._telemetry_value(frame, source)
        # Map source string to the internal key used by telemetry_range_vars
        source_to_key = {
            "Pitch": "pitch",
            "Roll": "roll",
            "Speed": "speed",
            "G Longitudinal": "g_long",
            "G Lateral": "g_lat",
            "G Vertical": "g_vert",
        }

        if range_override is not None:
            r = float(range_override)
        else:
            key = source_to_key.get(source)
            range_var = self.telemetry_range_vars.get(key) if key is not None else None
            if range_var is not None:
                r = float(range_var.get())
            else:
                defaults = {"Pitch": 15.0, "Roll": 15.0, "Speed": 100.0, "G Longitudinal": 3.0, "G Lateral": 3.0, "G Vertical": 2.0}
                r = float(defaults.get(source, 1.0))

        if r <= 0:
            return 0.0
        val = max(min(raw, r), -r)
        return val / r

    def _compute_axis_output(self, frame: TelemetryFrame, is_pitch: bool) -> float:
        # Use the applied settings snapshot for output calculation
        if is_pitch:
            sources = self.applied_settings.get("pitch_mapping_sources", [])
            percents = self.applied_settings.get("pitch_mapping_percents", [])
            limit = float(self.applied_settings.get("pitch_limit", 0.0))
        else:
            sources = self.applied_settings.get("roll_mapping_sources", [])
            percents = self.applied_settings.get("roll_mapping_percents", [])
            limit = float(self.applied_settings.get("roll_limit", 0.0))

        # map telemetry name to internal key for ranges
        source_to_key = {
            "Pitch": "pitch",
            "Roll": "roll",
            "Speed": "speed",
            "G Longitudinal": "g_long",
            "G Lateral": "g_lat",
            "G Vertical": "g_vert",
        }

        total = 0.0
        for src, pct in zip(sources, percents):
            if src == "None":
                continue
            key = source_to_key.get(src)
            range_val = None
            if key is not None:
                range_val = float(self.applied_settings.get("telemetry_ranges", {}).get(key, 0.0))
            normalized = self._normalized_telemetry_value(frame, src, range_override=range_val)
            total += normalized * (float(pct) / 100.0)

        # total is normalized, multiply by applied limit to get axis units
        return total * limit

    def _update_axis_output_values(self, frame: TelemetryFrame) -> None:
        # Compute outputs in axis units using applied settings
        pitch_axis_value = self._compute_axis_output(frame, True)
        roll_axis_value = self._compute_axis_output(frame, False)

        # Clamp to applied limits for display/storage
        pitch_limit = float(self.applied_settings.get("pitch_limit", self.pitch_limit_var.get()))
        roll_limit = float(self.applied_settings.get("roll_limit", self.roll_limit_var.get()))
        pitch_output = self._clamp_axis_value(pitch_axis_value, pitch_limit)
        roll_output = self._clamp_axis_value(roll_axis_value, roll_limit)

        self.pitch_output_var.set(f"{pitch_output:.2f} {self._unit_for_type(self.applied_settings.get('pitch_type', self.pitch_type_var.get()))}")
        self.roll_output_var.set(f"{roll_output:.2f} {self._unit_for_type(self.applied_settings.get('roll_type', self.roll_type_var.get()))}")

        # Store the axis-unit outputs for motor position calculation
        self.current_pitch_output = pitch_output
        self.current_roll_output = roll_output
        self.axis_output_history.append((frame.timestamp, pitch_output, roll_output))
        while len(self.axis_output_history) > self.max_history:
            self.axis_output_history.pop(0)

    def _update_axis_output_graph(self) -> None:
        self.output_figure.clear()
        history = self.axis_output_history
        if not history:
            ax = self.output_figure.add_subplot(2, 1, 1)
            ax.set_title("Axis 1 Output")
            ax.set_xlabel("Seconds")
            ax.set_ylabel(self._unit_for_type(self.pitch_type_var.get()))
            ax.grid(True)
            ax = self.output_figure.add_subplot(2, 1, 2)
            ax.set_title("Axis 2 Output")
            ax.set_xlabel("Seconds")
            ax.set_ylabel(self._unit_for_type(self.roll_type_var.get()))
            ax.grid(True)
            self.output_figure.tight_layout()
            self.output_canvas.draw()
            return

        timestamps = [row[0] - history[0][0] for row in history]
        pitch_values = [row[1] for row in history]
        roll_values = [row[2] for row in history]

        pitch_axis = self.output_figure.add_subplot(2, 1, 1)
        pitch_axis.plot(timestamps, pitch_values, label="Axis 1 output")
        pitch_axis.set_title("Axis 1 Output")
        pitch_axis.set_ylabel(self._unit_for_type(self.pitch_type_var.get()))
        pitch_axis.set_xlabel("Seconds")
        pitch_axis.grid(True)
        if pitch_values:
            pitch_min = min(pitch_values)
            pitch_max = max(pitch_values)
            if pitch_min == pitch_max:
                margin = abs(pitch_min) * 0.1 + 1.0
            else:
                margin = max(0.1, (pitch_max - pitch_min) * 0.1)
            pitch_axis.set_ylim(pitch_min - margin, pitch_max + margin)

        roll_axis = self.output_figure.add_subplot(2, 1, 2)
        roll_axis.plot(timestamps, roll_values, color="tab:orange", label="Axis 2 output")
        roll_axis.set_title("Axis 2 Output")
        roll_axis.set_ylabel(self._unit_for_type(self.roll_type_var.get()))
        roll_axis.set_xlabel("Seconds")
        roll_axis.grid(True)
        if roll_values:
            roll_min = min(roll_values)
            roll_max = max(roll_values)
            if roll_min == roll_max:
                margin = abs(roll_min) * 0.1 + 1.0
            else:
                margin = max(0.1, (roll_max - roll_min) * 0.1)
            roll_axis.set_ylim(roll_min - margin, roll_max + margin)

        self.output_figure.tight_layout()
        self.output_canvas.draw()

    def _update_axis_limit_labels(self, *args) -> None:
        self.pitch_limit_label.config(text=f"Limit (±{self._unit_for_type(self.pitch_type_var.get())}):")
        self.roll_limit_label.config(text=f"Limit (±{self._unit_for_type(self.roll_type_var.get())}):")

    def _unit_for_type(self, axis_type: str) -> str:
        return "mm" if axis_type == "linear" else "deg"

    def _profile_names(self) -> list[str]:
        return self.profile_manager.profile_names()

    def _on_profile_selected(self) -> None:
        """Populate the GUI fields with the selected profile but do not apply to the motion cue or axis output."""
        name = self.profile_var.get().strip()
        if not name:
            return
        profile = self.profile_manager.profiles.get(name)
        if profile is None:
            messagebox.showwarning("Profile not found", f"Profile '{name}' was not found.")
            return
        # Store the selected profile so we can apply it later
        self.selected_profile = profile
        # Load values into UI for viewing, but don't apply mappings yet
        self.pitch_scale_var.set(profile.pitch_scale)
        self.roll_scale_var.set(profile.roll_scale)
        self.pitch_accel_gain_var.set(profile.pitch_accel_gain)
        self.roll_accel_gain_var.set(profile.roll_accel_gain)
        self.pitch_limit_var.set(profile.pitch_limit)
        self.roll_limit_var.set(profile.roll_limit)
        self.pitch_type_var.set(profile.pitch_type)
        self.roll_type_var.set(profile.roll_type)
        self.smoothing_var.set(profile.smoothing)
        # Restore telemetry mapping selections into the input boxes (preview only)
        p_sources = (profile.pitch_mapping_sources or [])[:10]
        p_percents = (profile.pitch_mapping_percents or [])[:10]
        r_sources = (profile.roll_mapping_sources or [])[:10]
        r_percents = (profile.roll_mapping_percents or [])[:10]
        while len(p_sources) < 10:
            p_sources.append("None")
        while len(p_percents) < 10:
            p_percents.append(0.0)
        while len(r_sources) < 10:
            r_sources.append("None")
        while len(r_percents) < 10:
            r_percents.append(0.0)
        for i in range(10):
            self.pitch_mapping_source_vars[i].set(p_sources[i])
            self.pitch_mapping_percent_vars[i].set(float(p_percents[i]))
            self.roll_mapping_source_vars[i].set(r_sources[i])
            self.roll_mapping_percent_vars[i].set(float(r_percents[i]))
        # Note: these values are loaded into the input boxes but not applied until the user presses Apply
        self._update_axis_limit_labels()
        self._append_serial_log(f"Selected profile '{name}' (not applied yet, press 'Apply current settings' to apply)")

    def _load_selected_profile(self) -> None:
        name = self.profile_var.get().strip()
        if not name:
            return
        profile = self.profile_manager.profiles.get(name)
        if profile is None:
            messagebox.showwarning("Profile not found", f"Profile '{name}' was not found.")
            return
        self.pitch_scale_var.set(profile.pitch_scale)
        self.roll_scale_var.set(profile.roll_scale)
        self.pitch_accel_gain_var.set(profile.pitch_accel_gain)
        self.roll_accel_gain_var.set(profile.roll_accel_gain)
        self.pitch_limit_var.set(profile.pitch_limit)
        self.roll_limit_var.set(profile.roll_limit)
        self.pitch_type_var.set(profile.pitch_type)
        self.roll_type_var.set(profile.roll_type)
        self.smoothing_var.set(profile.smoothing)
        # Restore telemetry mapping selections (ensure lists have 10 entries)
        p_sources = (profile.pitch_mapping_sources or [])[:10]
        p_percents = (profile.pitch_mapping_percents or [])[:10]
        r_sources = (profile.roll_mapping_sources or [])[:10]
        r_percents = (profile.roll_mapping_percents or [])[:10]
        # Pad to 10
        while len(p_sources) < 10:
            p_sources.append("None")
        while len(p_percents) < 10:
            p_percents.append(0.0)
        while len(r_sources) < 10:
            r_sources.append("None")
        while len(r_percents) < 10:
            r_percents.append(0.0)
        for i in range(10):
            self.pitch_mapping_source_vars[i].set(p_sources[i])
            self.pitch_mapping_percent_vars[i].set(float(p_percents[i]))
            self.roll_mapping_source_vars[i].set(r_sources[i])
            self.roll_mapping_percent_vars[i].set(float(r_percents[i]))
        self._set_cue_from_vars()
        self._update_axis_limit_labels()
        self._append_serial_log(f"Loaded profile '{name}'")

    def _save_profile(self) -> None:
        name = simpledialog.askstring("Save profile", "Enter a profile name:", parent=self.root)
        if not name:
            return
        pitch_sources = [var.get() for var in self.pitch_mapping_source_vars]
        pitch_percents = [var.get() for var in self.pitch_mapping_percent_vars]
        roll_sources = [var.get() for var in self.roll_mapping_source_vars]
        roll_percents = [var.get() for var in self.roll_mapping_percent_vars]
        profile = AxisProfile(
            name=name,
            pitch_scale=self.pitch_scale_var.get(),
            roll_scale=self.roll_scale_var.get(),
            pitch_accel_gain=self.pitch_accel_gain_var.get(),
            roll_accel_gain=self.roll_accel_gain_var.get(),
            pitch_limit=self.pitch_limit_var.get(),
            roll_limit=self.roll_limit_var.get(),
            pitch_type=self.pitch_type_var.get(),
            roll_type=self.roll_type_var.get(),
            smoothing=self.smoothing_var.get(),
            pitch_mapping_sources=pitch_sources,
            pitch_mapping_percents=pitch_percents,
            roll_mapping_sources=roll_sources,
            roll_mapping_percents=roll_percents,
        )
        self.profile_manager.save_profile(profile)
        self.profile_combo["values"] = self._profile_names()
        self.profile_var.set(name)
        self._append_serial_log(f"Saved profile '{name}'")

    def _delete_profile(self) -> None:
        name = self.profile_var.get().strip()
        if not name:
            messagebox.showinfo("Delete profile", "Select a profile first.")
            return
        if messagebox.askyesno("Delete profile", f"Delete profile '{name}'?"):
            self.profile_manager.delete_profile(name)
            self.profile_combo["values"] = self._profile_names()
            self.profile_var.set("")
            self._append_serial_log(f"Deleted profile '{name}'")

    def _set_cue_from_vars(self) -> None:
        # Update the motion cue object
        self.cue.pitch_scale = self.pitch_scale_var.get()
        self.cue.roll_scale = self.roll_scale_var.get()
        self.cue.pitch_accel_gain = self.pitch_accel_gain_var.get()
        self.cue.roll_accel_gain = self.roll_accel_gain_var.get()
        self.cue.smoothing = self.smoothing_var.get()

        # Capture the current UI mapping and range settings as the applied settings
        pitch_sources = [var.get() for var in self.pitch_mapping_source_vars]
        pitch_percents = [float(var.get()) for var in self.pitch_mapping_percent_vars]
        roll_sources = [var.get() for var in self.roll_mapping_source_vars]
        roll_percents = [float(var.get()) for var in self.roll_mapping_percent_vars]
        telemetry_ranges = {k: float(v.get()) for k, v in self.telemetry_range_vars.items()}

        self.applied_settings = {
            "pitch_mapping_sources": pitch_sources,
            "pitch_mapping_percents": pitch_percents,
            "roll_mapping_sources": roll_sources,
            "roll_mapping_percents": roll_percents,
            "telemetry_ranges": telemetry_ranges,
            "pitch_limit": float(self.pitch_limit_var.get()),
            "roll_limit": float(self.roll_limit_var.get()),
            "pitch_type": self.pitch_type_var.get(),
            "roll_type": self.roll_type_var.get(),
            "pitch_scale": float(self.pitch_scale_var.get()),
            "roll_scale": float(self.roll_scale_var.get()),
            "pitch_accel_gain": float(self.pitch_accel_gain_var.get()),
            "roll_accel_gain": float(self.roll_accel_gain_var.get()),
            "smoothing": float(self.smoothing_var.get()),
        }

        if self.selected_profile is not None:
            self._append_serial_log(f"Applied profile '{self.selected_profile.name}' settings")
        else:
            self._append_serial_log("Applied current axis adjustment settings")

    def _clamp_axis_value(self, value: float, limit: float) -> float:
        if limit <= 0:
            return 0.0
        return max(min(value, limit), -limit)

    def _axis_position(self, value: float, limit: float, axis_type: str) -> int:
        clamped_value = self._clamp_axis_value(value, limit)
        if limit <= 0:
            return 512

        normalized = clamped_value / limit
        if axis_type == "linear":
            normalized = clamped_value / limit
        target = round(512 + normalized * 512)
        return max(0, min(1024, target))

    def _available_serial_ports(self) -> list[str]:
        ports = [port.device for port in serial.tools.list_ports.comports()]
        if not ports:
            ports = ["COM3", "COM4", "COM5"]
        return ports

    def _toggle_serial_connection(self) -> None:
        if self.serial_controller.is_connected():
            self.serial_controller.disconnect()
            self._update_serial_status()
            return

        port = self.port_var.get().strip()
        baud = int(self.baud_var.get()) if self.baud_var.get().isdigit() else 115200
        try:
            self.serial_controller.connect(port, baud)
            self._append_serial_log(f"Connected to {port} @ {baud}")
        except RuntimeError as exc:
            self._append_serial_log(str(exc))
        finally:
            self._update_serial_status()

    def _send_custom_serial(self) -> None:
        try:
            if not self.serial_controller.is_connected():
                raise RuntimeError("Connect to the Arduino before sending data.")
            self.serial_controller.send(self.custom_command_var.get())
            self._append_serial_log(f"Sent: {self.custom_command_var.get()}")
        except RuntimeError as exc:
            self._append_serial_log(str(exc))

    def _append_serial_log(self, message: str) -> None:
        self.serial_log.configure(state=tk.NORMAL)
        self.serial_log.insert(tk.END, message + "\n")
        self.serial_log.see(tk.END)
        self.serial_log.configure(state=tk.DISABLED)

    def _update_game_values(self, frame: TelemetryFrame) -> None:
        # For each telemetry field, clamp to the user-defined ±range and display that value.
        mapping = {
            "pitch": ("Pitch", frame.pitch_deg),
            "roll": ("Roll", frame.roll_deg),
            "speed": ("Speed", frame.speed_kmh),
            "g_long": ("G Longitudinal", frame.g_force_longitudinal),
            "g_lat": ("G Lateral", frame.g_force_lateral),
            "g_vert": ("G Vertical", frame.g_force_vertical),
        }
        for key, (source_name, raw) in mapping.items():
            range_var = self.telemetry_range_vars.get(key)
            r = float(range_var.get()) if range_var is not None else 0.0
            if r > 0:
                limited = max(min(raw, r), -r)
            else:
                limited = raw
            # Format display
            if key == "speed":
                self.value_vars[key].set(f"{limited:.1f}")
            else:
                self.value_vars[key].set(f"{limited:.2f}")

        self.value_vars["updated"].set(time.strftime("%H:%M:%S", time.localtime(frame.timestamp)))

    def _update_charts(self) -> None:
        self.figure.clear()
        if not self.telemetry_history:
            ax = self.figure.add_subplot(1, 1, 1)
            ax.set_title("Telemetry Overview")
            ax.set_xlabel("Seconds")
            ax.set_ylabel("Value")
            ax.grid(True)
            self.figure.tight_layout()
            self.canvas.draw()
            return

        history = self.telemetry_history
        x_values = [frame.timestamp - history[0].timestamp for frame in history]
        if self.chart_mode_var.get() == "One graph":
            ax = self.figure.add_subplot(1, 1, 1)
            series = [
                ([frame.pitch_deg for frame in history], "Pitch [deg]"),
                ([frame.roll_deg for frame in history], "Roll [deg]"),
                ([frame.speed_kmh for frame in history], "Speed [km/h]"),
                ([frame.g_force_longitudinal for frame in history], "G Longitudinal"),
                ([frame.g_force_lateral for frame in history], "G Lateral"),
                ([frame.g_force_vertical for frame in history], "G Vertical"),
            ]
            all_values = []
            for values, label in series:
                ax.plot(x_values, values, label=label)
                all_values.extend(values)

            if all_values:
                y_min = min(all_values)
                y_max = max(all_values)
                if y_min == y_max:
                    margin = abs(y_min) * 0.1 + 1.0
                    ax.set_ylim(y_min - margin, y_max + margin)
                else:
                    margin = max(0.1, (y_max - y_min) * 0.1)
                    ax.set_ylim(y_min - margin, y_max + margin)

            ax.set_xlabel("Seconds")
            ax.set_title("Telemetry Overview")
            ax.legend(loc="upper left", fontsize="small")
            ax.grid(True)
        else:
            plot_data = [
                ("Pitch [deg]", [frame.pitch_deg for frame in history]),
                ("Roll [deg]", [frame.roll_deg for frame in history]),
                ("Speed [km/h]", [frame.speed_kmh for frame in history]),
                ("G Longitudinal", [frame.g_force_longitudinal for frame in history]),
                ("G Lateral", [frame.g_force_lateral for frame in history]),
                ("G Vertical", [frame.g_force_vertical for frame in history]),
            ]
            axes = [self.figure.add_subplot(3, 2, index + 1) for index in range(len(plot_data))]
            for ax, (title, values) in zip(axes, plot_data):
                ax.plot(x_values, values)
                if values:
                    y_min = min(values)
                    y_max = max(values)
                    if y_min == y_max:
                        margin = abs(y_min) * 0.1 + 1.0
                        ax.set_ylim(y_min - margin, y_max + margin)
                    else:
                        margin = max(0.1, (y_max - y_min) * 0.1)
                        ax.set_ylim(y_min - margin, y_max + margin)
                ax.set_title(title)
                ax.grid(True)
                ax.set_xlabel("Seconds")
        self.figure.tight_layout()
        self.canvas.draw()

    def _generate_sample_frame(self) -> TelemetryFrame:
        elapsed = time.time() - self.sample_start_time
        pitch = math.sin(elapsed * 0.8) * 12.0
        roll = math.cos(elapsed * 1.1) * 10.0
        g_long = math.sin(elapsed * 0.5) * 2.4
        g_lat = math.cos(elapsed * 0.7) * 2.0
        g_vert = math.sin(elapsed * 1.3) * 1.8
        speed = max(40.0 + math.sin(elapsed * 0.25) * 50.0, 0.0)
        return TelemetryFrame(
            timestamp=time.time(),
            pitch_deg=pitch,
            roll_deg=roll,
            g_force_longitudinal=g_long,
            g_force_lateral=g_lat,
            g_force_vertical=g_vert,
            speed_kmh=speed,
        )

    def _update_serial_status(self) -> None:
        status = "Connected" if self.serial_controller.is_connected() else "Disconnected"
        self.serial_status_var.set(status)
        self.connect_button.config(text="Disconnect" if self.serial_controller.is_connected() else "Connect")

    def _schedule_update(self) -> None:
        if self.use_sample_data_var.get():
            frame = self._generate_sample_frame()
        else:
            frame = self.receiver.get_latest()

        if frame is not None:
            self.latest_frame = frame
            self.telemetry_history.append(frame)
            while len(self.telemetry_history) > self.max_history:
                self.telemetry_history.pop(0)
            # Do not auto-apply GUI vars each update; only apply when user presses 'Apply current settings' or when loading a profile
            self.platform_command = self.cue.compute(frame)
            self._update_game_values(frame)
            self._update_axis_output_values(frame)
            self._update_charts()
            self._update_axis_output_graph()
            if self.auto_send_var.get() and self.serial_controller.is_connected():
                try:
                    # Use applied settings for limits and axis types when sending commands
                    pitch_limit = float(self.applied_settings.get("pitch_limit", self.pitch_limit_var.get()))
                    roll_limit = float(self.applied_settings.get("roll_limit", self.roll_limit_var.get()))
                    pitch_type = self.applied_settings.get("pitch_type", self.pitch_type_var.get())
                    roll_type = self.applied_settings.get("roll_type", self.roll_type_var.get())

                    pitch_position = self._axis_position(
                        self.current_pitch_output,
                        pitch_limit,
                        pitch_type,
                    )
                    roll_position = self._axis_position(
                        self.current_roll_output,
                        roll_limit,
                        roll_type,
                    )
                    self.actuator_output.send_positions(pitch_position, roll_position)
                    self._append_serial_log(
                        f"Actuator command sent: [A{pitch_position}][B{roll_position}]"
                    )
                except RuntimeError as exc:
                    self._append_serial_log(str(exc))
                    self._update_serial_status()

        self._update_serial_status()
        self.root.after(100, self._schedule_update)

    def run(self) -> None:
        self.root.mainloop()

    def on_close(self) -> None:
        self.serial_controller.stop()
        self.receiver.stop()
        self.root.destroy()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="2DOF motion simulator for Assetto Corsa Competizione")
    parser.add_argument("--host", default="0.0.0.0", help="UDP listen address for ACC telemetry")
    parser.add_argument("--port", type=int, default=9996, help="UDP port used by ACC telemetry")
    parser.add_argument("--pitch-scale", type=float, default=1.0, help="Scale factor for pitch angle input")
    parser.add_argument("--roll-scale", type=float, default=1.0, help="Scale factor for roll angle input")
    parser.add_argument("--pitch-accel-gain", type=float, default=4.0, help="Longitudinal g-force gain added to pitch cue")
    parser.add_argument("--roll-accel-gain", type=float, default=4.0, help="Lateral g-force gain added to roll cue")
    parser.add_argument("--max-angle", type=float, default=15.0, help="Maximum platform angle in degrees")
    parser.add_argument("--log-file", type=Path, help="Optional CSV file path for recorded platform commands")
    parser.add_argument("--verbose", action="store_true", help="Enable debug output")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    receiver = AccTelemetryReceiver(host=args.host, port=args.port)
    receiver.start()

    cue = TwoDoFMotionCue(
        pitch_scale=args.pitch_scale,
        roll_scale=args.roll_scale,
        pitch_accel_gain=args.pitch_accel_gain,
        roll_accel_gain=args.roll_accel_gain,
        max_angle=args.max_angle,
    )

    app = MotionSimulatorApp(receiver, cue)
    app.run()


if __name__ == "__main__":
    main()
