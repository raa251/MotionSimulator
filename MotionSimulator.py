"""Assetto Corsa Competizione 2DOF motion simulator core.

This script listens for ACC UDP telemetry, extracts local car motion data,
and converts it into a 2 degrees-of-freedom (pitch/roll) motion cue.
"""

from __future__ import annotations

import argparse
import math
import logging
import queue
import socket
import struct
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


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

        if player_index >= 64:  # sanity guard
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
        # Motion packet may include world velocity in meters per second.
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
            pitch_deg=self._clamp(pitch, -self.max_angle, self.max_angle),
            roll_deg=self._clamp(roll, -self.max_angle, self.max_angle),
            timestamp=frame.timestamp,
        )

    def _lowpass(self, value: float, previous: float) -> float:
        return previous * self.smoothing + value * (1.0 - self.smoothing)

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(min(value, maximum), minimum)


class ConsoleActuatorOutput:
    """Default output driver that logs platform commands to the console."""

    def send(self, command: PlatformCommand) -> None:
        logging.info(
            "Platform command: pitch=%.2f°, roll=%.2f° at %.3f",
            command.pitch_deg,
            command.roll_deg,
            command.timestamp,
        )


class CsvActuatorLogger:
    """Write motion commands to a CSV file for offline analysis."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = open(self.path, "w", encoding="utf-8", newline="")
        self.file.write("timestamp,pitch_deg,roll_deg\n")

    def send(self, command: PlatformCommand) -> None:
        self.file.write(f"{command.timestamp:.6f},{command.pitch_deg:.4f},{command.roll_deg:.4f}\n")
        self.file.flush()

    def close(self) -> None:
        self.file.close()


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

    actuator = ConsoleActuatorOutput()
    csv_logger: Optional[CsvActuatorLogger] = None
    if args.log_file is not None:
        csv_logger = CsvActuatorLogger(args.log_file)

    try:
        logging.info("Waiting for ACC telemetry packets...")
        while True:
            frame = receiver.get_latest()
            if frame is None:
                time.sleep(0.01)
                continue

            command = cue.compute(frame)
            actuator.send(command)
            if csv_logger is not None:
                csv_logger.send(command)

            time.sleep(0.01)
    except KeyboardInterrupt:
        logging.info("Stopping motion simulator")
    finally:
        receiver.stop()
        if csv_logger is not None:
            csv_logger.close()


if __name__ == "__main__":
    main()
