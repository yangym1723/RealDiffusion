from __future__ import annotations

import json
import time
from math import pi, radians
from typing import Optional

import math3d as m3d
import numpy as np
import transforms3d.affines
from scipy.spatial.transform import Rotation
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


class ArduinoValveController:
    HOLD_VALUE = 0
    CHANNEL_NAMES = ("d1", "d2", "d3", "d4")

    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = 115200,
        timeout: float = 1.0,
        startup_delay: float = 2.0,
        initialize: bool = True,
    ):
        self._require_pyserial()
        self._port = port or self.auto_detect_port()
        self._serial = self._open_serial(self._port, baudrate, timeout)
        self._last_command = self._build_command(0, 0, 0, 0)

        # Many Arduino boards reset when the serial port is opened.
        if startup_delay > 0:
            time.sleep(startup_delay)
        if initialize:
            self.set_valves(0, 0, 0, 0)

    @classmethod
    def _require_pyserial(cls):
        if serial is None:
            raise ImportError(
                "pyserial is required for Arduino valve control. "
                "Install it in the active environment first."
            )

    @classmethod
    def list_available_ports(cls) -> list[str]:
        cls._require_pyserial()
        return [port.device for port in serial.tools.list_ports.comports()]

    @classmethod
    def auto_detect_port(cls) -> str:
        cls._require_pyserial()
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            raise RuntimeError(
                "No serial ports were found. Connect the Arduino first, or pass "
                'its port explicitly, for example port="/dev/ttyACM0".'
            )

        ranked_ports = []
        for port in ports:
            description = " ".join(
                filter(None, [port.description, port.manufacturer, port.hwid])
            ).lower()
            score = 0
            if port.device.startswith("/dev/ttyACM"):
                score += 5
            elif port.device.startswith("/dev/ttyUSB"):
                score += 4
            elif port.device.upper().startswith("COM"):
                score += 3
            if "arduino" in description:
                score += 5
            if "usb" in description:
                score += 1
            ranked_ports.append((score, port.device))

        ranked_ports.sort(reverse=True)
        return ranked_ports[0][1]

    @classmethod
    def _normalize_value(cls, channel_name: str, value: int) -> int:
        value = int(value)
        if value == cls.HOLD_VALUE:
            return value
        if not 0 <= value <= 5000:
            raise ValueError(
                f"{channel_name} must be in [0, 5000], or {cls.HOLD_VALUE} to keep the previous output."
            )
        return value

    @classmethod
    def _build_command(cls, d1: int, d2: int, d3: int, d4: int) -> dict[str, int]:
        return {
            "cmd": 2,
            "d1": cls._normalize_value("d1", d1),
            "d2": cls._normalize_value("d2", d2),
            "d3": cls._normalize_value("d3", d3),
            "d4": cls._normalize_value("d4", d4),
        }

    @staticmethod
    def _open_serial(port: str, baudrate: int, timeout: float):
        try:
            return serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout,
                bytesize=8,
                parity="N",
                stopbits=1,
            )
        except serial.SerialException as exc:
            raise RuntimeError(
                f"Failed to open Arduino serial port {port}. "
                "On Ubuntu, make sure the device exists and this user has access "
                "to /dev/ttyACM* or /dev/ttyUSB*."
            ) from exc

    @property
    def port(self) -> str:
        return self._port

    @property
    def last_command(self) -> dict[str, int]:
        return dict(self._last_command)

    def send_command(self, payload: dict[str, int]) -> dict[str, int]:
        message = json.dumps(payload, separators=(",", ":")) + "\n"
        self._serial.write(message.encode("utf-8"))
        self._serial.flush()
        self._last_command = dict(payload)
        return dict(payload)

    def set_valves(self, d1: int = 0, d2: int = 0, d3: int = 0, d4: int = 0) -> dict[str, int]:
        payload = self._build_command(d1, d2, d3, d4)
        return self.send_command(payload)

    def set_channel(self, channel_name: str, value: int, keep_others: bool = True) -> dict[str, int]:
        channel_name = channel_name.lower()
        if channel_name not in self.CHANNEL_NAMES:
            raise ValueError(f"channel_name must be one of {self.CHANNEL_NAMES}.")

        if keep_others:
            payload = self.last_command
        else:
            payload = self._build_command(0, 0, 0, 0)

        payload[channel_name] = self._normalize_value(channel_name, value)
        return self.send_command(payload)

    def stop_all(self) -> dict[str, int]:
        return self.set_valves(0, 0, 0, 0)

    def close(self):
        if self._serial is None:
            return
        try:
            self._serial.close()
        finally:
            self._serial = None


class Robot:
    def __init__(
        self,
        ip: str = "192.168.1.102",
        rotation_y_deg: float = 45,
        rtde_frequency: float = 200.0,
        ur_cap_port: int = 50002,
        valve_controller: Optional[ArduinoValveController] = None,
    ):
        super().__init__()
        self._ip = ip
        self._valve_controller = valve_controller
        self._tf_world_base = m3d.Transform()
        self._tf_world_base.orient.rotate_yb(radians(rotation_y_deg))
        self._rtde_r = RTDEReceive(ip, rtde_frequency)
        self._rtde_c = RTDEControl(
            ip,
            rtde_frequency,
            RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT,
            ur_cap_port,
        )

    def attach_valve_controller(self, valve_controller: ArduinoValveController):
        self._valve_controller = valve_controller
        return self

    def set_valves(self, d1: int = 0, d2: int = 0, d3: int = 0, d4: int = 0) -> dict[str, int]:
        if self._valve_controller is None:
            raise RuntimeError(
                "No ArduinoValveController is attached. "
                "Create one first and call robot.attach_valve_controller(...)."
            )
        return self._valve_controller.set_valves(d1=d1, d2=d2, d3=d3, d4=d4)

    def set_valve_channel(self, channel_name: str, value: int, keep_others: bool = True) -> dict[str, int]:
        if self._valve_controller is None:
            raise RuntimeError(
                "No ArduinoValveController is attached. "
                "Create one first and call robot.attach_valve_controller(...)."
            )
        return self._valve_controller.set_channel(
            channel_name=channel_name, value=value, keep_others=keep_others
        )

    def stop_valves(self) -> dict[str, int]:
        if self._valve_controller is None:
            raise RuntimeError(
                "No ArduinoValveController is attached. "
                "Create one first and call robot.attach_valve_controller(...)."
            )
        return self._valve_controller.stop_all()

    def set_pose(self, tf_world_target, *args, **kwargs):
        tf_base_target = self._tf_world_base.inverse * tf_world_target

        pose = np.asarray(tf_base_target.matrix, dtype=float)
        position = pose[:3, 3].reshape(3)
        orientation_raw_rotvec = np.asarray(
            Rotation.from_matrix(pose[:3, :3]).as_rotvec(),
            dtype=float,
        ).reshape(3)
        pose_ur = position.tolist() + orientation_raw_rotvec.tolist()
        self._rtde_c.moveL(pose_ur, kwargs.get("vel", 0.1), kwargs.get("acc", 0.1))

    def get_pose_raw(self):
        if not self._rtde_r.isConnected():
            self._rtde_r.reconnect()
        pose_raw = self._rtde_r.getActualTCPPose()
        if len(pose_raw) != 6:
            print("ERROR: Invalid output from getActualTCPPose(): ", pose_raw)
            return None
        position = pose_raw[:3]
        orientation = Rotation.from_rotvec(pose_raw[3:]).as_matrix()
        pose = transforms3d.affines.compose(position, orientation, np.ones(3))
        return m3d.Transform(pose)

    def get_pose(self):
        tf_base_target = self.get_pose_raw()
        return self._tf_world_base * tf_base_target

    def move_tool_xyz(self, x=0, y=0, z=0, acc=0.1, vel=0.1):
        pose = self.get_pose()
        tool_pose_change = m3d.Transform()
        tool_pose_change.pos.x = x
        tool_pose_change.pos.y = y
        tool_pose_change.pos.z = z
        pose = pose * tool_pose_change
        self.set_pose(pose, acc=acc, vel=vel)

    def move_tool_ypr(self, yaw=0, pitch=0, roll=0, acc=0.1, vel=0.1):
        pose = self.get_pose()
        pose.orient.rotate_zt(yaw)
        pose.orient.rotate_xt(pitch)
        pose.orient.rotate_yt(roll)
        self.set_pose(pose, acc=acc, vel=vel)

    def move_tool_rpy(self, yaw=0, pitch=0, roll=0, acc=0.1, vel=0.1):
        pose = self.get_pose()
        pose.orient.rotate_yt(roll)
        pose.orient.rotate_xt(pitch)
        pose.orient.rotate_zt(yaw)
        self.set_pose(pose, acc=acc, vel=vel)

    def move_tool_xyzypr(self, x=0, y=0, z=0, pitch=0, roll=0, yaw=0, acc=0.1, vel=0.1):
        pose = self.get_pose()
        pose.orient.rotate_zt(yaw)
        pose.orient.rotate_xt(pitch)
        pose.orient.rotate_yt(roll)
        tool_pose_change = m3d.Transform()
        tool_pose_change.pos.x = x
        tool_pose_change.pos.y = y
        tool_pose_change.pos.z = z
        pose = pose * tool_pose_change
        self.set_pose(pose, acc=acc, vel=vel)

    def go_home(self, acc=0.1, vel=0.1):
        pose = m3d.Transform()
        if self._ip == "192.168.1.102":
            pose.pos.x = 0.1
        elif self._ip == "192.168.1.101":
            pose.pos.x = -0.1
        pose.pos.y = -0.35
        pose.pos.z = 0.15
        pose.orient.rotate_yb(-pi)
        self.set_pose(pose, acc=acc, vel=vel)

    def close(self):
        try:
            self._rtde_c.stopScript()
        except Exception:
            pass
        try:
            self._rtde_c.disconnect()
        except Exception:
            pass
        try:
            self._rtde_r.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    robot_right = None
    valves = None
    try:
        # If auto detection does not pick the right device on Ubuntu,
        # pass the port explicitly, for example ArduinoValveController(port="/dev/ttyACM0").
        valves = ArduinoValveController(port="/dev/ttyACM0")
        # Keep the same world/base alignment as class_robot_new_version.py.
        robot_right = Robot("192.168.1.102", rotation_y_deg=45, valve_controller=valves)

        robot_right.go_home()
        # yellow:three,d1
        # black:two,d3
        robot_right.set_valves(d1=0, d2=0, d3=5000, d4=0)
        robot_right.move_tool_xyz(x=0.0, y=0.0, z=0.0, acc=0.1, vel=0.1)
        robot_right.stop_valves()
    finally:
        if robot_right is not None:
            robot_right.close()
        if valves is not None:
            valves.close()
