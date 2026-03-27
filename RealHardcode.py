# pip install math3d scipy ur-rtde transforms3d

from math import radians, pi

import math3d as m3d
import numpy as np
from scipy.spatial.transform import Rotation
import transforms3d.affines
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive


class Robot:
    def __init__(
        self,
        ip="192.168.1.102",
        rotation_y_deg=45,
        rtde_frequency=200.0,
        ur_cap_port=50002,
    ):
        super().__init__()
        self._ip = ip
        self._tf_world_base = m3d.Transform()
        self._tf_world_base.orient.rotate_yb(radians(rotation_y_deg))
        self._rtde_r = RTDEReceive(ip, rtde_frequency)
        self._rtde_c = RTDEControl(
            ip,
            rtde_frequency,
            RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT,
            ur_cap_port,
        )

    def set_pose(self, tf_world_target, *args, **kwargs):
        tf_base_target = self._tf_world_base.inverse * tf_world_target

        pose = tf_base_target.matrix
        position = pose[:3, 3]
        orientation_raw_rotvec = Rotation.from_matrix(pose[:3, :3]).as_rotvec()
        pose_ur = list(position) + list(orientation_raw_rotvec)
        self._rtde_c.moveL(pose_ur, kwargs.get("vel", 0.1), kwargs.get("acc", 0.1))

    def get_pose_raw(self):
        if not self._rtde_r.isConnected():
            self._rtde_r.reconnect()
        pose_raw = self._rtde_r.getActualTCPPose()
        if len(pose_raw) != 6:
            print('ERROR: Invalid output from getActualTCPPose(): ', pose_raw)
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
            pose.pos.x = 0.1  # if left -x
        elif self._ip == "192.168.1.101":
            pose.pos.x = -0.1  # if left -x
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
    # robot_right = Robot("192.168.1.102", rotation_y_deg=45)
    # robot_right.go_home()
    # robot_right.move_tool_xyzypr(x=0,y=-0.0,z=0,pitch=radians(0), roll=radians(0), yaw=0, acc=0.1, vel=0.1)
    # robot_right.move_tool_ypr(yaw=radians(0), pitch=radians(0), roll=radians(0))
    # for i in range(3):
    #    robot_right.move_tool_xyz(x=0.1, y=0.0, z=0, acc=0.1, vel=0.1) 
    #    robot_right.move_tool_xyz(x=-0.1, y=0.0, z=0, acc=0.1, vel=0.1)
        
    # robot_right.close()

    robot_left = Robot("192.168.1.101", rotation_y_deg=-45)
    robot_left.go_home()
    # robot_left.move_tool_ypr(yaw=radians(0), pitch=radians(0), roll=radians(0))
    # for i in range(3):
    #     robot_left.move_tool_xyz(x=-0.1, y=0, z=0, acc=0.1, vel=0.1)
    #     robot_left.move_tool_xyz(x=0.1, y=0, z=0, acc=0.1, vel=0.1)
        
    robot_left.close()
    
    
