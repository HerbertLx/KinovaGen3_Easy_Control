import sys
import os
import time
import threading

from Kinova_kortex2_Gen3_G3L.api_python.examples import utilities

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2

class KinovaManager:
    def __init__(self, ip_address="192.168.8.10"):
        self.ip = ip_address
        self.connection = None
        self.router = None
        self.base = None
        self.base_cyclic = None

        class Args:
            def __init__(self, ip):
                self.ip = ip
                self.username = "admin"
                self.password = "admin"
                self.port = 10000
        self.args = Args(self.ip)

    def connect(self):
        self.connection = utilities.DeviceConnection.createTcpConnection(self.args)
        self.router = self.connection.__enter__()
        self.base = BaseClient(self.router)
        self.base_cyclic = BaseCyclicClient(self.router)
        print(f"✅ Connected to Kinova at {self.ip}")

    def disconnect(self):
        if self.connection:
            self.connection.__exit__(None, None, None)
            print("❌ Disconnected.")

    def _wait_for_action(self):
        e = threading.Event()
        def check(notification):
            if notification.action_event in [Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT]:
                e.set()
        handle = self.base.OnNotificationActionTopic(check, Base_pb2.NotificationOptions())
        return e, handle

    def get_status(self):
        try:
            return self.base_cyclic.RefreshFeedback()
        except Exception as e:
            print(f"Error getting status: {e}")
            return None

    def print_status(self, status):
        """
        全面状态报告：包含末端位姿、捻度、受力、关节详情、系统健康度及夹爪。
        Full Status Report: Pose, Twist, Wrench, Actuators, System Health, and Gripper.
        """
        if not status:
            print("无状态信息 / No status information available.")
            return

        b = status.base
        print(f"\n{' 系统状态报告 SYSTEM STATUS REPORT ':=^85}")

        # 1. 末端位姿与指令参考 (Base Pose & Command Reference)
        print(f"\n[ 末端位姿与指令 / POSE & COMMAND ]")
        print(f"当前位姿/Actual  | XYZ(m): {b.tool_pose_x:7.4f}, {b.tool_pose_y:7.4f}, {b.tool_pose_z:7.4f}")
        print(f"                  | RPY(°): {b.tool_pose_theta_x:7.2f}, {b.tool_pose_theta_y:7.2f}, {b.tool_pose_theta_z:7.2f}")
        print(f"指令参考/Command | XYZ(m): {b.commanded_tool_pose_x:7.4f}, {b.commanded_tool_pose_y:7.4f}, {b.commanded_tool_pose_z:7.4f}")
        print(f"                  | RPY(°): {b.commanded_tool_pose_theta_x:7.2f}, {b.commanded_tool_pose_theta_y:7.2f}, {b.commanded_tool_pose_theta_z:7.2f}")

        # 2. 末端运动捻度 (Tool Twist - Linear & Angular Velocity)
        print(f"\n[ 末端捻度速度 / TOOL TWIST ]")
        print(f"线速度/Linear (m/s) | X: {b.tool_twist_linear_x:9.5f}, Y: {b.tool_twist_linear_y:9.5f}, Z: {b.tool_twist_linear_z:9.5f}")
        print(f"角速度/Angular(°/s) | X: {b.tool_twist_angular_x:9.5f}, Y: {b.tool_twist_angular_y:9.5f}, Z: {b.tool_twist_angular_z:9.5f}")

        # 3. 外部受力情况 (External Wrench / Force-Torque)
        print(f"\n[ 末端受力 / EXTERNAL WRENCH ]")
        print(f"力/Force (N)        | X: {b.tool_external_wrench_force_x:7.2f}, Y: {b.tool_external_wrench_force_y:7.2f}, Z: {b.tool_external_wrench_force_z:7.2f}")
        print(f"力矩/Torque (Nm)     | X: {b.tool_external_wrench_torque_x:7.2f}, Y: {b.tool_external_wrench_torque_y:7.2f}, Z: {b.tool_external_wrench_torque_z:7.2f}")

        # 4. 惯性测量单元 (Base IMU Data)
        print(f"\n[ 基座惯性单元 / BASE IMU DATA ]")
        print(f"加速度/Accel (m/s²) | X: {b.imu_acceleration_x:7.2f}, Y: {b.imu_acceleration_y:7.2f}, Z: {b.imu_acceleration_z:7.2f}")
        print(f"角速度/Gyro (°/s)   | X: {b.imu_angular_velocity_x:7.2f}, Y: {b.imu_angular_velocity_y:7.2f}, Z: {b.imu_angular_velocity_z:7.2f}")

        # 5. 关节执行器详情 (Actuators Detail)
        print(f"\n[ 关节详情 / JOINT DETAILS ]")
        header = f"{'ID':<4} {'Pos(°)':<9} {'Vel(°/s)':<10} {'Trq(Nm)':<9} {'Cur(A)':<8} {'Volt(V)':<8} {'Tmp_C':<8} {'Jitter':<10}"
        print(header)
        print("-" * len(header))
        for i, act in enumerate(status.actuators):
            # 这里的字段完全对应报文中的：position, velocity, torque, current_motor, voltage, temperature_core, jitter_comm
            print(f"J{i+1:<3} {act.position:<9.2f} {act.velocity:<10.3f} {act.torque:<9.2f} "
                  f"{act.current_motor:<8.2f} {act.voltage:<8.1f} {act.temperature_core:<8.1f} {act.jitter_comm:<10}")

        # 6. 夹爪与互联模块 (Gripper & Interconnect)
        print(f"\n[ 夹爪与外设 / GRIPPER & INTERCONNECT ]")
        inter = status.interconnect
        gripper = inter.gripper_feedback.motor
        # 对应报文：gripper_feedback -> motor -> position
        grip_pos = gripper[0].position if gripper else 0.0
        print(f"夹爪位置/Gripper     | Position: {grip_pos:6.2f}% (Voltage: {inter.voltage:4.1f}V)")
        print(f"末端 IMU Acc (m/s²)  | X: {inter.imu_acceleration_x:7.2f}, Y: {inter.imu_acceleration_y:7.2f}, Z: {inter.imu_acceleration_z:7.2f}")

        # 7. 系统健康度 (System Health)
        print(f"\n[ 系统健康度 / SYSTEM HEALTH ]")
        # 对应报文：active_state, arm_voltage, arm_current, temperature_cpu
        print(f"当前状态/Active State | {b.active_state}")
        print(f"主供电/Power Supply  | {b.arm_voltage:4.1f} V / {b.arm_current:4.2f} A")
        print(f"温度/Temperature     | CPU: {b.temperature_cpu:4.1f} °C, Ambient: {b.temperature_ambient:4.1f} °C")
        
        print(f"\n{'='*85}")

    def control_gripper(self, value, dual_grip=True):
        target_pos = 0.0
        if dual_grip:
            target_pos = 100.0 if value >= 0.5 else 0.0
        else:
            target_pos = max(0.0, min(100.0, value))

        cmd = Base_pb2.GripperCommand()
        cmd.mode = Base_pb2.GRIPPER_POSITION
        finger = cmd.gripper.finger.add()
        finger.finger_identifier = 1
        finger.value = target_pos / 100.0
        self.base.SendGripperCommand(cmd)
        time.sleep(0.5)

    def move_angular(self, angles_and_gripper, dual_grip=True):
        angles = angles_and_gripper[:7]
        gripper_val = angles_and_gripper[7]
        action = Base_pb2.Action()
        for i, angle in enumerate(angles):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = i
            joint_angle.value = angle
        e, handle = self._wait_for_action()
        self.base.ExecuteAction(action)
        e.wait()
        self.base.Unsubscribe(handle)
        self.control_gripper(gripper_val, dual_grip)

    def move_cartesian(self, pose_and_gripper, dual_grip=True, skip_gripper=False):
        p = pose_and_gripper
        gripper_val = p[6]
        action = Base_pb2.Action()
        tp = action.reach_pose.target_pose
        tp.x, tp.y, tp.z = p[0], p[1], p[2]
        tp.theta_x, tp.theta_y, tp.theta_z = p[3], p[4], p[5]
        e, handle = self._wait_for_action()
        self.base.ExecuteAction(action)
        e.wait()
        self.base.Unsubscribe(handle)
        if not skip_gripper:
            self.control_gripper(gripper_val, dual_grip)

    def move_velocity(self, speeds, duration_ms=20000):
        """
        speeds: [vx, vy, vz, wx, wy, wz]
        duration_ms: 自动停止阈值，默认200ms（防止程序崩溃导致机器人撞墙）
        """
        # print(f"\nIn kinova_manage, sending velocity command: {speeds} for duration: {duration_ms} ms")
        command = Base_pb2.TwistCommand()
        # 建议使用 REFERENCE_FRAME_BASE，更符合 RL 的坐标系逻辑
        command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
        
        # 设置自动停止阈值
        command.duration = duration_ms 
        
        twist = command.twist
        twist.linear_x, twist.linear_y, twist.linear_z = speeds[0], speeds[1], speeds[2]
        twist.angular_x, twist.angular_y, twist.angular_z = speeds[3], speeds[4], speeds[5]
        
        # 立即发送，不阻塞
        self.base.SendTwistCommand(command)

    def move_relative(self, delta_pose, dual_grip=True):
        current_feedback = self.get_status()
        if not current_feedback: return
        curr = current_feedback.base
        target = [
            curr.tool_pose_x + delta_pose[0],
            curr.tool_pose_y + delta_pose[1],
            curr.tool_pose_z + delta_pose[2],
            curr.tool_pose_theta_x + delta_pose[3],
            curr.tool_pose_theta_y + delta_pose[4],
            curr.tool_pose_theta_z + delta_pose[5],
            0.0
        ]
        self.move_cartesian(target, dual_grip=False, skip_gripper=True)
        gripper_input = delta_pose[6]
        if dual_grip:
            if gripper_input >= 0.5:
                current_grip_pos = current_feedback.interconnect.gripper_feedback.motor[0].position
                new_target = 100.0 if current_grip_pos < 50.0 else 0.0
                self.control_gripper(new_target, dual_grip=False)
        else:
            self.control_gripper(gripper_input, dual_grip=False)


def test_kinova_manager():
    import numpy as np
    arm = KinovaManager()
    arm.connect()
    try:
        
        home_joints = [360.00, 0.00, 180.00, 241.29, 180.00, 61.82, 270.00, 0.0]
        home_joints = [355.22, 4.94, 190.63, 241.29, 181.31, 51.82, 277.83, 100.0]

        CUP_PUT_POSE = np.array([0.2860,  0.2387,  0.12, -180, 0, 180])
        CAMERA_PUT_POSE = np.array([0.5601,  0.3974,  0.2585, 100.65,  -24.28,   58.29]) # camera's final positon

        # arm.move_cartesian(np.array([*CUP_PUT_POSE, 100.0]).tolist(), dual_grip=False)
        # arm.move_cartesian(np.array([*CAMERA_PUT_POSE, 85.0]).tolist(), dual_grip=False) # move to camera put position
        arm.move_angular(home_joints, dual_grip=False)

        exit()

        '''
        status = arm.get_status()
        arm.print_status(status)
        print(f"\ntime.time(): {time.time()}")
        action = np.array([0.00, 0.00, 0.02, 0.0, 0.0, 0.0])
        action = -action
        print(f"action = {action}")
        arm.move_velocity(action)  # 向上移动
        print(f"time.time() after sending velocity command: {time.time()}\n")
        time.sleep(4)
        arm.move_velocity([0.0]*6)  # 停止移动
        time.sleep(1)
        status = arm.get_status()
        arm.print_status(status)

        time.sleep(1)
        arm.move_cartesian(np.array([0.3, 0.07, 0.35, 174.2, 9.16, 91.68, 0.0]).tolist(), dual_grip=True)
        status = arm.get_status()
        arm.print_status(status)
        '''

    finally:
        arm.disconnect()

# test_kinova_manager()