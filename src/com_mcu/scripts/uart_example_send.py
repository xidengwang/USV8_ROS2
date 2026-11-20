#!/usr/bin/env python3

import struct
import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
import serial
from serial import SerialException
from sensor_msgs.msg import Imu

from traj.msg import Expect
from usv_interfaces.msg import Controller, McuAux


class McuSender(Node):
    """Pack ROS 2 state and command topics into the legacy 83-byte MCU frame."""

    HEADER_1 = 0xCC
    HEADER_2 = 0xFD
    CONTROL_COMMAND = 0x01
    TRAJECTORY_COMMAND = 0x02
    FRAME_FORMAT = '<BBB20i'
    FRAME_SIZE = struct.calcsize(FRAME_FORMAT)
    INT32_MIN = -(2 ** 31)
    INT32_MAX = 2 ** 31 - 1
    PORT = '/dev/com_mcu'
    BAUDRATE = 115200
    SEND_PERIOD = 0.01
    RECONNECT_PERIOD = 1.0

    def __init__(self) -> None:
        super().__init__('uart_send')

        self._serial: Optional[serial.Serial] = None
        self._next_reconnect_time = 0.0
        self._command = self.TRAJECTORY_COMMAND

        self._position = [0.0, 0.0, 0.0]
        self._velocity = [0.0, 0.0, 0.0]
        self._acceleration = [0.0, 0.0, 0.0]
        self._euler = [0.0, 0.0, 0.0]
        self._angular_velocity = [0.0, 0.0, 0.0]
        self._controller = [0.0, 0.0, 0.0, 0.0]
        self._trajectory = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.create_subscription(Imu, 'imu', self._imu_callback, 10)
        self.create_subscription(Vector3, 'euler_angles', self._euler_callback, 10)
        self.create_subscription(Twist, 'system_speed', self._twist_callback, 10)
        self.create_subscription(Odometry, 'NED_odometry', self._odom_callback, 10)
        self.create_subscription(Expect, 'Trajectory', self._trajectory_callback, 10)
        self.create_subscription(McuAux, 'MCU_AUX', self._aux_callback, 10)
        self.create_subscription(Controller, 'Controller', self._controller_callback, 10)
        self._timer = self.create_timer(self.SEND_PERIOD, self._send_frame)
        self._try_open_serial()

    def _try_open_serial(self) -> None:
        if self._serial is not None and self._serial.is_open:
            return

        now = time.monotonic()
        if now < self._next_reconnect_time:
            return
        self._next_reconnect_time = now + self.RECONNECT_PERIOD

        try:
            self._serial = serial.Serial(
                port=self.PORT,
                baudrate=self.BAUDRATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.0,
            )

        except (SerialException, OSError) as error:
            self._serial = None

    def _close_serial(self) -> None:
        if self._serial is not None:
            try:
                if self._serial.is_open:
                    self._serial.close()
            except (SerialException, OSError):
                pass
        self._serial = None

    def _imu_callback(self, message: Imu) -> None:
        self._acceleration = [
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        ]
        self._angular_velocity = [
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        ]

    def _euler_callback(self, message: Vector3) -> None:
        self._euler = [message.x, message.y, message.z]

    def _twist_callback(self, message: Twist) -> None:
        self._velocity = [message.linear.x, message.linear.y, message.linear.z]

    def _odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self._position = [position.x, position.y, position.z]

    def _trajectory_callback(self, message: Expect) -> None:
        self._trajectory = [
            message.position_x,
            message.position_y,
            message.position_h,
            message.speed_x,
            message.speed_y,
        ]

    def _aux_callback(self, message: McuAux) -> None:
        self._command = (
            self.CONTROL_COMMAND
            if message.aux1 == 1
            else self.TRAJECTORY_COMMAND
        )

    def _controller_callback(self, message: Controller) -> None:
        self._controller = [message.u1, message.u2, message.u3, message.u4]

    @classmethod
    def _scaled_int32(cls, value: float, scale: float) -> int:
        scaled = int(value * scale)
        return max(cls.INT32_MIN, min(cls.INT32_MAX, scaled))

    def _build_frame(self) -> bytes:
        if self._command == self.CONTROL_COMMAND:
            command_values: List[float] = self._controller + [0.0]
        else:
            command_values = self._trajectory

        values = (
            [self._scaled_int32(value, 1000.0) for value in self._position]
            + [self._scaled_int32(value, 1000.0) for value in self._velocity]
            + [self._scaled_int32(value, 1000.0) for value in self._acceleration]
            + [self._scaled_int32(value, 10000.0) for value in self._euler]
            + [self._scaled_int32(value, 10000.0) for value in self._angular_velocity]
            + [self._scaled_int32(value, 1000.0) for value in command_values]
        )

        return struct.pack(
            self.FRAME_FORMAT,
            self.HEADER_1,
            self.HEADER_2,
            self._command,
            *values,
        )

    def _send_frame(self) -> None:
        if self._serial is None or not self._serial.is_open:
            self._try_open_serial()
            return

        try:
            frame = self._build_frame()
            written = self._serial.write(frame)
            if written != len(frame): self.get_logger().error(f'Incomplete MCU serial write: {written}/{len(frame)} bytes')
        except (SerialException, OSError) as error:
            self.get_logger().error(f'MCU serial write failed: {error}')
            self._close_serial()

    def destroy_node(self):
        self._close_serial()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = McuSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
