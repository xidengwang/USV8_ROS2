#!/usr/bin/env python3

import time
from typing import Optional

import rclpy
from rclpy.node import Node
import serial
from serial import SerialException

from usv_interfaces.msg import McuAux, McuInput


class McuReceiver(Node):

    HEADER = b'\xCC\xFD'
    FRAME_SIZE = 16
    PORT = '/dev/com_mcu'
    BAUDRATE = 115200
    POLL_PERIOD = 0.01
    RECONNECT_PERIOD = 1.0

    def __init__(self) -> None:
        super().__init__('uart_accept')

        self._serial: Optional[serial.Serial] = None
        self._buffer = bytearray()
        self._next_reconnect_time = 0.0
        self._valid_frames = 0

        self._aux_publisher = self.create_publisher(McuAux, 'MCU_AUX', 10)
        self._input_publisher = self.create_publisher(McuInput, 'MCU_Input', 10)
        self._timer = self.create_timer(self.POLL_PERIOD, self._poll_serial)

        self._try_open_serial() ## 打开串口

    def _try_open_serial(self) -> None:
        if self._serial is not None and self._serial.is_open:
            return

        now = time.monotonic()
        if now < self._next_reconnect_time:
            return
        self._next_reconnect_time = now + self.RECONNECT_PERIOD ## 1s重试一次

        try:
            self._serial = serial.Serial(
                port=self.PORT,
                baudrate=self.BAUDRATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.0,
            )
            # self.get_logger().info(f'Opened MCU serial port {self.PORT} at {self.BAUDRATE} baud')
        except (SerialException, OSError) as error:
            self._serial = None
            self.get_logger().warning(f'Cannot open MCU serial port {self.PORT}: {error}')

    def _close_serial(self) -> None:
        if self._serial is not None:
            try:
                if self._serial.is_open:
                    self._serial.close()
            except (SerialException, OSError):
                pass
        self._serial = None

    def _poll_serial(self) -> None:
        if self._serial is None or not self._serial.is_open:
            self._try_open_serial()
            return

        try:
            waiting = self._serial.in_waiting
            if waiting <= 0:
                return
            self._buffer.extend(self._serial.read(waiting))
            self._decode_buffer()
        except (SerialException, OSError) as error:
            self.get_logger().error(f'MCU serial read failed: {error}')
            self._close_serial()

    def _decode_buffer(self) -> None:
        while True:
            header_index = self._buffer.find(self.HEADER)
            if header_index < 0:
                if self._buffer and self._buffer[-1] == self.HEADER[0]:
                    self._buffer[:] = self._buffer[-1:]
                else:
                    self._buffer.clear()
                return

            if header_index > 0:
                del self._buffer[:header_index]

            if len(self._buffer) < self.FRAME_SIZE:
                return

            frame = bytes(self._buffer[:self.FRAME_SIZE])
            del self._buffer[:self.FRAME_SIZE]
            self._publish_frame(frame)

    @staticmethod
    def _signed_int16(frame: bytes, offset: int) -> int:
        return int.from_bytes(frame[offset:offset + 2], 'little', signed=True)

    def _publish_frame(self, frame: bytes) -> None:
        command_1 = frame[2]
        command_2 = frame[3]

        aux_message = McuAux()
        aux_message.aux1 = command_1 % 10
        aux_message.aux2 = command_1 // 10
        aux_message.aux3 = command_2 % 10
        aux_message.aux4 = command_2 // 10

        input_message = McuInput()
        input_message.u1 = self._signed_int16(frame, 4)
        input_message.u2 = self._signed_int16(frame, 6)
        input_message.u3 = self._signed_int16(frame, 8)
        input_message.u4 = self._signed_int16(frame, 10)
        input_message.motor_speed_1 = self._signed_int16(frame, 12) / 100.0
        input_message.motor_speed_2 = self._signed_int16(frame, 14) / 100.0

        self._aux_publisher.publish(aux_message)
        self._input_publisher.publish(input_message)
        self._valid_frames += 1

        # if self._valid_frames == 1:
        #     self.get_logger().info('Received the first valid MCU feedback frame')

    def destroy_node(self):
        self._close_serial()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = McuReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
