#!/usr/bin/env python3

import csv
import math
from pathlib import Path
from typing import List, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Int8

from traj.msg import Expect
from usv_interfaces.msg import McuAux


Point = Tuple[float, float]


class TrajectoryGenerator(Node):
    """Publish a fixed point or a CSV trajectory at a configurable period."""

    STOP = 0
    FORWARD = 1
    RETURN = 2

    def __init__(self) -> None:
        super().__init__('trajgen')

        self.declare_parameter('publish_period', 0.02)
        self.declare_parameter('trajectory_source', 'fixed')
        self.declare_parameter('trajectory_file', '')
        self.declare_parameter('fixed_x', 423.0)
        self.declare_parameter('fixed_y', -472.0)
        self.declare_parameter('initial_mode', self.STOP)
        self.declare_parameter('use_aux_mode', True)

        self._period = float(self.get_parameter('publish_period').value)
        if self._period <= 0.0:
            raise ValueError('publish_period must be greater than zero')

        self._source = str(self.get_parameter('trajectory_source').value).lower()
        self._points = self._load_points()
        self._last_heading = self._initial_heading()
        self._index = 0
        self._mode = self._validated_mode(
            int(self.get_parameter('initial_mode').value)
        )
        self._last_mode = self._mode
        self._use_aux_mode = bool(self.get_parameter('use_aux_mode').value)
        self._local_aux2 = 0
        self._local_aux3 = 0
        self._usv1_aux2 = 0
        self._usv1_aux3 = 0

        self._publisher = self.create_publisher(Expect, 'Trajectory', 10)
        self._mode_subscription = self.create_subscription(
            Int8, 'trajectory_mode', self._mode_callback, 10
        )
        self._local_aux_subscription = self.create_subscription(
            McuAux, 'MCU_AUX', self._local_aux_callback, 10
        )
        self._usv1_aux_subscription = self.create_subscription(
            McuAux, '/USV1/MCU_AUX', self._usv1_aux_callback, 10
        )
        self._timer = self.create_timer(self._period, self._publish_expectation)

        self.get_logger().info(
            f'Trajectory generator started: source={self._source}, '
            f'points={len(self._points)}, period={self._period:.3f}s'
        )
        self.get_logger().info(
            'Publish std_msgs/msg/Int8 on /trajectory_mode: '
            '0=stop, 1=forward, 2=return.'
        )

    def _load_points(self) -> List[Point]:
        if self._source == 'fixed':
            return [
                (
                    float(self.get_parameter('fixed_x').value),
                    float(self.get_parameter('fixed_y').value),
                )
            ]

        if self._source != 'csv':
            raise ValueError("trajectory_source must be either 'fixed' or 'csv'")

        configured_path = str(self.get_parameter('trajectory_file').value).strip()
        if configured_path:
            csv_path = Path(configured_path).expanduser()
        else:
            csv_path = (
                Path(get_package_share_directory('traj'))
                / 'data'
                / 'trajectory.csv'
            )

        if not csv_path.is_file():
            raise FileNotFoundError(f'Trajectory CSV not found: {csv_path}')

        points: List[Point] = []
        with csv_path.open('r', encoding='utf-8-sig', newline='') as csv_file:
            reader = csv.DictReader(csv_file)
            if reader.fieldnames is None or not {'x', 'y'}.issubset(reader.fieldnames):
                raise ValueError('Trajectory CSV must contain x and y columns')
            for row_number, row in enumerate(reader, start=2):
                try:
                    points.append((float(row['x']), float(row['y'])))
                except (TypeError, ValueError) as error:
                    raise ValueError(
                        f'Invalid trajectory point at CSV row {row_number}'
                    ) from error

        if not points:
            raise ValueError('Trajectory CSV does not contain any points')

        self.get_logger().info(f'Loaded trajectory CSV: {csv_path}')
        return points

    def _initial_heading(self) -> float:
        if len(self._points) < 2:
            return 0.0
        first = self._points[0]
        second = self._points[1]
        return math.atan2(second[1] - first[1], second[0] - first[0])

    def _validated_mode(self, mode: int) -> int:
        if mode not in (self.STOP, self.FORWARD, self.RETURN):
            fallback = getattr(self, '_mode', self.STOP)
            self.get_logger().warning(
                f'Unsupported trajectory mode {mode}; using mode {fallback}'
            )
            return fallback
        return mode

    def _mode_callback(self, message: Int8) -> None:
        requested_mode = self._validated_mode(int(message.data))
        self._set_mode(requested_mode, 'trajectory_mode topic')

    def _local_aux_callback(self, message: McuAux) -> None:
        self._local_aux2 = int(message.aux2)
        self._local_aux3 = int(message.aux3)
        self._update_mode_from_aux()

    def _usv1_aux_callback(self, message: McuAux) -> None:
        self._usv1_aux2 = int(message.aux2)
        self._usv1_aux3 = int(message.aux3)
        self._update_mode_from_aux()

    def _update_mode_from_aux(self) -> None:
        if not self._use_aux_mode:
            return

        requested_mode = None
        if (
            (self._local_aux3 == 0 and self._local_aux2 == 0)
            or (
                self._usv1_aux3 == 0
                and self._local_aux2 == 1
                and self._usv1_aux2 == 1
            )
        ):
            requested_mode = self.STOP
        if (
            (self._local_aux3 == 1 and self._local_aux2 == 0)
            or (
                self._usv1_aux3 == 1
                and self._local_aux2 == 1
                and self._usv1_aux2 == 1
            )
        ):
            requested_mode = self.FORWARD
        if (
            (self._local_aux3 == 2 and self._local_aux2 == 0)
            or (
                self._usv1_aux3 == 2
                and self._local_aux2 == 1
                and self._usv1_aux2 == 1
            )
        ):
            requested_mode = self.RETURN

        if requested_mode is not None:
            self._set_mode(requested_mode, 'MCU AUX')

    def _set_mode(self, requested_mode: int, source: str) -> None:
        if requested_mode != self._mode:
            self.get_logger().info(
                f'Trajectory mode changed by {source}: '
                f'{self._mode} -> {requested_mode}'
            )
            self._mode = requested_mode

    def _apply_mode_transition(self) -> None:
        if self._mode in (self.STOP, self.FORWARD):
            self._index = 0
        else:
            self._index = len(self._points) - 1
        self._last_mode = self._mode

    def _publish_expectation(self) -> None:
        if self._mode != self._last_mode:
            self._apply_mode_transition()

        previous_index = self._index
        if self._mode == self.STOP:
            self._index = 0
        elif self._mode == self.FORWARD and self._index < len(self._points) - 1:
            self._index += 1
        elif self._mode == self.RETURN and self._index > 0:
            self._index -= 1

        previous_point = self._points[previous_index]
        current_point = self._points[self._index]
        delta_x = current_point[0] - previous_point[0]
        delta_y = current_point[1] - previous_point[1]

        if self._mode == self.STOP or self._index == previous_index:
            speed_x = 0.0
            speed_y = 0.0
        else:
            speed_x = delta_x / self._period
            speed_y = delta_y / self._period
            if delta_x != 0.0 or delta_y != 0.0:
                self._last_heading = math.atan2(delta_y, delta_x)

        message = Expect()
        message.position_x = current_point[0]
        message.position_y = current_point[1]
        message.position_h = self._last_heading
        message.speed_x = speed_x
        message.speed_y = speed_y
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
