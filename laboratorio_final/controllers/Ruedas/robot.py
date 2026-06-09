from __future__ import annotations

from controller import Robot
from typing import Optional

from wheel import WheelController
from proximity import ProximityController
from estimation import Odometry


class EpuckRobot:
    def __init__(
        self,
        wheel_radius_m: float = 0.0205,
        axle_length_m: float = 0.052,
        x0: float = 0.0,
        y0: float = 0.0,
        theta0: float = 0.0,
    ):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.wheel_radius_m = float(wheel_radius_m)
        self.axle_length_m = float(axle_length_m)

        self.wheels = WheelController(self.robot, self.timestep)
        self.proximity = ProximityController(self.robot, self.timestep)

        self.odometry = Odometry(x=float(x0), y=float(y0), theta=float(theta0))

        self._prev_left_rad: Optional[float] = None
        self._prev_right_rad: Optional[float] = None

    def step(self) -> bool:
        return self.robot.step(self.timestep) != -1

    def encoder_increment(self) -> tuple[float, float, float, float, float, float]:
        """
        Lee encoders, actualiza la pose acumulada (x, y, theta) y retorna los
        incrementos del paso actual.

        Retorna: (enc_left_rad, enc_right_rad, d_left_rad, d_right_rad,
                  delta_s_m, delta_theta_rad)
        """
        left_rad, right_rad = self.wheels.get_positions()
        left_rad = float(left_rad)
        right_rad = float(right_rad)

        if self._prev_left_rad is None or self._prev_right_rad is None:
            self._prev_left_rad = left_rad
            self._prev_right_rad = right_rad
            return left_rad, right_rad, 0.0, 0.0, 0.0, 0.0

        d_left = left_rad - self._prev_left_rad
        d_right = right_rad - self._prev_right_rad

        self._prev_left_rad = left_rad
        self._prev_right_rad = right_rad

        delta_s_l = self.wheel_radius_m * d_left
        delta_s_r = self.wheel_radius_m * d_right

        delta_s = (delta_s_r + delta_s_l) / 2.0
        delta_theta = 0.0
        if self.axle_length_m != 0.0:
            delta_theta = (delta_s_r - delta_s_l) / self.axle_length_m

        self.odometry.update(delta_s, delta_theta)

        return left_rad, right_rad, d_left, d_right, float(delta_s), float(delta_theta)

    def get_position(self) -> tuple[float, float, float]:
        """Retorna la pose estimada por odometría: (x_m, y_m, theta_rad)."""
        return self.odometry.pose()

    def stop(self) -> None:
        self.wheels.stop()
