from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class ExponentialMovingAverage:
    alpha: float
    value: Optional[float] = None

    def update(self, measurement: float) -> float:
        if self.value is None:
            self.value = float(measurement)
        else:
            a = float(self.alpha)
            self.value = a * float(measurement) + (1.0 - a) * float(self.value)
        return float(self.value)


@dataclass
class Kalman1D:
    x: float
    P: float
    Q: float
    R: float

    def predict(self, u: float) -> float:
        self.x = float(self.x) + float(u)
        self.P = float(self.P) + float(self.Q)
        return float(self.x)

    def update(self, z: float) -> float:
        y = float(z) - float(self.x)
        S = float(self.P) + float(self.R)
        if S <= 0.0:
            return float(self.x)
        K = float(self.P) / S
        self.x = float(self.x) + K * y
        self.P = (1.0 - K) * float(self.P)
        return float(self.x)

    def step(self, u: float, z: float) -> float:
        self.predict(u)
        return self.update(z)


@dataclass
class Odometry:
    """
    Estimador de pose 2D por odometría diferencial.

    Aplica las ecuaciones del modelo cinemático diferencial (ecs. 5-7 del enunciado):
        x_k   = x_{k-1} + Δs · cos(φ_{k-1} + Δφ/2)
        y_k   = y_{k-1} + Δs · sin(φ_{k-1} + Δφ/2)
        φ_k   = φ_{k-1} + Δφ

    donde Δs y Δφ son el avance lineal y el giro del paso actual,
    calculados a partir de los encoders de rueda.
    """

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def update(self, delta_s: float, delta_theta: float) -> tuple[float, float, float]:
        """
        Integra un incremento de movimiento y retorna la nueva pose (x, y, theta).

        Args:
            delta_s:     desplazamiento lineal del paso (metros).
            delta_theta: cambio de orientación del paso (radianes).
        """
        mid_theta = self.theta + float(delta_theta) / 2.0
        self.x += float(delta_s) * math.cos(mid_theta)
        self.y += float(delta_s) * math.sin(mid_theta)
        self.theta += float(delta_theta)
        return self.x, self.y, self.theta

    def update_with_gyro(self, delta_s: float, omega_z: float, Ts: float) -> tuple[float, float, float]:
        """
        Igual que update() pero usa la velocidad angular del giróscopo (omega_z)
        para el cambio de orientación en vez de los encoders.
        Los encoders siguen calculando el desplazamiento lineal (delta_s).

        Args:
            delta_s:  desplazamiento lineal del paso (metros), de encoders.
            omega_z:  velocidad angular alrededor del eje Z (rad/s), del giróscopo.
            Ts:       período de muestreo (segundos).
        """
        delta_theta = float(omega_z) * float(Ts)
        mid_theta = self.theta + delta_theta / 2.0
        self.x += float(delta_s) * math.cos(mid_theta)
        self.y += float(delta_s) * math.sin(mid_theta)
        self.theta += delta_theta
        return self.x, self.y, self.theta

    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.theta

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)
