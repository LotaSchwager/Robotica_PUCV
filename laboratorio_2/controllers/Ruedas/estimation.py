from __future__ import annotations

from dataclasses import dataclass
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
        # Innovation
        y = float(z) - float(self.x)
        # Innovation covariance
        S = float(self.P) + float(self.R)
        if S <= 0.0:
            # Degenerate; skip update.
            return float(self.x)

        # Ganancia de Kalman 
        K = float(self.P) / S

        # State update
        self.x = float(self.x) + K * y
        # Covariance update
        self.P = (1.0 - K) * float(self.P)
        return float(self.x)

    def step(self, u: float, z: float) -> float:
        self.predict(u)
        return self.update(z)
