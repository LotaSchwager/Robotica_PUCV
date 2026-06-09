from __future__ import annotations

import bisect
import math
from typing import Iterable


class ProximityController:
   
    # El e-puck tiene 8 sensores de proximidad, 2 llamados 'ps0' a 'ps7'
    NUM_SENSORS = 8

    FRONT_SENSORS = (0, 7)
    RIGHT_SIDE_SENSORS = (1, 2)
    LEFT_SIDE_SENSORS = (5, 6)

    def __init__(self, robot, timestep):
        self.sensors = []
        for i in range(self.NUM_SENSORS):
            sensor_name = f"ps{i}"
            sensor = robot.getDevice(sensor_name)
            sensor.enable(timestep)
            self.sensors.append(sensor)

        self._inv_tables = [
            self._build_inverse_lookup_table(sensor) for sensor in self.sensors
        ]

    def _build_inverse_lookup_table(self, sensor):
        
        try:
            size = int(sensor.getLookupTableSize())
            table = list(sensor.getLookupTable())
        except Exception:
            return None

        if size <= 0 or len(table) < 3 * size:
            return None

        pairs: list[tuple[float, float]] = []
        for i in range(size):
            distance = float(table[3 * i + 0])
            value = float(table[3 * i + 1])
            pairs.append((value, distance))

        pairs.sort(key=lambda t: t[0])
        values = [v for v, _ in pairs]
        distances = [d for _, d in pairs]
        return values, distances

    def _value_to_distance(self, idx: int, value: float) -> float:
        table = self._inv_tables[idx]
        if table is None:
            v = max(0.0, min(float(value), 4096.0))
            d_min, d_max = 0.01, 0.20
            return d_min + (d_max - d_min) * (1.0 - v / 4096.0)

        values, distances = table
        if not values:
            return math.nan

        v = float(value)
        if v <= values[0]:
            return float(distances[0])
        if v >= values[-1]:
            return float(distances[-1])

        j = bisect.bisect_left(values, v)
        v0, v1 = float(values[j - 1]), float(values[j])
        d0, d1 = float(distances[j - 1]), float(distances[j])
        if v1 == v0:
            return d0

        t = (v - v0) / (v1 - v0)
        return d0 + t * (d1 - d0)

    def get_values(self) -> list[float]:
        """Retorna una lista con los valores actuales de los 8 sensores."""
        return [float(sensor.getValue()) for sensor in self.sensors]

    def compute_distances_m(self, values: Iterable[float]) -> list[float]:
        """Convertir mediciones de los sensores en distancias en metros."""
        vals = list(values)
        if len(vals) != self.NUM_SENSORS:
            raise ValueError(f"Expected {self.NUM_SENSORS} sensor values, got {len(vals)}")
        return [self._value_to_distance(i, vals[i]) for i in range(self.NUM_SENSORS)]

    def get_distances_m(self) -> list[float]:
        return self.compute_distances_m(self.get_values())

    def front_distance_m(self, distances_m: list[float]) -> float:
        # Distancia en metros al obstáculo más cercano en el frente (entre ps0 y ps7).
        d0 = float(distances_m[self.FRONT_SENSORS[0]])
        d7 = float(distances_m[self.FRONT_SENSORS[1]])
        if math.isnan(d0):
            return d7
        if math.isnan(d7):
            return d0
        return min(d0, d7)

    def side_distances_m(self, distances_m: list[float]) -> tuple[float, float]:
        """Distancia hacia derecha e izquierda (metros)."""
        left_candidates = [float(distances_m[i]) for i in self.LEFT_SIDE_SENSORS]
        right_candidates = [float(distances_m[i]) for i in self.RIGHT_SIDE_SENSORS]

        left = min((d for d in left_candidates if not math.isnan(d)), default=math.nan)
        right = min((d for d in right_candidates if not math.isnan(d)), default=math.nan)
        return left, right

    def side_proximity_values(self, values: list[float]) -> tuple[float, float]:
        """(left, right) side proximity in raw units (higher means closer)."""
        # Proximidad derecha e izquierda (mayor es más cerca o mas próximo al reves que distancia).
        left = max(values[i] for i in self.LEFT_SIDE_SENSORS)
        right = max(values[i] for i in self.RIGHT_SIDE_SENSORS)
        return float(left), float(right)

    def is_obstacle_ahead(self, threshold: float = 80.0) -> bool:
        values = self.get_values()
        return values[0] > threshold or values[7] > threshold

    def front_obstacle_hits(self, threshold: float = 80.0) -> tuple[bool, bool]:
        """Devuelve qué sensores frontales detectan obstáculo."""
        values = self.get_values()
        return values[0] > threshold, values[7] > threshold

    def front_obstacle_count(self, threshold: float = 80.0) -> int:
        """Devuelve cuántos sensores frontales están activos."""
        left_hit, right_hit = self.front_obstacle_hits(threshold=threshold)
        return int(left_hit) + int(right_hit)
