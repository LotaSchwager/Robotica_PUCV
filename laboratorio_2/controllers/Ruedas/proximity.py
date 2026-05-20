from __future__ import annotations

import bisect
import math
from typing import Iterable


class ProximityController:
    """Proximity/Distance sensors helper for the e-puck.

    Lab 2 requires treating the front sensors as distance measurements (z_k).
    Webots DistanceSensors expose a lookup table that maps distance->value;
    we invert it (value->distance) to work in meters.
    """

    # El e-puck tiene 8 sensores de proximidad llamados 'ps0' a 'ps7'
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

        # For each sensor: (sorted_values, sorted_distances)
        self._inv_tables = [
            self._build_inverse_lookup_table(sensor) for sensor in self.sensors
        ]

    def _build_inverse_lookup_table(self, sensor):
        """Build a (value->distance) table from Webots lookup table.

        Webots stores entries as triples: [distance, value, noise].
        """
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

        # Sort by value to allow fast inversion via bisect.
        pairs.sort(key=lambda t: t[0])
        values = [v for v, _ in pairs]
        distances = [d for _, d in pairs]
        return values, distances

    def _value_to_distance(self, idx: int, value: float) -> float:
        table = self._inv_tables[idx]
        if table is None:
            # Fallback monotonic mapping (raw proximity -> pseudo distance in meters).
            # Used only if the lookup table is not available.
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
        """Convert sensor values into distance estimates in meters."""
        vals = list(values)
        if len(vals) != self.NUM_SENSORS:
            raise ValueError(f"Expected {self.NUM_SENSORS} sensor values, got {len(vals)}")
        return [self._value_to_distance(i, vals[i]) for i in range(self.NUM_SENSORS)]

    def get_distances_m(self) -> list[float]:
        return self.compute_distances_m(self.get_values())

    def front_distance_m(self, distances_m: list[float]) -> float:
        """Distance to the nearest obstacle in front (meters)."""
        d0 = float(distances_m[self.FRONT_SENSORS[0]])
        d7 = float(distances_m[self.FRONT_SENSORS[1]])
        if math.isnan(d0):
            return d7
        if math.isnan(d7):
            return d0
        return min(d0, d7)

    def side_distances_m(self, distances_m: list[float]) -> tuple[float, float]:
        """(left, right) side obstacle distances (meters)."""
        left_candidates = [float(distances_m[i]) for i in self.LEFT_SIDE_SENSORS]
        right_candidates = [float(distances_m[i]) for i in self.RIGHT_SIDE_SENSORS]

        left = min((d for d in left_candidates if not math.isnan(d)), default=math.nan)
        right = min((d for d in right_candidates if not math.isnan(d)), default=math.nan)
        return left, right

    def side_proximity_values(self, values: list[float]) -> tuple[float, float]:
        """(left, right) side proximity in raw units (higher means closer)."""
        left = max(values[i] for i in self.LEFT_SIDE_SENSORS)
        right = max(values[i] for i in self.RIGHT_SIDE_SENSORS)
        return float(left), float(right)

    def is_obstacle_ahead(self, threshold: float = 80.0) -> bool:
        """Raw-threshold obstacle detector (kept for Lab 1 compatibility)."""
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
