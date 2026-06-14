from __future__ import annotations

import math


class OccupancyGrid:
    """
    Grilla de ocupación 2D construida en tiempo de ejecución a partir de lecturas
    de sensores de proximidad.

    Internamente es una matriz de booleanos (False = libre/desconocido,
    True = ocupado).  Se usa ray-casting (Bresenham) para marcar el espacio
    libre a lo largo del haz de cada sensor, y se marcan como ocupadas las
    celdas donde se detecta un obstáculo (con inflación para el radio del robot).

    Índices de celda: (col, row) donde col crece hacia +X y row hacia +Y.
    """

    def __init__(
        self,
        world_min_x: float,
        world_max_x: float,
        world_min_y: float,
        world_max_y: float,
        cell_size_m: float,
        inflation_m: float = 0.0,
    ):
        self.min_x = float(world_min_x)
        self.min_y = float(world_min_y)
        self.cell_size = float(cell_size_m)
        self.inflation = float(inflation_m)

        self.cols = math.ceil((world_max_x - world_min_x) / cell_size_m)
        self.rows = math.ceil((world_max_y - world_min_y) / cell_size_m)

        # False = libre/desconocido, True = ocupado
        self._grid: list[list[bool]] = [
            [False] * self.cols for _ in range(self.rows)
        ]

    # ------------------------------------------------------------------
    # Conversión coordenadas mundo ↔ celda
    # ------------------------------------------------------------------

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        col = int((float(x) - self.min_x) / self.cell_size)
        row = int((float(y) - self.min_y) / self.cell_size)
        return col, row

    def cell_to_world(self, col: int, row: int) -> tuple[float, float]:
        x = self.min_x + (col + 0.5) * self.cell_size
        y = self.min_y + (row + 0.5) * self.cell_size
        return x, y

    # ------------------------------------------------------------------
    # Consultas
    # ------------------------------------------------------------------

    def is_valid(self, col: int, row: int) -> bool:
        return 0 <= col < self.cols and 0 <= row < self.rows

    def is_free(self, col: int, row: int) -> bool:
        if not self.is_valid(col, row):
            return False
        return not self._grid[row][col]

    # ------------------------------------------------------------------
    # Marcado manual de obstáculos (útil para paredes conocidas a priori)
    # ------------------------------------------------------------------

    def mark_occupied(self, col: int, row: int) -> None:
        if self.is_valid(col, row):
            self._grid[row][col] = True

    def mark_rect_obstacle(
        self,
        cx: float,
        cy: float,
        half_x: float,
        half_y: float,
    ) -> None:
        """Marca celdas dentro del rectángulo [cx±half_x, cy±half_y] + inflación."""
        infl = self.inflation
        col_lo, row_lo = self.world_to_cell(cx - half_x - infl, cy - half_y - infl)
        col_hi, row_hi = self.world_to_cell(cx + half_x + infl, cy + half_y + infl)
        for r in range(max(0, row_lo), min(self.rows, row_hi + 1)):
            for c in range(max(0, col_lo), min(self.cols, col_hi + 1)):
                self._grid[r][c] = True

    def mark_wall(
        self,
        cx: float,
        cy: float,
        size_x: float,
        size_y: float,
        rotated_90: bool = False,
    ) -> None:
        if rotated_90:
            half_x, half_y = size_y / 2.0, size_x / 2.0
        else:
            half_x, half_y = size_x / 2.0, size_y / 2.0
        self.mark_rect_obstacle(cx, cy, half_x, half_y)

    def mark_rotated_rect(
        self,
        cx: float,
        cy: float,
        half_x: float,
        half_y: float,
        angle_rad: float = 0.0,
    ) -> None:
        """
        Marca las celdas dentro de un rectángulo centrado en (cx, cy), de
        semiejes (half_x, half_y) y rotado angle_rad respecto al eje X
        (+ inflación).  Una celda se considera ocupada si su centro cae
        dentro del rectángulo inflado.
        """
        hx = float(half_x) + self.inflation
        hy = float(half_y) + self.inflation
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)

        # AABB del rectángulo rotado para acotar la búsqueda de celdas
        ext_x = abs(cos_a) * hx + abs(sin_a) * hy
        ext_y = abs(sin_a) * hx + abs(cos_a) * hy
        col_lo, row_lo = self.world_to_cell(cx - ext_x, cy - ext_y)
        col_hi, row_hi = self.world_to_cell(cx + ext_x, cy + ext_y)

        for r in range(max(0, row_lo), min(self.rows, row_hi + 1)):
            for c in range(max(0, col_lo), min(self.cols, col_hi + 1)):
                x, y = self.cell_to_world(c, r)
                dx, dy = x - cx, y - cy
                # Coordenadas en el marco del rectángulo
                u = dx * cos_a + dy * sin_a
                v = -dx * sin_a + dy * cos_a
                if abs(u) <= hx and abs(v) <= hy:
                    self._grid[r][c] = True

    # ------------------------------------------------------------------
    # Actualización dinámica con sensores (construcción del mapa)
    # ------------------------------------------------------------------

    @staticmethod
    def _bresenham(
        c0: int, r0: int, c1: int, r1: int
    ) -> list[tuple[int, int]]:
        """
        Retorna las celdas del segmento (c0,r0)→(c1,r1) excluyendo el endpoint,
        usando el algoritmo de Bresenham.
        """
        cells: list[tuple[int, int]] = []
        dc = abs(c1 - c0)
        dr = abs(r1 - r0)
        sc = 1 if c1 > c0 else -1
        sr = 1 if r1 > r0 else -1
        err = dc - dr
        c, r = c0, r0
        while (c, r) != (c1, r1):
            cells.append((c, r))
            e2 = 2 * err
            if e2 > -dr:
                err -= dr
                c += sc
            if e2 < dc:
                err += dc
                r += sr
        return cells

    def update_from_sensor(
        self,
        rx: float,
        ry: float,
        rtheta: float,
        sensor_angle_rad: float,
        distance_m: float,
        max_range_m: float = 0.20,
    ) -> None:
        """
        Actualiza la grilla con la lectura de un sensor de proximidad.

        - Las celdas a lo largo del haz (Bresenham) se marcan libres.
        - Si distance_m < max_range_m, la celda del obstáculo se marca ocupada
          (con inflación).
        """
        if math.isnan(distance_m):
            return

        world_angle = float(rtheta) + float(sensor_angle_rad)
        d = min(float(distance_m), float(max_range_m))

        obs_x = float(rx) + d * math.cos(world_angle)
        obs_y = float(ry) + d * math.sin(world_angle)

        r_col, r_row = self.world_to_cell(rx, ry)
        obs_col, obs_row = self.world_to_cell(obs_x, obs_y)

        # Marcar celdas a lo largo del haz como libres (confirmar espacio vacío)
        for c, r_idx in self._bresenham(r_col, r_row, obs_col, obs_row):
            if self.is_valid(c, r_idx):
                self._grid[r_idx][c] = False

        # Marcar obstáculo si el sensor no está saturado
        if float(distance_m) < float(max_range_m) * 0.95:
            self.mark_rect_obstacle(obs_x, obs_y, 0.0, 0.0)

    def update_from_all_sensors(
        self,
        rx: float,
        ry: float,
        rtheta: float,
        distances_m: list[float],
        sensor_angles_rad: list[float],
        max_range_m: float = 0.20,
    ) -> None:
        """Actualiza la grilla con las lecturas de todos los sensores."""
        for angle, dist in zip(sensor_angles_rad, distances_m):
            self.update_from_sensor(rx, ry, rtheta, angle, dist, max_range_m)

    # ------------------------------------------------------------------
    # Fábrica genérica
    # ------------------------------------------------------------------

    @classmethod
    def empty(
        cls,
        width_m: float = 3.0,
        height_m: float = 3.0,
        cell_size_m: float = 0.05,
        inflation_m: float = 0.07,
    ) -> "OccupancyGrid":
        """
        Grilla en blanco centrada en el origen.  El tamaño debe ser mayor que
        el arena real para que el robot nunca llegue al borde de la grilla.
        Por defecto 3×3m es suficiente para arenas de hasta 2×2m.
        """
        return cls(
            world_min_x=-width_m / 2.0,
            world_max_x=width_m / 2.0,
            world_min_y=-height_m / 2.0,
            world_max_y=height_m / 2.0,
            cell_size_m=cell_size_m,
            inflation_m=inflation_m,
        )

    # ------------------------------------------------------------------
    # Utilidad de depuración
    # ------------------------------------------------------------------

    def to_ascii(
        self,
        start: tuple[int, int] | None = None,
        goal: tuple[int, int] | None = None,
        path: list[tuple[int, int]] | None = None,
        robot: tuple[int, int] | None = None,
    ) -> str:
        """
        Representación ASCII de la grilla.
        '#'=ocupado, '.'=libre, 'S'=inicio, 'G'=meta, '*'=ruta, 'O'=robot actual.
        El eje Y se invierte (row 0 = abajo).
        """
        path_set = set(path) if path else set()
        lines = []
        for r in range(self.rows - 1, -1, -1):
            row_chars = []
            for c in range(self.cols):
                if robot and (c, r) == robot:
                    row_chars.append("O")
                elif start and (c, r) == start:
                    row_chars.append("S")
                elif goal and (c, r) == goal:
                    row_chars.append("G")
                elif (c, r) in path_set:
                    row_chars.append("*")
                elif self._grid[r][c]:
                    row_chars.append("#")
                else:
                    row_chars.append(".")
            lines.append("".join(row_chars))
        return "\n".join(lines)
