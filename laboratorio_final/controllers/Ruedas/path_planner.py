from __future__ import annotations

import heapq
import math
from collections import deque

from occupancy_grid import OccupancyGrid


class AStarPlanner:
    """
    Planificador de rutas A* sobre una OccupancyGrid 2D.

    El algoritmo usa conectividad 8 (ortogonal + diagonal) con costo real
    (1 celda = cell_size_m metros; diagonal = cell_size_m × √2).
    La heurística es distancia euclídea al objetivo (admisible y consistente).

    Devuelve waypoints en coordenadas del mundo (metros), ya simplificados
    para eliminar puntos colineales innecesarios.

    Relación con el filtro de Kalman:
        La posición del robot que se alimenta al planificador proviene de la
        odometría integrada en EpuckRobot.  El filtro Kalman1D (estimation.py)
        mejora las lecturas de distancia frontal que usa la capa reactiva, lo
        que reduce falsas paradas y mejora el seguimiento de la ruta planificada.
    """

    def __init__(self, grid: OccupancyGrid):
        self.grid = grid

    # ------------------------------------------------------------------
    # API pública
    # ------------------------------------------------------------------

    def plan(
        self,
        start_x: float,
        start_y: float,
        goal_x: float,
        goal_y: float,
    ) -> list[tuple[float, float]]:
        """
        Calcula la ruta más corta desde (start_x, start_y) hasta (goal_x, goal_y).

        Retorna lista de coordenadas del mundo [(x, y), ...] incluyendo inicio y
        meta, simplificada para eliminar puntos intermedios colineales.
        Retorna lista vacía si no existe ruta.
        """
        grid = self.grid

        start_col, start_row = grid.world_to_cell(start_x, start_y)
        goal_col, goal_row = grid.world_to_cell(goal_x, goal_y)

        # Si inicio o meta caen dentro de zona inflada, buscar la celda libre más cercana
        if not grid.is_free(start_col, start_row):
            start_col, start_row = self._nearest_free(start_col, start_row)
        if not grid.is_free(goal_col, goal_row):
            goal_col, goal_row = self._nearest_free(goal_col, goal_row)

        if (start_col, start_row) == (goal_col, goal_row):
            return [grid.cell_to_world(goal_col, goal_row)]

        # Cola de prioridad: (f, g, col, row)
        open_heap: list[tuple[float, float, int, int]] = []
        g0 = 0.0
        h0 = self._heuristic(start_col, start_row, goal_col, goal_row)
        heapq.heappush(open_heap, (g0 + h0, g0, start_col, start_row))

        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        g_score: dict[tuple[int, int], float] = {(start_col, start_row): 0.0}

        while open_heap:
            _, g_curr, col, row = heapq.heappop(open_heap)

            if col == goal_col and row == goal_row:
                return self._reconstruct_path(came_from, col, row, start_x, start_y)

            # Entrada obsoleta en el heap
            if g_curr > g_score.get((col, row), math.inf):
                continue

            for nc, nr, move_cost in self._neighbors(col, row):
                if not grid.is_free(nc, nr):
                    continue
                new_g = g_curr + move_cost
                if new_g < g_score.get((nc, nr), math.inf):
                    g_score[(nc, nr)] = new_g
                    came_from[(nc, nr)] = (col, row)
                    h = self._heuristic(nc, nr, goal_col, goal_row)
                    heapq.heappush(open_heap, (new_g + h, new_g, nc, nr))

        print("A*: no se encontró ruta desde el inicio hasta la meta.")
        return []

    # ------------------------------------------------------------------
    # Internos
    # ------------------------------------------------------------------

    def _heuristic(self, col: int, row: int, gc: int, gr: int) -> float:
        return math.hypot(gc - col, gr - row) * self.grid.cell_size

    def _neighbors(
        self, col: int, row: int
    ) -> list[tuple[int, int, float]]:
        """
        8-conectado.  Costo ortogonal = cell_size; diagonal = cell_size × √2.
        Las diagonales se bloquean si alguno de los dos vecinos ortogonales
        adyacentes está ocupado (evita cortar esquinas de paredes).
        """
        cell = self.grid.cell_size
        diag = cell * math.sqrt(2)

        ortho = [(-1, 0, cell), (1, 0, cell), (0, -1, cell), (0, 1, cell)]
        diagonal = [(-1, -1, diag), (-1, 1, diag), (1, -1, diag), (1, 1, diag)]

        result: list[tuple[int, int, float]] = []

        for dc, dr, cost in ortho:
            nc, nr = col + dc, row + dr
            if self.grid.is_valid(nc, nr):
                result.append((nc, nr, cost))

        for dc, dr, cost in diagonal:
            nc, nr = col + dc, row + dr
            if not self.grid.is_valid(nc, nr):
                continue
            # Bloquear diagonal si alguno de los ortogonales adyacentes está ocupado
            if not self.grid.is_free(col + dc, row) or not self.grid.is_free(col, row + dr):
                continue
            result.append((nc, nr, cost))

        return result

    def _reconstruct_path(
        self,
        came_from: dict[tuple[int, int], tuple[int, int]],
        col: int,
        row: int,
        start_x: float,
        start_y: float,
    ) -> list[tuple[float, float]]:
        cells: list[tuple[int, int]] = []
        curr = (col, row)
        while curr in came_from:
            cells.append(curr)
            curr = came_from[curr]
        cells.append(curr)
        cells.reverse()

        waypoints: list[tuple[float, float]] = []
        # El primer waypoint usa la posición real del robot, no el centro de celda
        waypoints.append((float(start_x), float(start_y)))
        for c, r in cells[1:]:
            waypoints.append(self.grid.cell_to_world(c, r))

        return waypoints  # No simplificar: waypoints celda-a-celda evitan deriva lateral en corredores

    def _simplify(
        self, waypoints: list[tuple[float, float]]
    ) -> list[tuple[float, float]]:
        """
        Elimina puntos intermedios colineales.
        Actualmente NO se usa: mantener todos los waypoints (c/5cm) evita que el
        robot apunte a waypoints lejanos en diagonal, reduciendo deriva hacia paredes.
        """
        if len(waypoints) <= 2:
            return waypoints

        result = [waypoints[0]]
        for i in range(1, len(waypoints) - 1):
            ax, ay = waypoints[i - 1]
            bx, by = waypoints[i]
            cx, cy = waypoints[i + 1]
            cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
            if abs(cross) > 1e-9:
                result.append(waypoints[i])

        result.append(waypoints[-1])
        return result

    def _nearest_free(self, col: int, row: int) -> tuple[int, int]:
        """BFS desde (col, row) para encontrar la celda libre más cercana."""
        visited: set[tuple[int, int]] = {(col, row)}
        queue: deque[tuple[int, int]] = deque([(col, row)])
        while queue:
            c, r = queue.popleft()
            if self.grid.is_free(c, r):
                return c, r
            for dc, dr in [(-1, 0), (1, 0), (0, -1), (0, 1),
                           (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                nc, nr = c + dc, r + dr
                if self.grid.is_valid(nc, nr) and (nc, nr) not in visited:
                    visited.add((nc, nr))
                    queue.append((nc, nr))
        return col, row
