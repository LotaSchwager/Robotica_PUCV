# Proyecto Final — Línea A: Navegación Autónoma con Planificación de Rutas (A*).
#
# Flujo de misión (USE_PRELOADED_MAP = True, modo por defecto):
#   NAV   → la grilla de ocupación se precarga parseando el archivo .wbt del
#           mundo (world_map.py) y se planifica A* desde el inicio hasta la
#           meta de inmediato.  El robot sigue los waypoints con control
#           proporcional y la capa reactiva lo protege de colisiones.
#   DONE  → robot detenido en la meta.
#
# Flujo alternativo (USE_PRELOADED_MAP = False, mapa construido con sensores):
#   EXPLORE  → el robot navega reactivamente y construye la grilla de ocupación
#              con sus sensores de proximidad (sin conocimiento previo del mapa).
#   RETURN   → A* sobre el mapa construido para volver al punto de inicio.
#   NAV      → A* desde el inicio hasta la meta definida.
#   DONE     → robot detenido en la meta.
#
# El escenario (pose inicial y meta) se selecciona automáticamente según el
# mundo cargado en Webots (robot.getWorldPath() → SCENARIOS).
#
# El filtro de Kalman (Kalman1D) mejora las lecturas de distancia frontal
# que usa la capa de evasión reactiva, reduciendo falsas detecciones de obstáculos
# durante el seguimiento de rutas planificadas.

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
import math
from pathlib import Path
from typing import Any, Optional

from csv_logger import CsvLogger
from estimation import ExponentialMovingAverage, Kalman1D
from occupancy_grid import OccupancyGrid
from path_planner import AStarPlanner
from robot import EpuckRobot
from world_map import build_grid_from_world

# ---------------------------------------------------------------------------
# Parámetros de sensores del e-puck
# Ángulos de ps0..ps7 respecto al eje frontal del robot (rad, + = izquierda).
# ---------------------------------------------------------------------------
PS_ANGLES_RAD: list[float] = [
    math.radians(-22.5),   # ps0: frente derecha
    math.radians(-67.5),   # ps1: derecha
    math.radians(-112.5),  # ps2: trasera derecha
    math.radians(-157.5),  # ps3: trasera
    math.radians(157.5),   # ps4: trasera izquierda
    math.radians(112.5),   # ps5: trasera izquierda
    math.radians(67.5),    # ps6: izquierda
    math.radians(22.5),    # ps7: frente izquierda
]
MAX_SENSOR_RANGE_M: float = 0.20   # Rango máximo de los sensores IR del e-puck

# ---------------------------------------------------------------------------
# Fuente del filtro de distancia frontal para la capa reactiva
# ---------------------------------------------------------------------------
CONTROL_SOURCE = "kalman"   # raw | filtered | kalman

# ---------------------------------------------------------------------------
# Línea de desarrollo
# ---------------------------------------------------------------------------
# True  → Línea A: el mapa se precarga desde el .wbt (entorno conocido) y se
#         planifica A* de inmediato desde el inicio hasta la meta.
# False → el robot construye el mapa con sensores durante EXPLORE_SECONDS
#         antes de planificar (estrategia de mapeo autónomo).
USE_PRELOADED_MAP: bool = True

# True  → usar el giróscopo interno del e-puck para estimar theta (más preciso en giros).
# False → solo encoders (comportamiento original).
# Confirmado por el PROTO: dispositivo "gyro" existe, eje Z (índice 2) = yaw.
USE_GYRO: bool = False

# ---------------------------------------------------------------------------
# Grilla de ocupación
# ---------------------------------------------------------------------------
GRID_CELL_M: float = 0.05
GRID_INFLATION_M: float = 0.06   # > radio del e-puck (0.037 m)

# ---------------------------------------------------------------------------
# Parámetros de exploración (solo con USE_PRELOADED_MAP = False)
# ---------------------------------------------------------------------------
# Tiempo de exploración antes de planificar el retorno al inicio.
EXPLORE_SECONDS: float = 60.0

# ---------------------------------------------------------------------------
# Parámetros de seguimiento de waypoints (control proporcional)
# ---------------------------------------------------------------------------
WAYPOINT_TOLERANCE_M: float = 0.04  # Distancia al waypoint para considerarlo alcanzado
HEADING_KP: float = 2.5             # Ganancia proporcional para el error de orientación
WAYPOINT_KV: float = 2.0            # Ganancia de velocidad lineal en función de distancia

# ---------------------------------------------------------------------------
# Diagnóstico: print periódico de la grilla con posición actual del robot
# ---------------------------------------------------------------------------
PRINT_GRID_EVERY_S: float = 5.0   # cada cuántos segundos imprimir la grilla

# ---------------------------------------------------------------------------
# Umbral de llegada a la meta final
# ---------------------------------------------------------------------------
GOAL_TOLERANCE_M: float = 0.05   # Robot debe estar a ≤ 5cm del marcador

# ---------------------------------------------------------------------------
# Parámetros de la capa reactiva
# ---------------------------------------------------------------------------
PRECAUTION_DISTANCE_M = 0.20  # Reducir velocidad al aproximarse a obstáculo
SAFE_DISTANCE_M = 0.15        # Umbral de giro de emergencia en modo reactivo (PHASE_EXPLORE)

# Detector de atasco por sensores
# Si el robot lleva este nº de pasos con pared frontal o lateral mientras sigue waypoints,
# se considera atascado (wheel-slip contra pared) → reset odometría + replanificar.
STUCK_FRONT_STEPS: int = 20   # ~0.32 s a 62 Hz

# Prevención de giro prematuro: antes de un giro >= TURN_VERIFY_DEG, el robot
# debe haber recorrido el segmento completo MÁS una celda extra (GRID_CELL_M).
# El margen de 1 celda compensa la deriva acumulada de odometría; sin él el robot
# giraba sistemáticamente 1 celda antes de llegar al punto de giro correcto.
TURN_VERIFY_DEG:  float = 75.0   # heading change mínimo para activar giro en el lugar
POINT_TURN_DONE_DEG: float = 8.0 # error de heading al que se considera alineado y se avanza

# Umbrales de parada de emergencia (lecturas RAW, sin filtro Kalman).
# Los cálculos asumen ángulos de sensor del e-puck:
#   ps0/ps7 a ±22.5°: leen D_pared / cos(22.5°) ≈ 1.08·D para pared frontal
#   ps1/ps6 a ±67.5°: leen D_pared / sin(67.5°) ≈ 1.08·D para pared lateral
# La ruta A* (inflación=6cm) mantiene el robot a ≥6.5cm de paredes → ps1 lee ~7cm.
# Con STOP_SIDE_M=6cm: no trigger en ruta planificada; sí si robot deriva >1cm.
STOP_FRONT_M = 0.10  # ps0/ps7 raw: detecta pared frontal a ~9.3cm real
STOP_SIDE_M  = 0.06  # ps1/ps6 raw: detecta pared lateral a ~5.5cm real (sin falso positivo en ruta)
SIDE_DECISION_DEADBAND_M = 0.01
FRONT_TIEBREAK_DEADBAND_M = 0.005

FORWARD_SPEED_FACTOR = 0.60   # velocidad lineal (fracción de MAX_SPEED ~6.28 rad/s)
TURN_SPEED_FACTOR = 0.50
TURN_MIN_SPEED_FACTOR = 0.08

BACKUP_HOLD_STEPS = 17
BACKUP_SPEED_FACTOR = 0.45

# ---------------------------------------------------------------------------
# Filtros de estimación
# ---------------------------------------------------------------------------
EMA_ALPHA = 0.25
KALMAN_P0 = 0.05
KALMAN_Q = 1e-4
KALMAN_R = 5e-3

# ---------------------------------------------------------------------------
# Parámetros geométricos del e-puck
# ---------------------------------------------------------------------------
WHEEL_RADIUS_M = 0.0205
AXLE_LENGTH_M = 0.0573  # valor empírico para la simulación; 0.052 m es el físico pero causa
                        # oscilación del loop de heading cuando se usa sin giróscopo

# ---------------------------------------------------------------------------
# Giro reactivo de 90° (control por posición de encoders)
# ---------------------------------------------------------------------------
TURN_ANGLE_DEG = 90.0
TURN_ANGLE_RAD = math.radians(TURN_ANGLE_DEG)
TURN_WHEEL_TARGET_RAD = (AXLE_LENGTH_M / 2.0) * TURN_ANGLE_RAD / WHEEL_RADIUS_M
TURN_WHEEL_COMMAND_RAD = TURN_WHEEL_TARGET_RAD
TURN_POSITION_TOLERANCE_RAD = 0.005

# ---------------------------------------------------------------------------
# Escenarios: pose inicial (x, y, theta) y meta (x, y) por mundo de Webots.
# La clave es el nombre del archivo de mundo (robot.getWorldPath()).
# Las poses corresponden a las marcas de inicio (roja) y meta (verde) de
# cada arena.
# ---------------------------------------------------------------------------
SCENARIOS: dict[str, dict[str, tuple[float, ...]]] = {
    # Centros de celda exactos (cell_size=0.05m, arena 1×1m centrada en origen):
    # start (-0.35,0.35) → col=3,row=17 → centro=(-0.325, 0.375)
    # goal  (0.35,-0.35) → col=17,row=3 → centro=(0.375, -0.325)
    "lab2_simple.wbt": {
        "start": (-0.325, 0.375, -math.pi / 2.0),
        "goal": (0.375, -0.325),
    },
    # Centros de celda exactos (cell_size=0.05m, arena 3×3m centrada en origen):
    # start (-1.375,-1.375) → col=2,row=2   → centro=(-1.375,-1.375) ✓
    # goal  (1.375,1.375)   → col=57,row=57 → centro=(1.375,1.375)   ✓
    "escenario_complejo.wbt": {
        "start": (-1.375, -1.375, math.pi / 2.0),
        "goal": (1.375, 1.375),
    },
}
DEFAULT_SCENARIO: dict[str, tuple[float, ...]] = {
    "start": (0.0, 0.0, 0.0),
    "goal": (0.5, 0.5),
}

# ---------------------------------------------------------------------------
# Fases de misión
# ---------------------------------------------------------------------------
PHASE_EXPLORE = "EXPLORE"
PHASE_RETURN  = "RETURN_TO_START"
PHASE_NAV     = "NAV_TO_GOAL"
PHASE_DONE    = "GOAL_REACHED"

# ---------------------------------------------------------------------------
# Estados de navegación reactiva (sub-estados durante cualquier fase)
# ---------------------------------------------------------------------------
STATE_FORWARD        = "FORWARD"
STATE_BACKUP         = "BACKUP"
STATE_TURN           = "TURN"
STATE_WAYPOINT_FOLLOW = "WAYPOINT_FOLLOW"
STATE_GOAL_REACHED   = "GOAL_REACHED"


# ---------------------------------------------------------------------------
# Estado global de navegación
# ---------------------------------------------------------------------------
@dataclass
class NavState:
    # Fase de misión
    phase: str = PHASE_EXPLORE

    # Estado de la capa reactiva / waypoint follower
    state: str = STATE_FORWARD

    # Evasión reactiva
    turn_dir: str = ""
    last_turn_dir: str = ""
    turn_start_left_rad: float = 0.0
    turn_start_right_rad: float = 0.0
    turn_target_left_rad: float = 0.0
    turn_target_right_rad: float = 0.0
    turn_travelled_wheel_rad: float = 0.0
    backup_remaining: int = 0

    # Seguimiento de waypoints
    waypoints: list = field(default_factory=list)
    waypoint_idx: int = 0

    # Prevención de giro prematuro: distancia recorrida desde el último waypoint consumido
    # y posición de ese waypoint (origen del segmento actual).
    dist_since_last_wp: float = 0.0
    prev_wp_x: float = 0.0
    prev_wp_y: float = 0.0
    point_turn_active: bool = False  # True mientras el robot gira en el lugar antes de avanzar

    # Replanificación tras evasión de obstáculo dinámico
    replan_pending: bool = False

    # Cooldown tras insertar waypoint de recuperación (evita insertar múltiples seguidos)
    evasion_cooldown: int = 0

    # Métricas
    dist_to_goal: float = math.inf


# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
def _logs_dir() -> Path:
    return Path(__file__).resolve().parents[2] / "logs"


def _log_path(mode: str) -> Path:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return _logs_dir() / f"final_{mode}_{ts}.csv"


# ---------------------------------------------------------------------------
# Selección de fuente de distancia frontal
# ---------------------------------------------------------------------------
def _select_front_used_m(
    source: str, *, front_raw_m: float, front_ema_m: float, front_kalman_m: float
) -> float:
    if source == "raw":
        return float(front_raw_m)
    if source == "filtered":
        return float(front_ema_m)
    return float(front_kalman_m)


# ---------------------------------------------------------------------------
# Capa reactiva: evasión de obstáculos
# ---------------------------------------------------------------------------
def _start_turn_position_control(
    *,
    robot: EpuckRobot,
    nav: NavState,
    enc_left_rad: float,
    enc_right_rad: float,
) -> None:
    nav.turn_start_left_rad = float(enc_left_rad)
    nav.turn_start_right_rad = float(enc_right_rad)

    wheel_rot = float(TURN_WHEEL_COMMAND_RAD)
    if nav.turn_dir == "right":
        nav.turn_target_left_rad  = nav.turn_start_left_rad  + wheel_rot
        nav.turn_target_right_rad = nav.turn_start_right_rad - wheel_rot
    else:
        nav.turn_target_left_rad  = nav.turn_start_left_rad  - wheel_rot
        nav.turn_target_right_rad = nav.turn_start_right_rad + wheel_rot

    max_speed = float(TURN_SPEED_FACTOR) * float(robot.wheels.MAX_SPEED)
    robot.wheels.set_position_targets(
        nav.turn_target_left_rad, nav.turn_target_right_rad, max_speed
    )


def _dist_point_to_segment(
    px: float, py: float,
    ax: float, ay: float,
    bx: float, by: float,
) -> float:
    """Distancia del punto (px,py) al segmento (ax,ay)→(bx,by)."""
    dx, dy = bx - ax, by - ay
    len_sq = dx * dx + dy * dy
    if len_sq < 1e-10:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / len_sq))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _find_recovery_cell(
    grid: OccupancyGrid,
    robot_x: float,
    robot_y: float,
    robot_theta: float,
    next_wp_x: float,
    next_wp_y: float,
    search_radius_m: float = 0.30,
) -> tuple[float, float] | None:
    """
    Cuando el robot está a punto de chocar, busca la celda libre más cercana
    a la ruta planificada (segmento robot→siguiente_waypoint) para esquivar.
    Prioriza celdas laterales o frontales a la trayectoria, no celdas detrás.
    Retorna coordenadas mundo (x, y) o None si no hay opción viable.
    """
    rob_col, rob_row = grid.world_to_cell(robot_x, robot_y)
    radius_cells = int(search_radius_m / grid.cell_size) + 1
    desired_heading = math.atan2(next_wp_y - robot_y, next_wp_x - robot_x)

    best_score = math.inf
    best_wx, best_wy = None, None

    for dr in range(-radius_cells, radius_cells + 1):
        for dc in range(-radius_cells, radius_cells + 1):
            nc, nr = rob_col + dc, rob_row + dr
            if not grid.is_free(nc, nr):
                continue
            wx, wy = grid.cell_to_world(nc, nr)
            dist_robot = math.hypot(wx - robot_x, wy - robot_y)
            # Ignorar demasiado cerca (misma celda) o fuera de radio
            if dist_robot < grid.cell_size or dist_robot > search_radius_m:
                continue

            # Ángulo de esta celda respecto al robot
            cell_angle = math.atan2(wy - robot_y, wx - robot_x)
            angle_diff = abs(math.atan2(
                math.sin(cell_angle - desired_heading),
                math.cos(cell_angle - desired_heading),
            ))
            # Descartar celdas casi opuestas a la dirección deseada (no retroceder)
            if angle_diff > math.radians(150):
                continue

            # Distancia de la celda candidata a la línea robot→waypoint
            path_dist = _dist_point_to_segment(
                wx, wy, robot_x, robot_y, next_wp_x, next_wp_y,
            )

            # Score: priorizar cercanía a la ruta, penalizar levemente distancia al robot
            score = path_dist * 3.0 + dist_robot * 0.5
            if score < best_score:
                best_score = score
                best_wx, best_wy = wx, wy

    return (best_wx, best_wy) if best_wx is not None else None


def _decide_turn_direction(
    *,
    side_left_m: float,
    side_right_m: float,
    side_left_raw: float,
    side_right_raw: float,
    front_left_m: float,
    front_right_m: float,
    last_turn_dir: str,
) -> tuple[str, str, float]:
    delta_side_m = float(side_left_m) - float(side_right_m)
    if not math.isnan(side_left_m) and not math.isnan(side_right_m):
        if abs(delta_side_m) < SIDE_DECISION_DEADBAND_M:
            if (
                not math.isnan(front_left_m)
                and not math.isnan(front_right_m)
                and abs(front_left_m - front_right_m) >= FRONT_TIEBREAK_DEADBAND_M
            ):
                turn_dir = "right" if front_left_m <= front_right_m else "left"
                return turn_dir, "front_tiebreak", delta_side_m
            if last_turn_dir in ("left", "right"):
                return last_turn_dir, "hold", delta_side_m
            return "right", "default", delta_side_m

        turn_dir = "left" if delta_side_m > 0.0 else "right"
        return turn_dir, "meters", delta_side_m

    turn_dir = "left" if side_left_raw <= side_right_raw else "right"
    return turn_dir, "raw", delta_side_m


def _reactive_step(
    *,
    robot: EpuckRobot,
    nav: NavState,
    front_used_m: float,
    front_left_m: float,
    front_right_m: float,
    side_left_m: float,
    side_right_m: float,
    side_left_raw: float,
    side_right_raw: float,
    enc_left_rad: float,
    enc_right_rad: float,
    post_turn_state: str = STATE_FORWARD,
) -> NavState:
    """
    Ejecuta un paso de evasión reactiva.  Cuando la evasión termina (fin del giro),
    transiciona a `post_turn_state` en vez de siempre a STATE_FORWARD, lo que
    permite reanudar el waypoint follower al terminar un giro durante navegación.
    """
    if nav.state == STATE_FORWARD or nav.state == STATE_WAYPOINT_FOLLOW:
        if front_used_m <= SAFE_DISTANCE_M:
            # Emergencia: robot muy cerca de obstáculo. Giro directo (sin backup).
            prev_turn_dir = nav.last_turn_dir
            nav.turn_dir, decision_basis, delta_side_m = _decide_turn_direction(
                side_left_m=side_left_m,
                side_right_m=side_right_m,
                side_left_raw=side_left_raw,
                side_right_raw=side_right_raw,
                front_left_m=front_left_m,
                front_right_m=front_right_m,
                last_turn_dir=nav.last_turn_dir,
            )
            nav.last_turn_dir = nav.turn_dir

            print(
                f"Giro emergencia | front={front_used_m:.3f}m | "
                f"elige={nav.turn_dir} (basis={decision_basis}, prev={prev_turn_dir or '-'})"
            )

            # Giro directo sin backup
            nav.turn_travelled_wheel_rad = 0.0
            nav.state = STATE_TURN
            _start_turn_position_control(
                robot=robot, nav=nav,
                enc_left_rad=enc_left_rad, enc_right_rad=enc_right_rad,
            )

    elif nav.state == STATE_TURN:
        nav.turn_travelled_wheel_rad = (
            abs(float(enc_left_rad)  - nav.turn_start_left_rad)
            + abs(float(enc_right_rad) - nav.turn_start_right_rad)
        ) / 2.0
        left_err  = abs(float(enc_left_rad)  - float(nav.turn_target_left_rad))
        right_err = abs(float(enc_right_rad) - float(nav.turn_target_right_rad))

        if left_err <= TURN_POSITION_TOLERANCE_RAD and right_err <= TURN_POSITION_TOLERANCE_RAD:
            d_left  = float(enc_left_rad)  - nav.turn_start_left_rad
            d_right = float(enc_right_rad) - nav.turn_start_right_rad
            theta_est = WHEEL_RADIUS_M * (d_right - d_left) / AXLE_LENGTH_M
            print(
                f"Fin giro | dir={nav.turn_dir} | "
                f"theta_est={math.degrees(theta_est):+.1f}° | "
                f"errL={left_err:.3f} errR={right_err:.3f}"
            )
            nav.turn_target_left_rad  = 0.0
            nav.turn_target_right_rad = 0.0
            nav.state = post_turn_state
            if nav.state == STATE_FORWARD:
                robot.wheels.forward(FORWARD_SPEED_FACTOR)

    return nav


# ---------------------------------------------------------------------------
# Waypoint follower (control proporcional unicycle → velocidades de rueda)
# ---------------------------------------------------------------------------
def _waypoint_step(
    robot: EpuckRobot,
    nav: NavState,
    x: float,
    y: float,
    theta: float,
    front_dist_m: float = math.inf,
    side_left_m: float = math.inf,
    side_right_m: float = math.inf,
) -> NavState:
    """
    Avanza hacia el waypoint actual.  Cuando se alcanza, avanza al siguiente.
    Usa control proporcional: gira hacia el waypoint y avanza con velocidad
    reducida si el error de orientación es grande o hay obstáculos cercanos.
    """
    if not nav.waypoints or nav.waypoint_idx >= len(nav.waypoints):
        robot.stop()
        return nav

    wp_x, wp_y = nav.waypoints[nav.waypoint_idx]
    dist = math.hypot(wp_x - x, wp_y - y)

    # Avanzar al siguiente waypoint si ya se llegó al actual.
    # El último waypoint usa GOAL_TOLERANCE_M (5cm) en vez de WAYPOINT_TOLERANCE_M (4cm):
    # así el check de meta (dist_to_goal ≤ 5cm) coincide con cuándo se consume el WP final.
    def _wp_tol(idx: int) -> float:
        return GOAL_TOLERANCE_M if idx == len(nav.waypoints) - 1 else WAYPOINT_TOLERANCE_M

    _turn_verify_rad = math.radians(TURN_VERIFY_DEG)
    deferred = False
    while dist <= _wp_tol(nav.waypoint_idx):
        # Prevención de giro prematuro: si el siguiente segmento cambia el heading más
        # de TURN_VERIFY_DEG, exigir haber recorrido el segmento completo + 1 celda extra.
        # El +GRID_CELL_M compensa la deriva acumulada de odometría que hace que el robot
        # "llegue" al waypoint de giro con la posición odométrica, estando aún 1 celda atrás.
        if nav.waypoint_idx + 1 < len(nav.waypoints):
            next_wp_x, next_wp_y = nav.waypoints[nav.waypoint_idx + 1]
            seg_hdg  = math.atan2(wp_y - nav.prev_wp_y, wp_x - nav.prev_wp_x)
            next_hdg = math.atan2(next_wp_y - wp_y, next_wp_x - wp_x)
            hdg_change = abs(math.atan2(
                math.sin(next_hdg - seg_hdg),
                math.cos(next_hdg - seg_hdg),
            ))
            expected_seg = math.hypot(wp_x - nav.prev_wp_x, wp_y - nav.prev_wp_y)
            if (hdg_change > _turn_verify_rad
                    and expected_seg > WAYPOINT_TOLERANCE_M
                    and nav.dist_since_last_wp < expected_seg + GRID_CELL_M * 2.5):
                deferred = True
                break  # aún no se recorrió suficiente del segmento actual → no girar

        nav.prev_wp_x, nav.prev_wp_y = wp_x, wp_y
        nav.dist_since_last_wp = 0.0
        nav.waypoint_idx += 1
        if nav.waypoint_idx >= len(nav.waypoints):
            robot.stop()
            return nav
        wp_x, wp_y = nav.waypoints[nav.waypoint_idx]
        dist = math.hypot(wp_x - x, wp_y - y)

    # Si se difirió el giro, usar la dirección del segmento en curso en vez de apuntar
    # al waypoint actual (que ya está detrás/al lado y daría un heading poco fiable).
    if deferred:
        desired_heading = math.atan2(wp_y - nav.prev_wp_y, wp_x - nav.prev_wp_x)
    else:
        desired_heading = math.atan2(wp_y - y, wp_x - x)
    heading_error = math.atan2(
        math.sin(desired_heading - theta),
        math.cos(desired_heading - theta),
    )

    # Giro en el lugar (point turn):
    # Se activa cuando el heading error supera TURN_VERIFY_DEG.
    # El robot se queda quieto y gira sobre su eje hasta quedar a < POINT_TURN_DONE_DEG
    # del objetivo, evitando que avance hacia paredes durante un giro brusco.
    _pt_activate = math.radians(TURN_VERIFY_DEG)
    _pt_done     = math.radians(POINT_TURN_DONE_DEG)

    if abs(heading_error) >= _pt_activate:
        nav.point_turn_active = True
    elif abs(heading_error) < _pt_done:
        nav.point_turn_active = False

    if nav.point_turn_active:
        omega = max(-TURN_SPEED_FACTOR, min(TURN_SPEED_FACTOR, HEADING_KP * heading_error))
        robot.wheels.set_velocities(
            -omega * robot.wheels.MAX_SPEED,
             omega * robot.wheels.MAX_SPEED,
        )
        nav.state = STATE_WAYPOINT_FOLLOW
        return nav

    # Control proporcional normal (heading error ya pequeño, robot alineado)
    omega = HEADING_KP * heading_error
    omega = max(-TURN_SPEED_FACTOR, min(TURN_SPEED_FACTOR, omega))

    HEADING_STOP_RAD = math.radians(30)
    if abs(heading_error) >= HEADING_STOP_RAD:
        fwd = 0.0
    else:
        fwd = FORWARD_SPEED_FACTOR * (1.0 - abs(heading_error) / HEADING_STOP_RAD)
        fwd = min(fwd, WAYPOINT_KV * dist)

        if front_dist_m < PRECAUTION_DISTANCE_M:
            front_factor = max(0.3, front_dist_m / PRECAUTION_DISTANCE_M)
            fwd *= front_factor

    left_factor  = fwd - omega
    right_factor = fwd + omega

    max_f = max(abs(left_factor), abs(right_factor))
    if max_f > FORWARD_SPEED_FACTOR:
        scale = FORWARD_SPEED_FACTOR / max_f
        left_factor  *= scale
        right_factor *= scale

    robot.wheels.set_velocities(
        left_factor  * robot.wheels.MAX_SPEED,
        right_factor * robot.wheels.MAX_SPEED,
    )
    nav.state = STATE_WAYPOINT_FOLLOW
    return nav


# ---------------------------------------------------------------------------
# Planificación de ruta y transición de fase
# ---------------------------------------------------------------------------
def _dump_route(
    grid: OccupancyGrid,
    waypoints: list[tuple[float, float]],
    start_xy: tuple[float, float],
    dest_xy: tuple[float, float],
    label: str,
) -> None:
    """
    Guarda la ruta planificada como CSV (final_route_*.csv) y el mapa con la
    ruta superpuesta como ASCII (final_map_*.txt) en logs/, para el análisis
    de ruta planificada vs trayectoria ejecutada.
    """
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    logs = _logs_dir()
    try:
        logs.mkdir(parents=True, exist_ok=True)

        route_path = logs / f"final_route_{label}_{ts}.csv"
        with open(route_path, "w", encoding="utf-8") as f:
            f.write("idx,x_m,y_m\n")
            for i, (wx, wy) in enumerate(waypoints):
                f.write(f"{i},{wx:.4f},{wy:.4f}\n")

        # Celdas de los tramos entre waypoints para dibujar la ruta en ASCII
        cells: list[tuple[int, int]] = []
        prev: tuple[int, int] | None = None
        for wx, wy in waypoints:
            cell = grid.world_to_cell(wx, wy)
            if prev is not None:
                cells.extend(grid._bresenham(prev[0], prev[1], cell[0], cell[1]))
            prev = cell
        if prev is not None:
            cells.append(prev)

        map_path = logs / f"final_map_{label}_{ts}.txt"
        map_path.write_text(grid.to_ascii(
            start=grid.world_to_cell(*start_xy),
            goal=grid.world_to_cell(*dest_xy),
            path=cells,
        ), encoding="utf-8")
        print(f"Ruta guardada: {route_path.name} | Mapa: {map_path.name}")
    except OSError as e:
        print(f"WARNING: no se pudo guardar la ruta ({e}).")


def _plan_route(
    planner: AStarPlanner,
    nav: NavState,
    x: float,
    y: float,
    dest_x: float,
    dest_y: float,
    next_phase: str,
    label: str,
) -> NavState:
    """
    Planifica A* desde (x,y) hasta (dest_x,dest_y).
    Si hay ruta, actualiza nav.phase, nav.waypoints y nav.waypoint_idx.
    """
    waypoints = planner.plan(x, y, dest_x, dest_y)
    if waypoints:
        nav.phase = next_phase
        nav.state = STATE_WAYPOINT_FOLLOW
        nav.waypoints = waypoints
        nav.waypoint_idx = 0
        nav.prev_wp_x = x
        nav.prev_wp_y = y
        nav.dist_since_last_wp = 0.0
        nav.point_turn_active = False
        print(
            f"Ruta planificada → {label} | "
            f"{len(waypoints)} waypoints | "
            f"inicio=({x:.2f},{y:.2f}) dest=({dest_x:.2f},{dest_y:.2f})"
        )
        _dump_route(planner.grid, waypoints, (x, y), (dest_x, dest_y), label)
    else:
        print(f"WARN: A* no encontró ruta → {label}. Se continúa en fase actual.")
    return nav


# ---------------------------------------------------------------------------
# Construcción de la fila de log
# ---------------------------------------------------------------------------
def _build_log_row(
    *,
    k: int,
    time_s: float,
    Ts: float,
    fs: float,
    nav: NavState,
    x: float,
    y: float,
    theta: float,
    enc_left_rad: float,
    enc_right_rad: float,
    d_left_rad: float,
    d_right_rad: float,
    delta_s_m: float,
    delta_theta_rad: float,
    cmd_left_vel: float,
    cmd_right_vel: float,
    ps: list[float],
    front_left_m: float,
    front_right_m: float,
    front_raw_m: float,
    front_ema_m: float,
    front_kalman_m: float,
    front_used_m: float,
    side_left_m: float,
    side_right_m: float,
    side_left_raw: float,
    side_right_raw: float,
) -> dict[str, Any]:
    row: dict[str, Any] = {
        "k": int(k),
        "time_s": float(time_s),
        "Ts_s": float(Ts),
        "fs_Hz": float(fs),
        "phase": nav.phase,
        "state": nav.state,
        "turn_dir": nav.turn_dir,
        "turn_travelled_wheel_rad": float(nav.turn_travelled_wheel_rad),
        "turn_target_wheel_rad": float(TURN_WHEEL_TARGET_RAD),
        "backup_remaining": int(nav.backup_remaining),
        "waypoint_idx": int(nav.waypoint_idx),
        "waypoints_total": int(len(nav.waypoints)),
        # Odometría
        "x_m": float(x),
        "y_m": float(y),
        "theta_rad": float(theta),
        "dist_to_goal_m": float(nav.dist_to_goal),
        # Encoders
        "enc_left_rad": float(enc_left_rad),
        "enc_right_rad": float(enc_right_rad),
        "d_left_rad": float(d_left_rad),
        "d_right_rad": float(d_right_rad),
        "delta_s_m": float(delta_s_m),
        "delta_theta_rad": float(delta_theta_rad),
        "cmd_left_vel": float(cmd_left_vel),
        "cmd_right_vel": float(cmd_right_vel),
    }
    row.update({f"ps{i}": float(ps[i]) for i in range(8)})
    row.update({
        "front_left_m":   float(front_left_m),
        "front_right_m":  float(front_right_m),
        "front_raw_m":    float(front_raw_m),
        "front_ema_m":    float(front_ema_m),
        "front_kalman_m": float(front_kalman_m),
        "front_used_m":   float(front_used_m),
        "side_left_m":    float(side_left_m),
        "side_right_m":   float(side_right_m),
        "side_left_raw":  float(side_left_raw),
        "side_right_raw": float(side_right_raw),
    })
    return row


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    robot = EpuckRobot(
        wheel_radius_m=WHEEL_RADIUS_M,
        axle_length_m=AXLE_LENGTH_M,
        use_gyro=USE_GYRO,
    )

    # ------------------------------------------------------------------
    # Selección de escenario según el mundo cargado en Webots
    # ------------------------------------------------------------------
    world_path = Path(robot.robot.getWorldPath())
    scenario = SCENARIOS.get(world_path.name)
    if scenario is None:
        print(f"WARN: mundo '{world_path.name}' sin escenario definido; usando valores por defecto.")
        scenario = DEFAULT_SCENARIO

    start_x, start_y, start_theta = scenario["start"]
    goal_x, goal_y = scenario["goal"]
    robot.set_pose(start_x, start_y, start_theta)

    Ts = robot.timestep / 1000.0
    fs = (1.0 / Ts) if Ts > 0 else 0.0

    # ------------------------------------------------------------------
    # Grilla de ocupación
    # ------------------------------------------------------------------
    if USE_PRELOADED_MAP:
        # Línea A: mapa conocido a priori, parseado del archivo .wbt del mundo.
        grid, world_model = build_grid_from_world(
            world_path, cell_size_m=GRID_CELL_M, inflation_m=GRID_INFLATION_M
        )
        print(
            f"Mapa precargado desde {world_path.name}: "
            f"arena {world_model.arena_width:.1f}x{world_model.arena_height:.1f} m | "
            f"{len(world_model.obstacles)} obstáculos | "
            f"grilla {grid.cols}x{grid.rows} celdas"
        )
    else:
        # Grilla en blanco (3×3m centrada en el origen), construida en tiempo
        # real durante la exploración.
        grid = OccupancyGrid.empty(
            width_m=3.0, height_m=3.0,
            cell_size_m=GRID_CELL_M, inflation_m=GRID_INFLATION_M,
        )
    planner = AStarPlanner(grid)

    ema    = ExponentialMovingAverage(alpha=EMA_ALPHA)
    kalman: Optional[Kalman1D] = None
    nav    = NavState()

    # Con mapa precargado se planifica de inmediato: inicio → meta.
    if USE_PRELOADED_MAP:
        nav = _plan_route(
            planner, nav, start_x, start_y, goal_x, goal_y, PHASE_NAV, "meta"
        )
        if not nav.waypoints:
            print("ERROR: no existe ruta inicio → meta en el mapa precargado.")

    # Imprimir grilla ASCII con inicio (S), meta (G) y ruta planificada (*).
    # Nota: la resolución es 5 cm/celda (necesaria para corredores de 25 cm con
    # inflación de 6 cm). Una cuadrícula visual de 24×24 usaría 12.5 cm/celda,
    # dejando solo ~1 celda libre en corredores de 25 cm: insuficiente para A*.
    s_cell = grid.world_to_cell(start_x, start_y)
    g_cell = grid.world_to_cell(goal_x, goal_y)
    route_cells: list[tuple[int, int]] = []
    if nav.waypoints:
        prev_c: tuple[int, int] | None = None
        for wx, wy in nav.waypoints:
            c = grid.world_to_cell(wx, wy)
            if prev_c is not None:
                route_cells.extend(grid._bresenham(prev_c[0], prev_c[1], c[0], c[1]))
            prev_c = c
        if prev_c is not None:
            route_cells.append(prev_c)
    print(f"\n=== Grilla de ocupación ({grid.cols}×{grid.rows} celdas, {GRID_CELL_M*100:.0f}cm/celda) ===")
    print(grid.to_ascii(start=s_cell, goal=g_cell, path=route_cells))
    print(f"S=inicio ({start_x:.3f},{start_y:.3f})  G=meta ({goal_x:.3f},{goal_y:.3f})")
    print(f"Waypoints: {len(nav.waypoints)}\n")

    log_file = _log_path(CONTROL_SOURCE)

    fieldnames = [
        "k", "time_s", "Ts_s", "fs_Hz",
        "phase", "state", "turn_dir",
        "turn_travelled_wheel_rad", "turn_target_wheel_rad", "backup_remaining",
        "waypoint_idx", "waypoints_total",
        # Odometría
        "x_m", "y_m", "theta_rad", "dist_to_goal_m",
        # Encoders
        "enc_left_rad", "enc_right_rad", "d_left_rad", "d_right_rad",
        "delta_s_m", "delta_theta_rad", "cmd_left_vel", "cmd_right_vel",
        # Proximidad cruda
        "ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7",
        # Distancias filtradas
        "front_left_m", "front_right_m",
        "front_raw_m", "front_ema_m", "front_kalman_m", "front_used_m",
        "side_left_m", "side_right_m", "side_left_raw", "side_right_raw",
    ]

    fase_inicial = (
        nav.phase if USE_PRELOADED_MAP
        else f"{PHASE_EXPLORE} ({EXPLORE_SECONDS:.0f}s)"
    )
    print(
        "=== Proyecto Final (Línea A): Navegación Autónoma con A* ===\n"
        f"Mundo: {world_path.name} | Fase inicial: {fase_inicial} | "
        f"CONTROL_SOURCE={CONTROL_SOURCE} | Ts={Ts:.3f}s\n"
        f"Pose inicial: ({start_x:.2f}, {start_y:.2f}, "
        f"{math.degrees(start_theta):.1f}°) | "
        f"Meta: ({goal_x:.2f}, {goal_y:.2f})"
    )
    print(f"Log: {log_file}")

    logger = None
    try:
        logger = CsvLogger(log_file, fieldnames=fieldnames, flush_every=50)
        logger.write_metadata({
            "lab": "final",
            "world": world_path.name,
            "use_preloaded_map": USE_PRELOADED_MAP,
            "control_source": CONTROL_SOURCE,
            "grid_cell_m": GRID_CELL_M, "grid_inflation_m": GRID_INFLATION_M,
            "explore_seconds": EXPLORE_SECONDS,
            "waypoint_tolerance_m": WAYPOINT_TOLERANCE_M,
            "heading_kp": HEADING_KP,
            "Ts_s": Ts, "fs_Hz": fs,
            "safe_distance_m": SAFE_DISTANCE_M,
            "ema_alpha": EMA_ALPHA,
            "kalman_P0": KALMAN_P0, "kalman_Q": KALMAN_Q, "kalman_R": KALMAN_R,
            "wheel_radius_m": WHEEL_RADIUS_M, "axle_length_m": AXLE_LENGTH_M,
            "robot_x0_m": start_x, "robot_y0_m": start_y,
            "robot_theta0_rad": start_theta,
            "goal_x_m": goal_x, "goal_y_m": goal_y,
            "goal_tolerance_m": GOAL_TOLERANCE_M,
        })
    except Exception as e:
        print(f"WARNING: no se pudo crear el log ({type(e).__name__}: {e}).")

    # Última posición odométrica que estaba dentro de una celda libre.
    last_free_x: float = start_x
    last_free_y: float = start_y

    # Contador de pasos consecutivos con pared detectada mientras el robot avanza.
    stuck_steps_count: int = 0

    try:
        k = 0
        while robot.step():
            time_s = k * Ts

            # ----------------------------------------------------------
            # 1. Lecturas de sensores
            # ----------------------------------------------------------
            ps = robot.proximity.get_values()
            distances_m = robot.proximity.compute_distances_m(ps)

            front_left_m  = float(distances_m[7])
            front_right_m = float(distances_m[0])
            front_raw_m   = float(robot.proximity.front_distance_m(distances_m))
            side_left_m, side_right_m     = robot.proximity.side_distances_m(distances_m)
            side_left_raw, side_right_raw = robot.proximity.side_proximity_values(ps)

            # Filtros de distancia frontal
            front_ema_m = float(ema.update(front_raw_m))

            # ----------------------------------------------------------
            # 2. Odometría: encoders → pose acumulada
            # ----------------------------------------------------------
            (
                enc_left_rad, enc_right_rad,
                d_left_rad, d_right_rad,
                delta_s_m, delta_theta_rad,
            ) = robot.encoder_increment()

            x, y, theta = robot.get_position()

            # Kalman 1D para distancia frontal
            if kalman is None:
                kalman = Kalman1D(x=front_raw_m, P=KALMAN_P0, Q=KALMAN_Q, R=KALMAN_R)
            front_kalman_m = float(kalman.step(u=-delta_s_m, z=front_raw_m))

            front_used_m = _select_front_used_m(
                CONTROL_SOURCE,
                front_raw_m=front_raw_m,
                front_ema_m=front_ema_m,
                front_kalman_m=front_kalman_m,
            )

            # ----------------------------------------------------------
            # 3. Actualización del mapa con sensores (solo en exploración:
            #    el ray-casting marcaría como libres celdas del mapa conocido)
            # ----------------------------------------------------------
            if not USE_PRELOADED_MAP:
                grid.update_from_all_sensors(
                    x, y, theta, distances_m, PS_ANGLES_RAD, MAX_SENSOR_RANGE_M
                )

            # ----------------------------------------------------------
            # 4. Distancia a la meta final
            # ----------------------------------------------------------
            nav.dist_to_goal = math.hypot(goal_x - x, goal_y - y)

            # ----------------------------------------------------------
            # 5. Máquina de estados de misión
            # ----------------------------------------------------------
            r_col, r_row = grid.world_to_cell(x, y)
            g_col, g_row = grid.world_to_cell(goal_x, goal_y)

            # Corrección de deriva odométrica: si la pose estimada cayó dentro de
            # un obstáculo, las ruedas resbalaron sin avanzar (wheel-slip).
            # Se resetea a la última posición libre y se replantifica.
            if grid.is_free(r_col, r_row):
                last_free_x, last_free_y = x, y
            elif nav.phase in (PHASE_NAV, PHASE_RETURN):
                robot.stop()
                drift_dest_x = start_x if nav.phase == PHASE_RETURN else goal_x
                drift_dest_y = start_y if nav.phase == PHASE_RETURN else goal_y
                print(
                    f"DERIVA: celda ({r_col},{r_row}) OCUPADA | "
                    f"pose estimada=({x:.3f},{y:.3f}) | "
                    f"reset a última libre=({last_free_x:.3f},{last_free_y:.3f})"
                )
                robot.set_pose(last_free_x, last_free_y, theta)
                x, y = last_free_x, last_free_y
                r_col, r_row = grid.world_to_cell(x, y)
                nav = _plan_route(
                    planner, nav, x, y, drift_dest_x, drift_dest_y,
                    nav.phase, "replan-deriva"
                )

            # ----------------------------------------------------------
            # 5b. Acumulación de distancia por segmento + detector de atasco
            # ----------------------------------------------------------
            if nav.state == STATE_WAYPOINT_FOLLOW:
                nav.dist_since_last_wp += abs(float(delta_s_m))

            if nav.phase in (PHASE_NAV, PHASE_RETURN) and nav.state == STATE_WAYPOINT_FOLLOW:
                # Sensor frontal y diagonal raw (ya disponibles en distances_m de sección 1)
                _sf = min(float(distances_m[0]), float(distances_m[7]))
                _d1, _d6 = float(distances_m[1]), float(distances_m[6])
                _sd = min(
                    (d for d in (_d1, _d6) if not math.isnan(d) and not math.isinf(d)),
                    default=math.inf,
                )
                if _sf < STOP_FRONT_M or _sd < STOP_SIDE_M:
                    stuck_steps_count += 1
                else:
                    stuck_steps_count = 0

                if stuck_steps_count >= STUCK_FRONT_STEPS:
                    robot.stop()
                    stuck_steps_count = 0
                    nav.dist_since_last_wp = 0.0
                    _stuck_trig  = "frontal" if _sf < STOP_FRONT_M else "lateral"
                    _stuck_destx = start_x if nav.phase == PHASE_RETURN else goal_x
                    _stuck_desty = start_y if nav.phase == PHASE_RETURN else goal_y
                    print(
                        f"STUCK [{_stuck_trig}] t={time_s:.1f}s | "
                        f"sensor_front={_sf:.3f}m sensor_diag={_sd:.3f}m | "
                        f"reset ({x:.3f},{y:.3f})→({last_free_x:.3f},{last_free_y:.3f})"
                    )
                    robot.set_pose(last_free_x, last_free_y, theta)
                    x, y = last_free_x, last_free_y
                    r_col, r_row = grid.world_to_cell(x, y)
                    nav = _plan_route(
                        planner, nav, x, y, _stuck_destx, _stuck_desty,
                        nav.phase, "replan-stuck"
                    )
            else:
                stuck_steps_count = 0

            if nav.phase == PHASE_EXPLORE:
                # Navegación reactiva para explorar y construir el mapa
                nav = _reactive_step(
                    robot=robot, nav=nav,
                    front_used_m=front_used_m,
                    front_left_m=front_left_m, front_right_m=front_right_m,
                    side_left_m=side_left_m, side_right_m=side_right_m,
                    side_left_raw=side_left_raw, side_right_raw=side_right_raw,
                    enc_left_rad=enc_left_rad, enc_right_rad=enc_right_rad,
                    post_turn_state=STATE_FORWARD,
                )

                # Fin de exploración: planificar retorno al inicio
                if time_s >= EXPLORE_SECONDS and nav.state == STATE_FORWARD:
                    print(
                        f"\n>>> Exploración completa ({time_s:.1f}s). "
                        f"Retornando al inicio ({start_x:.2f},{start_y:.2f})..."
                    )
                    robot.stop()
                    nav = _plan_route(
                        planner, nav, x, y,
                        start_x, start_y,
                        PHASE_RETURN, "inicio",
                    )

            elif nav.phase in (PHASE_RETURN, PHASE_NAV):
                dest_x = start_x if nav.phase == PHASE_RETURN else goal_x
                dest_y = start_y if nav.phase == PHASE_RETURN else goal_y

                # Lecturas RAW de sensores para detección de colisión (sin lag del filtro Kalman).
                # ps0/ps7 → frontal ±22.5°. ps1/ps6 → diagonal ±67.5° (detectan paredes laterales).
                raw_front_m = min(float(distances_m[0]), float(distances_m[7]))
                d1_raw = float(distances_m[1])
                d6_raw = float(distances_m[6])
                raw_diag_m = min(
                    (d for d in (d1_raw, d6_raw) if not math.isnan(d) and not math.isinf(d)),
                    default=math.inf,
                )
                # diag_min_m limpio para controlar velocidad en _waypoint_step
                diag_min_m = raw_diag_m

                # Condición de parada: pared frontal < 10cm OR pared lateral < 6cm (ambas RAW).
                # Estos umbrales permiten navegar sin false-positive junto a paredes del plan A*
                # (que quedan a ≥6.5cm), pero detienen al robot si se desvía.
                stop_now = (
                    (raw_front_m < STOP_FRONT_M)
                    or (raw_diag_m < STOP_SIDE_M)
                )

                if stop_now:
                    if nav.evasion_cooldown <= 0:
                        # Primera detección: parar, resetear odometría a última posición
                        # libre (evita replanificar desde una posición dentro de un obstáculo
                        # por wheel-slip) y replanificar A*.
                        robot.stop()
                        trigger = "front" if raw_front_m < STOP_FRONT_M else "lateral"
                        print(
                            f"PARADA [{trigger}] | front_raw={raw_front_m:.3f}m "
                            f"diag_raw={raw_diag_m:.3f}m | "
                            f"pose=({x:.2f},{y:.2f},{math.degrees(theta):.0f}°) → "
                            f"reset a ({last_free_x:.3f},{last_free_y:.3f})"
                        )
                        nav.evasion_cooldown = 30
                        nav.dist_since_last_wp = 0.0
                        robot.set_pose(last_free_x, last_free_y, theta)
                        x, y = last_free_x, last_free_y
                        nav = _plan_route(
                            planner, nav, x, y, dest_x, dest_y,
                            nav.phase, "replan-parada"
                        )
                    else:
                        # En cooldown post-replan: seguir el nuevo plan (el waypoint debería
                        # alejar al robot de la pared). Solo parar si realmente toca (<3cm).
                        nav.evasion_cooldown -= 1
                        if raw_front_m < 0.03 or raw_diag_m < 0.03:
                            robot.stop()  # contacto físico inminente
                        else:
                            nav = _waypoint_step(
                                robot, nav, x, y, theta,
                                front_dist_m=min(front_used_m, diag_min_m),
                                side_left_m=side_left_m,
                                side_right_m=side_right_m,
                            )
                else:
                    # Sin obstáculo: seguir plan A*.
                    if nav.evasion_cooldown > 0:
                        nav.evasion_cooldown -= 1
                    nav = _waypoint_step(
                        robot, nav, x, y, theta,
                        front_dist_m=min(front_used_m, diag_min_m),
                        side_left_m=side_left_m,
                        side_right_m=side_right_m,
                    )

                # Verificar si se completó la ruta de esta fase
                if nav.waypoint_idx >= len(nav.waypoints) and nav.waypoints:
                    dist_dest = math.hypot(dest_x - x, dest_y - y)
                    if nav.phase == PHASE_RETURN:
                        print(
                            f"\n>>> Retorno completado (dist_inicio={dist_dest:.3f}m). "
                            f"Planificando ruta a la meta ({goal_x:.2f},{goal_y:.2f})..."
                        )
                        robot.stop()
                        nav = _plan_route(
                            planner, nav, x, y, goal_x, goal_y, PHASE_NAV, "meta",
                        )
                    elif nav.phase == PHASE_NAV:
                        # Declarar meta solo con distancia euclidiana estricta
                        if nav.dist_to_goal <= GOAL_TOLERANCE_M:
                            nav.phase = PHASE_DONE
                            nav.state = STATE_GOAL_REACHED
                            robot.stop()
                            print(
                                f"\n>>> META ALCANZADA | "
                                f"pose=({x:.3f},{y:.3f},{math.degrees(theta):.1f}°) | "
                                f"dist={nav.dist_to_goal:.4f}m | t={time_s:.1f}s"
                            )
                        else:
                            # Waypoints consumidos pero el robot no está en la celda meta:
                            # replanificar A* desde la posición actual.
                            print(
                                f"WARN: Waypoints consumidos pero lejos de meta "
                                f"(dist={nav.dist_to_goal:.2f}m, celda=({r_col},{r_row}) "
                                f"vs meta=({g_col},{g_row})). Replanificando..."
                            )
                            nav = _plan_route(
                                planner, nav, x, y, goal_x, goal_y, PHASE_NAV, "replan-goal"
                            )

            elif nav.phase == PHASE_DONE:
                # Meta alcanzada, robot detenido
                break

            # ----------------------------------------------------------
            # 6. Log
            # ----------------------------------------------------------
            cmd_left_vel, cmd_right_vel = robot.wheels.get_last_velocities()

            # ----------------------------------------------------------
            # 7. Print periódico de grilla con posición actual del robot
            # ----------------------------------------------------------
            if PRINT_GRID_EVERY_S > 0 and k > 0:
                steps_per_print = max(1, int(PRINT_GRID_EVERY_S / Ts))
                if k % steps_per_print == 0:
                    s_col, s_row = grid.world_to_cell(start_x, start_y)
                    print(
                        f"\n--- t={time_s:.1f}s | "
                        f"pose=({x:.3f},{y:.3f},{math.degrees(theta):.1f}°) | "
                        f"celda=({r_col},{r_row}) | "
                        f"wp {nav.waypoint_idx}/{len(nav.waypoints)} | "
                        f"dist_meta={nav.dist_to_goal:.3f}m ---"
                    )
                    print(grid.to_ascii(
                        start=(s_col, s_row),
                        goal=(g_col, g_row),
                        path=route_cells,
                        robot=(r_col, r_row),
                    ))
                    print(f"S=inicio  G=meta  *=ruta  O=robot  #=obstáculo\n")

            if logger is not None:
                logger.log(_build_log_row(
                    k=k, time_s=time_s, Ts=Ts, fs=fs, nav=nav,
                    x=x, y=y, theta=theta,
                    enc_left_rad=enc_left_rad, enc_right_rad=enc_right_rad,
                    d_left_rad=d_left_rad, d_right_rad=d_right_rad,
                    delta_s_m=delta_s_m, delta_theta_rad=delta_theta_rad,
                    cmd_left_vel=cmd_left_vel, cmd_right_vel=cmd_right_vel,
                    ps=ps,
                    front_left_m=front_left_m, front_right_m=front_right_m,
                    front_raw_m=front_raw_m, front_ema_m=front_ema_m,
                    front_kalman_m=front_kalman_m, front_used_m=front_used_m,
                    side_left_m=side_left_m, side_right_m=side_right_m,
                    side_left_raw=side_left_raw, side_right_raw=side_right_raw,
                ))

            k += 1

    finally:
        if logger is not None:
            logger.close()

    robot.stop()
    print(f"Fin. k={k} | Ts={Ts:.3f}s | fase_final={nav.phase}")


if __name__ == "__main__":
    main()
