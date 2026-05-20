"""Controlador principal (Lab 2).

Laboratorio 2: Navegación reactiva con filtrado y fusión de sensores.

- Registra sensores crudos + encoders.
- Aplica filtro simple (EMA) en distancia frontal.
- Implementa filtro de Kalman (predicción con encoders + corrección con sensores frontales).
- Usa la distancia frontal (raw/filtrada/Kalman) para decidir avanzar o girar.
- Usa sensores laterales para decidir dirección del giro.
"""

from __future__ import annotations

from datetime import datetime
import math
from pathlib import Path
from typing import Optional

from csv_logger import CsvLogger
from estimation import ExponentialMovingAverage, Kalman1D
from robot import EpuckRobot


# =========================
# Configuración del Lab 2
# =========================

# Fuente para la decisión de navegación reactiva:
#   - "raw": medición cruda (sensores frontales convertidos a metros)
#   - "filtered": medición filtrada (EMA)
#   - "kalman": estimación fusionada (Kalman)
CONTROL_SOURCE = "kalman"  # raw | filtered | kalman

# Duración de la corrida. Usa None para correr hasta que detengas la simulación.
RUN_SECONDS: Optional[float] = None

# Umbral de seguridad (m). Si la distancia frontal estimada <= umbral, gira.
SAFE_DISTANCE_M = 0.18

# Velocidades (factores de MAX_SPEED del e-puck en Webots)
FORWARD_SPEED_FACTOR = 0.55
TURN_SPEED_FACTOR = 0.35
TURN_MIN_SPEED_FACTOR = 0.08

# Retroceso breve al detectar obstáculo para evitar quedar "pegado".
BACKUP_HOLD_STEPS = 12
BACKUP_SPEED_FACTOR = 0.45

# Filtro simple (EMA)
EMA_ALPHA = 0.25

# Kalman (1D escalar)
KALMAN_P0 = 0.05
KALMAN_Q = 1e-4
KALMAN_R = 5e-3

# Parámetros geométricos aproximados del e-puck (para encoders -> metros)
WHEEL_RADIUS_M = 0.0205
AXLE_LENGTH_M = 0.052

# Giro fijo de 90° usando encoders (NO por tiempo).
# TURN_WHEEL_TARGET_RAD indica cuántos radianes debe girar cada rueda
# (en giro sobre su propio eje) para rotar el robot 90°.
TURN_ANGLE_DEG = 90.0
TURN_ANGLE_RAD = math.radians(TURN_ANGLE_DEG)
TURN_WHEEL_TARGET_RAD = (AXLE_LENGTH_M / 2.0) * TURN_ANGLE_RAD / WHEEL_RADIUS_M


def _logs_dir() -> Path:
    # controllers/Ruedas/Ruedas.py -> laboratorio_1/
    return Path(__file__).resolve().parents[2] / "logs"


def _log_path(mode: str) -> Path:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return _logs_dir() / f"lab2_{mode}_{ts}.csv"


def main() -> None:
    robot = EpuckRobot(wheel_radius_m=WHEEL_RADIUS_M, axle_length_m=AXLE_LENGTH_M)

    Ts = robot.timestep / 1000.0
    fs = (1.0 / Ts) if Ts > 0 else 0.0

    log_file = _log_path(CONTROL_SOURCE)

    fieldnames = [
        "k",
        "time_s",
        "Ts_s",
        "fs_Hz",
        "state",
        "turn_dir",
        "turn_travelled_wheel_rad",
        "turn_target_wheel_rad",
        "backup_remaining",
        "enc_left_rad",
        "enc_right_rad",
        "d_left_rad",
        "d_right_rad",
        "delta_s_m",
        "delta_theta_rad",
        "cmd_left_vel",
        "cmd_right_vel",
        # Proximidad cruda
        "ps0",
        "ps1",
        "ps2",
        "ps3",
        "ps4",
        "ps5",
        "ps6",
        "ps7",
        # Distancias (metros)
        "front_left_m",
        "front_right_m",
        "front_raw_m",
        "front_ema_m",
        "front_kalman_m",
        "front_used_m",
        "side_left_m",
        "side_right_m",
        "side_left_raw",
        "side_right_raw",
    ]

    ema = ExponentialMovingAverage(alpha=EMA_ALPHA)
    kalman: Optional[Kalman1D] = None

    state = "FORWARD"
    turn_dir = ""
    turn_start_left_rad = 0.0
    turn_start_right_rad = 0.0
    turn_travelled_wheel_rad = 0.0
    backup_remaining = 0

    print(
        "[Lab2] Iniciando navegación reactiva | "
        f"CONTROL_SOURCE={CONTROL_SOURCE} | RUN_SECONDS={RUN_SECONDS} | Ts={Ts:.3f}s (fs={fs:.1f}Hz) | "
        f"SAFE_DISTANCE_M={SAFE_DISTANCE_M:.3f}"
    )
    print(f"[Lab2] Log CSV: {log_file}")

    logger = None
    try:
        logger = CsvLogger(log_file, fieldnames=fieldnames, flush_every=50)
        logger.write_metadata(
            {
                "lab": "2",
                "control_source": CONTROL_SOURCE,
                "run_seconds": RUN_SECONDS,
                "Ts_s": Ts,
                "fs_Hz": fs,
                "safe_distance_m": SAFE_DISTANCE_M,
                "ema_alpha": EMA_ALPHA,
                "kalman_P0": KALMAN_P0,
                "kalman_Q": KALMAN_Q,
                "kalman_R": KALMAN_R,
                "wheel_radius_m": WHEEL_RADIUS_M,
                "axle_length_m": AXLE_LENGTH_M,
                "turn_angle_deg": TURN_ANGLE_DEG,
                "turn_wheel_target_rad": TURN_WHEEL_TARGET_RAD,
            }
        )
    except Exception as e:
        print(f"[Lab2] WARNING: no se pudo crear el log CSV ({type(e).__name__}: {e}).")
        logger = None

    try:
        k = 0
        while robot.step():
            time_s = k * Ts

            # Lecturas crudas
            ps = robot.proximity.get_values()
            distances_m = robot.proximity.compute_distances_m(ps)

            front_left_m = float(distances_m[7])
            front_right_m = float(distances_m[0])
            front_raw_m = float(robot.proximity.front_distance_m(distances_m))
            side_left_m, side_right_m = robot.proximity.side_distances_m(distances_m)
            side_left_raw, side_right_raw = robot.proximity.side_proximity_values(ps)

            # Filtro simple (EMA)
            front_ema_m = float(ema.update(front_raw_m))

            # Encoders -> avance
            (
                enc_left_rad,
                enc_right_rad,
                d_left_rad,
                d_right_rad,
                delta_s_m,
                delta_theta_rad,
            ) = robot.encoder_increment()

            # Kalman
            if kalman is None:
                kalman = Kalman1D(x=front_raw_m, P=KALMAN_P0, Q=KALMAN_Q, R=KALMAN_R)
            # Si avanza delta_s, la distancia frontal debería disminuir delta_s
            front_kalman_m = float(kalman.step(u=-delta_s_m, z=front_raw_m))

            if CONTROL_SOURCE == "raw":
                front_used_m = front_raw_m
            elif CONTROL_SOURCE == "filtered":
                front_used_m = front_ema_m
            else:
                front_used_m = front_kalman_m

            # =========================
            # Lógica reactiva
            # =========================
            if state == "FORWARD":
                if front_used_m <= SAFE_DISTANCE_M:
                    # Elegir dirección con más recorrido (más espacio libre).
                    if not math.isnan(side_left_m) and not math.isnan(side_right_m):
                        turn_dir = "left" if side_left_m >= side_right_m else "right"
                    else:
                        # Fallback: menor proximidad cruda => más espacio
                        turn_dir = "left" if side_left_raw <= side_right_raw else "right"

                    # Nuevo ciclo de evasión
                    backup_remaining = int(BACKUP_HOLD_STEPS)
                    turn_travelled_wheel_rad = 0.0

                    if backup_remaining > 0:
                        state = "BACKUP"
                        robot.wheels.backward(BACKUP_SPEED_FACTOR)
                    else:
                        state = "TURN"
                        turn_start_left_rad = enc_left_rad
                        turn_start_right_rad = enc_right_rad
                        if turn_dir == "right":
                            robot.wheels.turn_own_axis_right(TURN_SPEED_FACTOR)
                        else:
                            robot.wheels.turn_own_axis_left(TURN_SPEED_FACTOR)
                else:
                    turn_travelled_wheel_rad = 0.0
                    turn_dir = ""
                    robot.wheels.forward(FORWARD_SPEED_FACTOR)

            elif state == "BACKUP":
                turn_travelled_wheel_rad = 0.0
                robot.wheels.backward(BACKUP_SPEED_FACTOR)
                backup_remaining -= 1
                if backup_remaining <= 0:
                    state = "TURN"
                    turn_start_left_rad = enc_left_rad
                    turn_start_right_rad = enc_right_rad
                    turn_travelled_wheel_rad = 0.0
                    # Aplicar el giro inmediatamente al transicionar (evita 1 paso extra en reversa).
                    if turn_dir == "right":
                        robot.wheels.turn_own_axis_right(TURN_SPEED_FACTOR)
                    else:
                        robot.wheels.turn_own_axis_left(TURN_SPEED_FACTOR)

            else:  # TURN
                # Giro exacto de 90° usando encoders (rad de rueda)
                turn_travelled_wheel_rad = (
                    abs(enc_left_rad - turn_start_left_rad)
                    + abs(enc_right_rad - turn_start_right_rad)
                ) / 2.0
                remaining_wheel_rad = TURN_WHEEL_TARGET_RAD - turn_travelled_wheel_rad
                if remaining_wheel_rad <= 0.0:
                    state = "FORWARD"
                    robot.wheels.forward(FORWARD_SPEED_FACTOR)
                else:
                    # Disminuye velocidad cerca del objetivo para reducir overshoot.
                    turn_speed_factor = max(
                        TURN_MIN_SPEED_FACTOR,
                        min(TURN_SPEED_FACTOR, 0.5 * remaining_wheel_rad),
                    )
                    if turn_dir == "right":
                        robot.wheels.turn_own_axis_right(turn_speed_factor)
                    else:
                        robot.wheels.turn_own_axis_left(turn_speed_factor)

            cmd_left_vel, cmd_right_vel = robot.wheels.get_last_velocities()

            if logger is not None:
                logger.log(
                    {
                        "k": k,
                        "time_s": time_s,
                        "Ts_s": Ts,
                        "fs_Hz": fs,
                        "state": state,
                        "turn_dir": turn_dir,
                        "turn_travelled_wheel_rad": turn_travelled_wheel_rad,
                        "turn_target_wheel_rad": TURN_WHEEL_TARGET_RAD,
                        "backup_remaining": backup_remaining,
                        "enc_left_rad": enc_left_rad,
                        "enc_right_rad": enc_right_rad,
                        "d_left_rad": d_left_rad,
                        "d_right_rad": d_right_rad,
                        "delta_s_m": delta_s_m,
                        "delta_theta_rad": delta_theta_rad,
                        "cmd_left_vel": cmd_left_vel,
                        "cmd_right_vel": cmd_right_vel,
                        "ps0": ps[0],
                        "ps1": ps[1],
                        "ps2": ps[2],
                        "ps3": ps[3],
                        "ps4": ps[4],
                        "ps5": ps[5],
                        "ps6": ps[6],
                        "ps7": ps[7],
                        "front_left_m": front_left_m,
                        "front_right_m": front_right_m,
                        "front_raw_m": front_raw_m,
                        "front_ema_m": front_ema_m,
                        "front_kalman_m": front_kalman_m,
                        "front_used_m": front_used_m,
                        "side_left_m": side_left_m,
                        "side_right_m": side_right_m,
                        "side_left_raw": side_left_raw,
                        "side_right_raw": side_right_raw,
                    }
                )

            k += 1
            if RUN_SECONDS is not None and time_s >= float(RUN_SECONDS):
                break

    finally:
        if logger is not None:
            logger.close()

    robot.stop()
    print(f"[Lab2] Fin. Muestras: {k} | Ts={Ts:.3f}s (fs={fs:.1f}Hz)")


if __name__ == "__main__":
    main()