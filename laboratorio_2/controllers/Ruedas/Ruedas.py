# Laboratorio 2: Navegación reactiva con filtrado y fusión de sensores.


from __future__ import annotations
from dataclasses import dataclass
from datetime import datetime
import math
from pathlib import Path
from typing import Any, Optional
from csv_logger import CsvLogger
from estimation import ExponentialMovingAverage, Kalman1D
from robot import EpuckRobot

CONTROL_SOURCE = "filtered"  # raw | filtered | kalman

# Duración de la corrida. Usa None para correr hasta que detengas la simulación.
RUN_SECONDS: Optional[float] = None

# Umbral de seguridad (m). Si la distancia frontal estimada <= umbral, gira.
SAFE_DISTANCE_M = 0.17

# Deadband para evitar oscilación izquierda/derecha cuando ambos lados dan casi lo mismo.
# Si |side_left_m - side_right_m| < deadband, se usa desempate (front) o se mantiene el último giro.
SIDE_DECISION_DEADBAND_M = 0.01
FRONT_TIEBREAK_DEADBAND_M = 0.005

# Velocidades (factores de MAX_SPEED del e-puck en Webots)
FORWARD_SPEED_FACTOR = 0.55
TURN_SPEED_FACTOR = 0.35 # hdjsak
TURN_MIN_SPEED_FACTOR = 0.08

# Retroceso breve al detectar obstáculo para evitar quedar "pegado".
BACKUP_HOLD_STEPS = 17
BACKUP_SPEED_FACTOR = 0.45

# Filtro simple (EMA)
EMA_ALPHA = 0.25

# Kalman (1D escalar)
KALMAN_P0 = 0.05
KALMAN_Q = 1e-4
KALMAN_R = 5e-3

# Parámetros geométricos aproximados del e-puck (para encoders -> metros)
WHEEL_RADIUS_M = 0.0205
AXLE_LENGTH_M = 0.0573  # longitud entre ruedas aproximada usada en la geometría del giro

# Giro fijo de 90° con encoders.
# TURN_WHEEL_TARGET_RAD = cuántos radianes debe girar cada rueda

TURN_ANGLE_DEG = 90.0
TURN_ANGLE_RAD = math.radians(TURN_ANGLE_DEG)
TURN_WHEEL_TARGET_RAD = (AXLE_LENGTH_M / 2.0) * TURN_ANGLE_RAD / WHEEL_RADIUS_M

# Compensación para corregir por tolerancia debido a errL/errR de 0.007rad
# Con esto nos acercamos mas a un giro de 90°.
# TURN_WHEEL_EXTRA_RAD = 0.007
# Se solucionó por lo que no fue necesario agregar esta compensación
TURN_WHEEL_COMMAND_RAD = TURN_WHEEL_TARGET_RAD 
#TURN_WHEEL_COMMAND_RAD = TURN_WHEEL_TARGET_RAD + TURN_WHEEL_EXTRA_RAD

# Tolerancia para considerar que el motor llegó a su objetivo en modo posición.
TURN_POSITION_TOLERANCE_RAD = 0.005 # Antes 0.01, ahora 0.005

# Estados de navegación posibles del robot dependiendo de la situación 
STATE_FORWARD = "FORWARD"
STATE_BACKUP = "BACKUP"
STATE_TURN = "TURN"

#Clase del robot
@dataclass
class ReactiveNavState:
    state: str = STATE_FORWARD
    turn_dir: str = ""
    last_turn_dir: str = ""
    turn_start_left_rad: float = 0.0
    turn_start_right_rad: float = 0.0
    turn_target_left_rad: float = 0.0
    turn_target_right_rad: float = 0.0
    turn_travelled_wheel_rad: float = 0.0
    backup_remaining: int = 0


def _logs_dir() -> Path:
    
    return Path(__file__).resolve().parents[2] / "logs"


def _log_path(mode: str) -> Path:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return _logs_dir() / f"lab2_{mode}_{ts}.csv"


def _select_front_used_m(
    source: str, *, front_raw_m: float, front_ema_m: float, front_kalman_m: float
) -> float:
    if source == "raw":
        return float(front_raw_m)
    if source == "filtered":
        return float(front_ema_m)
    return float(front_kalman_m)


def _turn_speed_factor(remaining_wheel_rad: float) -> float:
    return max(
        TURN_MIN_SPEED_FACTOR,
        min(TURN_SPEED_FACTOR, 0.5 * float(remaining_wheel_rad)),
    )


def _start_turn_position_control(
    *,
    robot: EpuckRobot,
    nav: ReactiveNavState,
    enc_left_rad: float,
    enc_right_rad: float,
) -> None:
    nav.turn_start_left_rad = float(enc_left_rad)
    nav.turn_start_right_rad = float(enc_right_rad)

    wheel_rot = float(TURN_WHEEL_COMMAND_RAD)
    if nav.turn_dir == "right":
        nav.turn_target_left_rad = nav.turn_start_left_rad + wheel_rot
        nav.turn_target_right_rad = nav.turn_start_right_rad - wheel_rot
    else:
        nav.turn_target_left_rad = nav.turn_start_left_rad - wheel_rot
        nav.turn_target_right_rad = nav.turn_start_right_rad + wheel_rot

    max_speed_rad_s = float(TURN_SPEED_FACTOR) * float(robot.wheels.MAX_SPEED)
    robot.wheels.set_position_targets(
        nav.turn_target_left_rad,
        nav.turn_target_right_rad,
        max_speed_rad_s,
    )


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
    # es posible que esta decisión de diseño del algoritmo proboque errores al 
    # probocar giros incorrectos cuando se avanza muy en diagonal 
    if not math.isnan(side_left_m) and not math.isnan(side_right_m):
        if abs(delta_side_m) < SIDE_DECISION_DEADBAND_M:
            # Desempate: usa front_left/front_right o mantiene el último giro.
            if (
                not math.isnan(front_left_m)
                and not math.isnan(front_right_m)
                and abs(front_left_m - front_right_m) >= FRONT_TIEBREAK_DEADBAND_M
            ):
                # Si el obstáculo está más cerca en el lado izquierdo del frente, gira a la derecha (y viceversa).
                turn_dir = "right" if front_left_m <= front_right_m else "left"
                return turn_dir, "front_tiebreak", delta_side_m
            if last_turn_dir in ("left", "right"):
                return last_turn_dir, "hold", delta_side_m
            
            return "right", "default", delta_side_m

        turn_dir = "left" if delta_side_m > 0.0 else "right"
        return turn_dir, "meters", delta_side_m

    # menor proximidad cruda => más espacio
    turn_dir = "left" if side_left_raw <= side_right_raw else "right"
    return turn_dir, "raw", delta_side_m


def _print_turn_decision(
    *,
    front_used_m: float,
    side_left_m: float,
    side_right_m: float,
    delta_side_m: float,
    side_left_raw: float,
    side_right_raw: float,
    turn_dir: str,
    decision_basis: str,
    prev_turn_dir: str,
) -> None:
    print(
        "Decisión de giro | "
        f"front_used={front_used_m:.3f}m (umbral={SAFE_DISTANCE_M:.3f}m) | "
        f"side_left={side_left_m:.3f}m side_right={side_right_m:.3f}m | "
        f"delta_side={delta_side_m:+.3f}m (deadband={SIDE_DECISION_DEADBAND_M:.3f}m) | "
        f"raw_left={side_left_raw:.1f} raw_right={side_right_raw:.1f} | "
        f"elige={turn_dir} (basis={decision_basis}, prev={prev_turn_dir or '-'})"
    )


def _reactive_step(
    *,
    robot: EpuckRobot,
    nav: ReactiveNavState,
    front_used_m: float,
    front_left_m: float,
    front_right_m: float,
    side_left_m: float,
    side_right_m: float,
    side_left_raw: float,
    side_right_raw: float,
    enc_left_rad: float,
    enc_right_rad: float,
) -> ReactiveNavState:
    """Update navigation state and apply wheel commands."""

    if nav.state == STATE_FORWARD:
        if front_used_m <= SAFE_DISTANCE_M:
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

            _print_turn_decision(
                front_used_m=front_used_m,
                side_left_m=side_left_m,
                side_right_m=side_right_m,
                delta_side_m=delta_side_m,
                side_left_raw=side_left_raw,
                side_right_raw=side_right_raw,
                turn_dir=nav.turn_dir,
                decision_basis=decision_basis,
                prev_turn_dir=prev_turn_dir,
            )

            nav.backup_remaining = int(BACKUP_HOLD_STEPS)
            nav.turn_travelled_wheel_rad = 0.0

            if nav.backup_remaining > 0:
                nav.state = STATE_BACKUP
                robot.wheels.backward(BACKUP_SPEED_FACTOR)
            else:
                nav.state = STATE_TURN
                _start_turn_position_control(
                    robot=robot,
                    nav=nav,
                    enc_left_rad=enc_left_rad,
                    enc_right_rad=enc_right_rad,
                )
        else:
            nav.turn_travelled_wheel_rad = 0.0
            nav.turn_dir = ""
            robot.wheels.forward(FORWARD_SPEED_FACTOR)

    elif nav.state == STATE_BACKUP:
        nav.turn_travelled_wheel_rad = 0.0
        robot.wheels.backward(BACKUP_SPEED_FACTOR)
        nav.backup_remaining -= 1
        if nav.backup_remaining <= 0:
            nav.state = STATE_TURN
            nav.turn_travelled_wheel_rad = 0.0
            _start_turn_position_control(
                robot=robot,
                nav=nav,
                enc_left_rad=enc_left_rad,
                enc_right_rad=enc_right_rad,
            )

    else:  # TURN
        nav.turn_travelled_wheel_rad = (
            abs(float(enc_left_rad) - nav.turn_start_left_rad)
            + abs(float(enc_right_rad) - nav.turn_start_right_rad)
        ) / 2.0
        left_err = abs(float(enc_left_rad) - float(nav.turn_target_left_rad))
        right_err = abs(float(enc_right_rad) - float(nav.turn_target_right_rad))
        if left_err <= TURN_POSITION_TOLERANCE_RAD and right_err <= TURN_POSITION_TOLERANCE_RAD:
            # Estimación del giro del robot usando encoders.
            # En giro sobre su propio eje, el target esperado es ~±TURN_ANGLE_RAD 
            # (= ±1.5715 rad para 90° rquivalente a pi/2).
            d_left = float(enc_left_rad) - nav.turn_start_left_rad
            d_right = float(enc_right_rad) - nav.turn_start_right_rad
            theta_est = math.nan
            if AXLE_LENGTH_M != 0.0:
                theta_est = WHEEL_RADIUS_M * (d_right - d_left) / AXLE_LENGTH_M

            print(
                "Fin giro | "
                f"dir={nav.turn_dir} | "
                f"theta_est={theta_est:+.3f}rad ({math.degrees(theta_est):+.1f}°) | "
                f"target_theta={TURN_ANGLE_RAD:+.3f}rad ({TURN_ANGLE_DEG:.0f}°) | "
                f"wheel={nav.turn_travelled_wheel_rad:.3f}/{TURN_WHEEL_COMMAND_RAD:.3f}rad | "
                f"errL={left_err:.3f}rad errR={right_err:.3f}rad"
            )
            nav.turn_target_left_rad = 0.0
            nav.turn_target_right_rad = 0.0
            nav.state = STATE_FORWARD
            robot.wheels.forward(FORWARD_SPEED_FACTOR)

    return nav


def _build_log_row(
    *,
    k: int,
    time_s: float,
    Ts: float,
    fs: float,
    nav: ReactiveNavState,
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
        "state": nav.state,
        "turn_dir": nav.turn_dir,
        "turn_travelled_wheel_rad": float(nav.turn_travelled_wheel_rad),
        "turn_target_wheel_rad": float(TURN_WHEEL_TARGET_RAD),
        "backup_remaining": int(nav.backup_remaining),
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
    row.update(
        {
            "front_left_m": float(front_left_m),
            "front_right_m": float(front_right_m),
            "front_raw_m": float(front_raw_m),
            "front_ema_m": float(front_ema_m),
            "front_kalman_m": float(front_kalman_m),
            "front_used_m": float(front_used_m),
            "side_left_m": float(side_left_m),
            "side_right_m": float(side_right_m),
            "side_left_raw": float(side_left_raw),
            "side_right_raw": float(side_right_raw),
        }
    )
    return row


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

    nav = ReactiveNavState()

    print(
        "Iniciando navegación reactiva | "
        f"CONTROL_SOURCE={CONTROL_SOURCE} | RUN_SECONDS={RUN_SECONDS} | Ts={Ts:.3f}s (fs={fs:.1f}Hz) | "
        f"SAFE_DISTANCE_M={SAFE_DISTANCE_M:.3f}"
    )
    print(f"Log CSV: {log_file}")

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
        print(f"WARNING: no se pudo crear el log CSV ({type(e).__name__}: {e}).")
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

            # Filtro simple(EMA)
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

            front_used_m = _select_front_used_m(
                CONTROL_SOURCE,
                front_raw_m=front_raw_m,
                front_ema_m=front_ema_m,
                front_kalman_m=front_kalman_m,
            )

            nav = _reactive_step(
                robot=robot,
                nav=nav,
                front_used_m=front_used_m,
                front_left_m=front_left_m,
                front_right_m=front_right_m,
                side_left_m=side_left_m,
                side_right_m=side_right_m,
                side_left_raw=side_left_raw,
                side_right_raw=side_right_raw,
                enc_left_rad=enc_left_rad,
                enc_right_rad=enc_right_rad,
            )

            cmd_left_vel, cmd_right_vel = robot.wheels.get_last_velocities()

            if logger is not None:
                logger.log(
                    _build_log_row(
                        k=k,
                        time_s=time_s,
                        Ts=Ts,
                        fs=fs,
                        nav=nav,
                        enc_left_rad=enc_left_rad,
                        enc_right_rad=enc_right_rad,
                        d_left_rad=d_left_rad,
                        d_right_rad=d_right_rad,
                        delta_s_m=delta_s_m,
                        delta_theta_rad=delta_theta_rad,
                        cmd_left_vel=cmd_left_vel,
                        cmd_right_vel=cmd_right_vel,
                        ps=ps,
                        front_left_m=front_left_m,
                        front_right_m=front_right_m,
                        front_raw_m=front_raw_m,
                        front_ema_m=front_ema_m,
                        front_kalman_m=front_kalman_m,
                        front_used_m=front_used_m,
                        side_left_m=side_left_m,
                        side_right_m=side_right_m,
                        side_left_raw=side_left_raw,
                        side_right_raw=side_right_raw,
                    )
                )

            k += 1
            if RUN_SECONDS is not None and time_s >= float(RUN_SECONDS):
                break

    finally:
        if logger is not None:
            logger.close()

    robot.stop()
    print(f"Fin. Muestras: {k} | Ts={Ts:.3f}s (fs={fs:.1f}Hz)")


if __name__ == "__main__":
    main()