"""
Carga de mapas conocidos desde archivos de mundo de Webots (.wbt).

Los .wbt son texto plano (formato VRML), por lo que se pueden parsear
directamente.  Este módulo extrae:

  - El tamaño de la arena (campo ``floorSize`` de ``RectangleArena``).
  - Los obstáculos: nodos ``Solid`` con geometría/boundingObject ``Box``
    (posición, rotación en Z y dimensiones), como los muros del laberinto.

Con esa información construye una ``OccupancyGrid`` precargada: el mapa
"conocido a priori" que usa la Línea A (planificación de rutas) para
ejecutar A* de inmediato, sin necesidad de explorar primero el entorno.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path

from occupancy_grid import OccupancyGrid

# Número de punto flotante en formato VRML (acepta notación exponencial)
_FLOAT = r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"

_TRANSLATION_RE = re.compile(rf"translation\s+({_FLOAT})\s+({_FLOAT})\s+({_FLOAT})")
_ROTATION_RE = re.compile(
    rf"rotation\s+({_FLOAT})\s+({_FLOAT})\s+({_FLOAT})\s+({_FLOAT})"
)
_BOX_SIZE_RE = re.compile(rf"Box\s*\{{\s*size\s+({_FLOAT})\s+({_FLOAT})\s+({_FLOAT})")
_FLOOR_SIZE_RE = re.compile(rf"floorSize\s+({_FLOAT})\s+({_FLOAT})")
_NAME_RE = re.compile(r'name\s+"([^"]*)"')


@dataclass
class BoxObstacle:
    """Obstáculo rectangular extraído del mundo (muro o caja)."""

    x: float
    y: float
    angle_rad: float   # rotación alrededor del eje Z
    size_x: float
    size_y: float
    name: str = ""


@dataclass
class WorldModel:
    """Modelo geométrico 2D del mundo: arena centrada en el origen + obstáculos."""

    arena_width: float
    arena_height: float
    obstacles: list[BoxObstacle]


def _find_blocks(text: str, node_name: str) -> list[str]:
    """
    Retorna el cuerpo (contenido entre llaves) de cada nodo ``node_name { ... }``
    balanceando llaves, ya que los nodos VRML se anidan.
    """
    blocks: list[str] = []
    for match in re.finditer(rf"\b{node_name}\s*\{{", text):
        depth = 1
        i = match.end()
        while i < len(text) and depth > 0:
            if text[i] == "{":
                depth += 1
            elif text[i] == "}":
                depth -= 1
            i += 1
        blocks.append(text[match.end():i - 1])
    return blocks


def _z_rotation_rad(block: str) -> float:
    """
    Extrae la rotación alrededor del eje Z de un nodo.  Los muros de Webots
    usan ejes (0 0 ±1); cualquier otro eje se ignora (rotación 3D no
    relevante para la proyección 2D).
    """
    m = _ROTATION_RE.search(block)
    if not m:
        return 0.0
    ax, ay, az, angle = (float(m.group(i)) for i in range(1, 5))
    if abs(az) < 0.9:
        return 0.0
    return angle if az > 0 else -angle


def parse_world(wbt_path: str | Path) -> WorldModel:
    """Parsea un .wbt y retorna la arena y los obstáculos Box encontrados."""
    text = Path(wbt_path).read_text()

    # Arena (RectangleArena tiene floorSize 1x1 por defecto)
    arena_w, arena_h = 1.0, 1.0
    for body in _find_blocks(text, "RectangleArena"):
        m = _FLOOR_SIZE_RE.search(body)
        if m:
            arena_w, arena_h = float(m.group(1)), float(m.group(2))

    obstacles: list[BoxObstacle] = []
    for body in _find_blocks(text, "Solid"):
        tr = _TRANSLATION_RE.search(body)
        box = _BOX_SIZE_RE.search(body)
        if not tr or not box:
            continue
        name_m = _NAME_RE.search(body)
        obstacles.append(BoxObstacle(
            x=float(tr.group(1)),
            y=float(tr.group(2)),
            angle_rad=_z_rotation_rad(body),
            size_x=float(box.group(1)),
            size_y=float(box.group(2)),
            name=name_m.group(1) if name_m else "",
        ))

    return WorldModel(arena_width=arena_w, arena_height=arena_h, obstacles=obstacles)


def build_grid_from_world(
    wbt_path: str | Path,
    cell_size_m: float = 0.05,
    inflation_m: float = 0.06,
) -> tuple[OccupancyGrid, WorldModel]:
    """
    Construye una OccupancyGrid precargada con los obstáculos del mundo.

    La grilla cubre exactamente la arena (centrada en el origen); los bordes
    de la arena se marcan como muros y cada obstáculo se rasteriza como un
    rectángulo rotado con la inflación configurada.
    """
    world = parse_world(wbt_path)

    half_w = world.arena_width / 2.0
    half_h = world.arena_height / 2.0

    grid = OccupancyGrid(
        world_min_x=-half_w,
        world_max_x=half_w,
        world_min_y=-half_h,
        world_max_y=half_h,
        cell_size_m=cell_size_m,
        inflation_m=inflation_m,
    )

    # Muros perimetrales de la arena (espesor cero + inflación)
    grid.mark_rotated_rect(0.0, -half_h, half_w, 0.0)
    grid.mark_rotated_rect(0.0, half_h, half_w, 0.0)
    grid.mark_rotated_rect(-half_w, 0.0, 0.0, half_h)
    grid.mark_rotated_rect(half_w, 0.0, 0.0, half_h)

    # Obstáculos del mundo (los que caen fuera de la arena se recortan solos)
    for ob in world.obstacles:
        grid.mark_rotated_rect(
            ob.x, ob.y, ob.size_x / 2.0, ob.size_y / 2.0, ob.angle_rad
        )

    return grid, world
