#!/usr/bin/env python3
"""
Genera graficos comparando la ruta planificada con la trayectoria ejecutada.

Uso:
    python3 graficar_corridas.py              # usa ../logs/ y fallback a ../Lab_Final
    python3 graficar_corridas.py <logs_dir>   # especifica carpeta de logs

Los graficos se guardan en la misma carpeta donde esta este script (Analisis/).
"""

import os
import sys
import glob
import csv
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

LOGS_DIR  = os.path.join(os.path.dirname(__file__), '..', 'logs')
LAB_DIR   = os.path.join(os.path.dirname(__file__), '..', 'Lab_Final')
OUT_DIR   = os.path.dirname(os.path.abspath(__file__))

# Limites del mundo (min_x, max_x, min_y, max_y) segun escenario
WORLD_BOUNDS = {
    'escenario_complejo.wbt': (-1.5, 1.5, -1.5, 1.5),
    'lab2_simple.wbt':        (-0.5, 0.5, -0.5, 0.5),
}


# ---------------------------------------------------------------------------
# Lectura de archivos
# ---------------------------------------------------------------------------

def leer_metadata(log_path):
    """Lee las lineas de comentario al inicio del CSV de log."""
    meta = {}
    with open(log_path) as f:
        for line in f:
            if not line.startswith('#'):
                break
            if ':' in line:
                clave, _, valor = line[2:].partition(':')
                meta[clave.strip()] = valor.strip()
    return meta


def leer_trayectoria(log_path):
    """Extrae la posicion (x, y) y el tiempo de cada paso del log."""
    xs, ys, tiempos = [], [], []
    col_x = col_y = col_t = None
    with open(log_path) as f:
        for linea in f:
            if linea.startswith('#'):
                continue
            partes = linea.strip().split(',')
            if col_x is None:
                col_x = partes.index('x_m')
                col_y = partes.index('y_m')
                col_t = partes.index('time_s')
                continue
            if len(partes) <= max(col_x, col_y, col_t):
                continue
            xs.append(float(partes[col_x]))
            ys.append(float(partes[col_y]))
            tiempos.append(float(partes[col_t]))
    return xs, ys, tiempos


def leer_ruta(route_path):
    """Extrae los waypoints de la ruta planificada."""
    xs, ys = [], []
    with open(route_path) as f:
        for fila in csv.DictReader(f):
            xs.append(float(fila['x_m']))
            ys.append(float(fila['y_m']))
    return xs, ys


def leer_mapa(map_path, mundo):
    """Convierte el mapa ASCII en una imagen para usar de fondo."""
    if mundo not in WORLD_BOUNDS:
        return None
    min_x, max_x, min_y, max_y = WORLD_BOUNDS[mundo]
    with open(map_path) as f:
        lineas = f.read().splitlines()
    filas = len(lineas)
    cols  = max((len(l) for l in lineas), default=0)
    img = np.ones((filas, cols), dtype=float)
    for r, linea in enumerate(lineas):
        for c, ch in enumerate(linea):
            if ch == '#':
                img[r, c] = 0.0
    return img, min_x, max_x, min_y, max_y


# ---------------------------------------------------------------------------
# Emparejamiento de archivos
# ---------------------------------------------------------------------------

def buscar_pares(logs_dir):
    """Busca pares (log, ruta) que comparten el mismo timestamp.

    Si no encuentra pares en logs/, busca en Lab_Final/ los archivos de datos
    consolidados (datos_kalman_*.csv, datos_ruta_*.csv, datos_mapa_*.txt).
    """
    logs = sorted(glob.glob(os.path.join(logs_dir, 'final_kalman_*.csv')))
    pares = []
    for log in logs:
        ts = os.path.basename(log).replace('final_kalman_', '').replace('.csv', '')
        ruta = os.path.join(logs_dir, f'final_route_meta_{ts}.csv')
        mapa = os.path.join(logs_dir, f'final_map_meta_{ts}.txt')
        if os.path.exists(ruta):
            pares.append({
                'log':  log,
                'ruta': ruta,
                'mapa': mapa if os.path.exists(mapa) else None,
                'ts':   ts,
            })

    # Fallback: archivos consolidados en Lab_Final/
    if not pares and os.path.isdir(LAB_DIR):
        fallback = [
            {
                'log':  'datos_kalman_ruta_simple.csv',
                'ruta': 'datos_ruta_ruta_2_simple.csv',
                'mapa': 'datos_mapa_ruta_1_simple.txt',
                'ts':   'ruta_simple',
            },
            {
                'log':  'datos_kalman_ruta_1_compleja.csv',
                'ruta': 'datos_ruta_ruta_1_compleja.csv',
                'mapa': 'datos_mapa_ruta_1_compleja.txt',
                'ts':   'ruta_1_compleja',
            },
            {
                'log':  'datos_kalman_ruta_2_compleja.csv',
                'ruta': 'datos_ruta_ruta_2_compleja.csv',
                'mapa': 'datos_mapa_ruta_2_compleja.txt',
                'ts':   'ruta_2_compleja',
            },
        ]
        for item in fallback:
            log_path = os.path.join(LAB_DIR, item['log'])
            ruta_path = os.path.join(LAB_DIR, item['ruta'])
            mapa_path = os.path.join(LAB_DIR, item['mapa'])
            if os.path.exists(log_path) and os.path.exists(ruta_path):
                pares.append({
                    'log':  log_path,
                    'ruta': ruta_path,
                    'mapa': mapa_path if os.path.exists(mapa_path) else None,
                    'ts':   item['ts'],
                })
    return pares


# ---------------------------------------------------------------------------
# Calculo de metricas
# ---------------------------------------------------------------------------

def longitud(xs, ys):
    """Distancia total recorrida sumando segmentos consecutivos."""
    return sum(
        math.hypot(xs[i] - xs[i-1], ys[i] - ys[i-1])
        for i in range(1, len(xs))
    )


# ---------------------------------------------------------------------------
# Generacion de grafico
# ---------------------------------------------------------------------------

def graficar(par, out_dir):
    meta  = leer_metadata(par['log'])
    mundo = meta.get('world', '')

    traj_x, traj_y, tiempos = leer_trayectoria(par['log'])
    ruta_x, ruta_y          = leer_ruta(par['ruta'])

    if not traj_x:
        print(f"  Sin datos: {os.path.basename(par['log'])}")
        return

    fig, ax = plt.subplots(figsize=(8, 8))

    # Fondo: mapa de obstaculos
    if par['mapa']:
        resultado = leer_mapa(par['mapa'], mundo)
        if resultado:
            img, min_x, max_x, min_y, max_y = resultado
            ax.imshow(img, cmap='gray', origin='upper',
                      extent=[min_x, max_x, min_y, max_y],
                      vmin=0, vmax=1, alpha=0.35)

    # Ruta planificada
    ax.plot(ruta_x, ruta_y, 'b-o', linewidth=2, markersize=5,
            label='Ruta planificada', zorder=3)

    # Trayectoria real
    ax.plot(traj_x, traj_y, 'r-', linewidth=1.5, alpha=0.85,
            label='Trayectoria ejecutada', zorder=4)

    # Punto de inicio y llegada
    ax.plot(traj_x[0],  traj_y[0],  'go', markersize=10, label='Inicio',  zorder=5)
    ax.plot(traj_x[-1], traj_y[-1], 'r*', markersize=14, label='Llegada', zorder=5)

    # Metricas
    dist_ruta  = longitud(ruta_x, ruta_y)
    dist_traj  = longitud(traj_x, traj_y)
    duracion   = tiempos[-1] - tiempos[0] if len(tiempos) > 1 else 0
    error_pos  = math.hypot(traj_x[-1] - ruta_x[-1], traj_y[-1] - ruta_y[-1])

    escenario = 'Complejo' if 'complejo' in mundo else 'Simple'
    ax.set_title(f'Escenario {escenario}  —  {par["ts"]}', fontsize=12)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.legend(loc='upper left', fontsize=9)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    resumen = (
        f'Ruta planificada:  {dist_ruta:.2f} m\n'
        f'Trayectoria real:  {dist_traj:.2f} m\n'
        f'Diferencia:        {abs(dist_traj - dist_ruta):.2f} m\n'
        f'Tiempo total:      {duracion:.1f} s\n'
        f'Error posicion final: {error_pos:.3f} m'
    )
    # Cuadro de métricas en esquina inferior derecha, fuera del área principal de la ruta
    ax.text(0.98, 0.02, resumen, transform=ax.transAxes, fontsize=8,
            verticalalignment='bottom', horizontalalignment='right',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.85,
                      edgecolor='gray', linewidth=0.8))

    nombre = f'grafico_{escenario.lower()}_{par["ts"]}.png'
    salida = os.path.join(out_dir, nombre)
    plt.tight_layout()
    plt.savefig(salida, dpi=150)
    plt.close()

    print(f'  {nombre}')
    print(f'    Planificada: {dist_ruta:.2f} m | Ejecutada: {dist_traj:.2f} m | '
          f'Diferencia: {abs(dist_traj - dist_ruta):.2f} m | '
          f'Tiempo: {duracion:.1f} s | Error final: {error_pos:.3f} m')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    logs_dir = sys.argv[1] if len(sys.argv) > 1 else LOGS_DIR
    os.makedirs(OUT_DIR, exist_ok=True)

    pares = buscar_pares(logs_dir)
    if not pares:
        print(f'No se encontraron pares log/ruta en: {logs_dir}')
        return

    print(f'Encontradas {len(pares)} corridas. Generando graficos en {OUT_DIR}...\n')
    for par in pares:
        print(par['ts'])
        graficar(par, OUT_DIR)

    print(f'\nListo.')


if __name__ == '__main__':
    main()
