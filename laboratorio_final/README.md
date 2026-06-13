# Proyecto Final: Navegación Autónoma con Planificación de Rutas (A*) en Webots

**Curso:** ICI 4150 - Robótica y Sistemas Autónomos
**Semestre:** 2026-01
**Línea seleccionada:** Línea A — Planificación de rutas (A* sobre grilla de ocupación)
**Integrantes del grupo:**
- Ademir Muñoz
- Joaquín Tapia
- Matías Romero
- Fabrizzio Mura
- Vicente Sepúlveda

---

## 1. Objetivo del Proyecto

Diseñar, implementar y evaluar en Webots un sistema de navegación autónoma para un robot móvil diferencial (e-puck), capaz de desplazarse desde una posición inicial hasta una meta dentro de un entorno con obstáculos. El sistema integra el control cinemático diferencial del Laboratorio 1 y la percepción sensorial, filtrado y estimación del Laboratorio 2, agregando una capa de **navegación global**: el entorno se representa como una grilla de ocupación 2D y la ruta se calcula con el algoritmo **A\***, que luego el robot ejecuta siguiendo waypoints, con evasión reactiva de obstáculos como capa de seguridad.

## 2. Descripción del Robot, Sensores y Actuadores

Se utiliza el robot **e-puck** de Webots, un robot móvil diferencial.

| Componente | Uso en el proyecto |
|---|---|
| 2 motores de rueda (actuadores) | Control diferencial: avance, giros y seguimiento de waypoints. Velocidad máxima 6.28 rad/s |
| 2 encoders de rueda | Odometría: estimación de pose (x, y, θ) integrando los incrementos de cada rueda |
| 8 sensores de proximidad IR (ps0–ps7) | Detección de obstáculos. Frontales (ps0, ps7) para la capa reactiva; laterales (ps1, ps2, ps5, ps6) para decidir dirección de giro |

Parámetros geométricos: radio de rueda r = 0.0205 m, distancia entre ruedas L = 0.0573 m. Los valores crudos de los sensores IR se convierten a metros mediante la tabla de lookup inversa de Webots (interpolación lineal). El paso de simulación es Ts = 0.05 s (fs = 20 Hz).

## 3. Escenarios de Prueba

Ambos escenarios definen una **marca roja** (posición inicial del robot) y una **marca verde** (meta) en el piso de la arena.

- **`worlds/lab2_simple.wbt` (escenario simple):** arena de 1×1 m con dos muros que fuerzan una ruta en forma de "S" entre el inicio (-0.35, 0.35) y la meta (0.35, -0.35). Pocos obstáculos y ruta relativamente directa. Ruta planificada por A*: 11 waypoints, 1.47 m.

- **`worlds/escenario_complejo.wbt` (escenario complejo):** arena de 3×3 m modelada como una grilla de 12×12 celdas de 0.25 m. Contiene 52 obstáculos cúbicos (0.25×0.25 m) distribuidos en forma de laberinto con múltiples bloqueos y rutas alternativas. El robot parte de la esquina inferior-izquierda (-1.375, -1.375) y debe llegar a la esquina superior-derecha (1.375, 1.375), ambas en esquinas opuestas. Ruta planificada por A*: 14 waypoints, 4.91 m.

## 4. Algoritmo Implementado

### 4.1. Representación del entorno: grilla de ocupación precargada

Los archivos de mundo de Webots (`.wbt`) son texto plano en formato VRML. El módulo `world_map.py` los parsea directamente: extrae el tamaño de la arena (`RectangleArena.floorSize`) y cada obstáculo (`Solid` con geometría Box: posición, rotación en Z y dimensiones), y construye una **grilla de ocupación 2D** (`occupancy_grid.py`) donde cada celda de 0.05 m se marca como libre u ocupada. Los muros se rasterizan como rectángulos rotados y se les aplica una **inflación de 0.06 m** (mayor que el radio del e-puck, 0.037 m) para que el planificador mantenga distancia de seguridad. Los bordes de la arena también se marcan como ocupados.

El controlador detecta qué mundo está corriendo mediante `robot.getWorldPath()` y selecciona automáticamente el escenario (inicio/meta) y el mapa correspondiente.

### 4.2. Planificación: A* sobre la grilla

`path_planner.py` implementa **A\*** con conectividad 8 (movimientos ortogonales y diagonales):

- Costo real: 1 celda ortogonal = 0.05 m; diagonal = 0.05·√2 m.
- Heurística: distancia euclídea a la meta (admisible y consistente → ruta óptima).
- Las diagonales se bloquean si alguno de los vecinos ortogonales adyacentes está ocupado (evita cortar esquinas de muros).
- Si el inicio o la meta caen en zona inflada, se busca la celda libre más cercana (BFS).
- La ruta resultante se simplifica eliminando waypoints colineales.

Cada ruta planificada se guarda en `logs/` como `final_route_*.csv` (waypoints) y `final_map_*.txt` (mapa ASCII con la ruta superpuesta), para el análisis posterior.

### 4.3. Ejecución de la ruta: seguimiento de waypoints

El robot convierte la ruta en comandos de movimiento con un **control proporcional** tipo uniciclo (`_waypoint_step` en `Ruedas.py`): calcula el error de orientación hacia el waypoint actual, comanda velocidad angular ω = Kp·e_θ (Kp = 2.5) y reduce la velocidad lineal cuando el error de orientación es grande o el waypoint está cerca. Un waypoint se considera alcanzado a menos de 0.08 m.

### 4.4. Capa reactiva de seguridad

Si la distancia frontal (estimada con el **filtro de Kalman 1D** del Lab 2) cae bajo 0.17 m, la capa reactiva interrumpe el seguimiento: el robot retrocede (BACKUP), gira 90° hacia el lado más libre según los sensores laterales (TURN, con control por posición de encoders) y luego **reanuda el seguimiento de waypoints**. Esto protege contra desviaciones odométricas y obstáculos no modelados.

### 4.5. Pseudocódigo de la solución

```
INICIO
  mundo  ← getWorldPath()                       # mundo cargado en Webots
  (inicio, meta) ← SCENARIOS[mundo]
  grilla ← parsear .wbt y rasterizar obstáculos (+ inflación)
  ruta   ← A*(grilla, inicio, meta)             # lista de waypoints
  guardar ruta y mapa en logs/

  MIENTRAS simulación activa:
    leer sensores IR y encoders
    actualizar odometría (x, y, θ)              # ecuaciones del Lab 1
    d_frontal ← Kalman1D(predicción encoders, medición IR)   # Lab 2

    SI d_frontal < 0.17 m:                      # capa reactiva
      retroceder → girar 90° al lado más libre → reanudar ruta
    SINO:
      seguir waypoint actual (control proporcional)
      SI waypoint alcanzado: avanzar al siguiente

    SI último waypoint alcanzado:
      detener robot → META ALCANZADA
    registrar paso en CSV (pose, sensores, fase, comandos)
FIN
```

> El controlador conserva además un **modo alternativo de mapeo autónomo** (`USE_PRELOADED_MAP = False`): el robot explora reactivamente construyendo la grilla con sus sensores (ray-casting de Bresenham), vuelve al inicio y recién entonces planifica con A*. Se mantiene como extensión comparativa de la estrategia elegida.

## 5. Relación con los Laboratorios 1 y 2

| Laboratorio | Qué se reutiliza | Dónde |
|---|---|---|
| **Lab 1** — control cinemático | Modelo diferencial (v = r(ωr+ωl)/2, ω = r(ωr−ωl)/L), avance, giros por posición de encoders | `wheel.py`, capa reactiva y waypoint follower en `Ruedas.py` |
| **Lab 1** — odometría | Ecuaciones (5)–(7) del enunciado: integración de Δs y Δφ para estimar (x, y, θ) | `estimation.py` (clase `Odometry`), `robot.py` |
| **Lab 2** — percepción | Lectura de los 8 sensores IR y conversión cruda → metros por lookup table | `proximity.py` |
| **Lab 2** — filtrado y fusión | EMA (α = 0.25) y **Kalman 1D** (predicción por encoders + corrección por sensor IR, Q = 1e-4, R = 5e-3) sobre la distancia frontal | `estimation.py` |
| **Lab 2** — navegación reactiva | Máquina de estados FORWARD/BACKUP/TURN con decisión de giro por sensores laterales | `_reactive_step` en `Ruedas.py` |

El proyecto **extiende** estos aprendizajes con la navegación global: la odometría del Lab 1 deja de ser solo registro y pasa a alimentar el seguimiento de ruta; el Kalman del Lab 2 deja de controlar directamente y pasa a ser la capa de seguridad bajo un plan calculado por A*.

## 6. Instrucciones para Ejecutar

Requisitos: Webots R2023 o superior (probado con R2025a) y Python 3.10+.

1. Abrir Webots y cargar `laboratorio_final/worlds/lab2_simple.wbt` o `escenario_complejo.wbt` (File → Open World).
2. Presionar Play. El controlador detecta el mundo automáticamente: no hay que editar nada entre escenarios.
3. La consola muestra el mapa cargado, la ruta planificada y las transiciones de fase; al llegar imprime `META ALCANZADA` con pose, error y tiempo.
4. Al finalizar quedan en `laboratorio_final/logs/`:
   - `final_<modo>_<fecha>.csv` — registro completo paso a paso (pose, sensores, filtros, comandos, fase).
   - `final_route_meta_<fecha>.csv` — waypoints de la ruta planificada.
   - `final_map_meta_<fecha>.txt` — mapa ASCII con la ruta superpuesta.

Parámetros relevantes en `controllers/Ruedas/Ruedas.py`: `USE_PRELOADED_MAP` (Línea A vs mapeo autónomo), `CONTROL_SOURCE` (raw/filtered/kalman para la capa reactiva), `GRID_CELL_M`, `GRID_INFLATION_M`, `SAFE_DISTANCE_M`, y el diccionario `SCENARIOS` (inicio/meta por mundo).

## 7. Resultados y Métricas de Desempeño

> **Pendiente:** esta sección se completará con las corridas experimentales en ambos escenarios una vez finalizado el rediseño de los mapas.

Métricas a reportar por escenario (mínimo 5 corridas):

- Tiempo total hasta llegar a la meta.
- Longitud de la ruta planificada vs longitud de la trayectoria ejecutada (odometría) y diferencia entre ambas.
- Gráfico de ruta planificada vs trayectoria real sobre el mapa.
- Número de colisiones o casi-colisiones (distancia frontal < umbral).
- Número de activaciones de la capa reactiva (giros no planificados).
- Error de posición final (odometría vs marca de meta).
- Porcentaje de ejecuciones exitosas.

Validación offline ya realizada (sin simulación): el parser de mundos y A* encuentran ruta en ambos escenarios — simple: 11 waypoints, 1.47 m; complejo: 14 waypoints, 4.91 m atravesando el laberinto.

## 8. Evidencias

> **Pendiente:** capturas de ambos escenarios, gráficos de análisis y enlace al video demostrativo (ejecución en Webots mostrando la ruta seguida y la llegada a la meta en ambos escenarios).

## 9. Conclusiones, Limitaciones y Posibles Mejoras

> **Pendiente de resultados experimentales.** Limitaciones ya identificadas en el diseño:

- La pose del robot proviene solo de odometría: el error se acumula con la distancia recorrida y en rutas largas puede desviar el seguimiento de waypoints (sin corrección global tipo GPS/landmarks).
- En el escenario complejo, los corredores entre obstáculos de 0.25 m tienen un ancho libre de ~0.13 m tras la inflación de la grilla; el umbral reactivo `SAFE_DISTANCE_M = 0.17` puede activar giros en pasos estrechos — es el primer parámetro a calibrar experimentalmente.
- La inflación fija (0.06 m) es un compromiso: valores mayores dan más seguridad pero pueden cerrar pasillos estrechos en la grilla.
- Mejoras posibles: replanificación A* cuando la capa reactiva desvía al robot de la ruta, suavizado de trayectoria (línea de visión entre waypoints), y fusión de la odometría con mediciones absolutas.

## 10. Estructura del Repositorio

```
laboratorio_final/
├── README.md                  # este informe
├── ProyectoFinal.pdf          # enunciado
├── worlds/
│   ├── lab2_simple.wbt        # escenario simple (1×1 m, 2 muros)
│   └── escenario_complejo.wbt # escenario complejo (3×3 m, laberinto 12×12)
├── controllers/Ruedas/
│   ├── Ruedas.py              # controlador principal: escenarios, fases, waypoint follower
│   ├── world_map.py           # parser .wbt → grilla de ocupación precargada
│   ├── path_planner.py        # A* 8-conectado sobre la grilla
│   ├── occupancy_grid.py      # grilla 2D (rasterizado + ray-casting)
│   ├── robot.py               # EpuckRobot: motores, encoders, odometría
│   ├── estimation.py          # Odometry, Kalman1D, EMA
│   ├── proximity.py           # sensores IR y conversión a metros
│   ├── wheel.py               # control de ruedas
│   └── csv_logger.py          # registro CSV con metadata
└── logs/                      # CSVs de corridas, rutas y mapas generados
```
