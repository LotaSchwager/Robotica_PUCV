# Laboratorio 2: Navegación Reactiva con Filtrado y Fusión de Sensores en Webots

**Curso:** ICI 4150 - Laboratorios: Robótica y Sistemas Autónomos
**Semestre:** 2026-01
**Integrantes del grupo:**
- Ademir Muñoz
- Joaquín Tapia
- Matías Romero
- Fabrizzio Mura
- Vicente Sepúlveda

## 1. Objetivo del Trabajo

Implementar un sistema de navegación reactiva en Webots para un robot móvil diferencial. El sistema utiliza sensores de distancia y encoders de rueda, aplicando técnicas de filtrado y fusión sensorial mediante un filtro de Kalman para estimar la distancia frontal a obstáculos de forma más robusta. Se compara el comportamiento del robot usando tres fuentes de información: mediciones crudas, mediciones filtradas y estimación fusionada.

## 2. Descripción del Robot y Sensores Utilizados

Se utiliza el robot e-puck de Webots, un robot móvil diferencial con dos ruedas motorizadas independientes.

### Sensores de Proximidad/Distancia

El e-puck tiene 8 sensores de proximidad denominados ps0 a ps7. En este laboratorio se utilizan:

- **Sensores frontales:** ps0 (frente derecha) y ps7 (frente izquierda). Se toma el mínimo para obtener la distancia frontal al obstáculo más cercano.
- **Sensores laterales derechos:** ps1 y ps2. Se utilizan para decidir la dirección del giro.
- **Sensores laterales izquierdos:** ps5 y ps6. Se utilizan para decidir la dirección del giro.

Los sensores proporcionan valores crudos que se convierten a distancia en metros utilizando una tabla de lookup inversa de Webots (interpolación lineal mediante bisect).

### Encoders de Rueda

El robot dispone de dos encoders que miden el desplazamiento angular en radianes de cada rueda. Estos se utilizan para estimar el movimiento incremental del robot entre dos instantes consecutivos. El desplazamiento lineal se calcula como s = r × θ, donde r = 0.0205 m es el radio de la rueda.

## 3. Frecuencia de Muestreo

La simulación en Webots ejecuta el controlador con un paso de tiempo fijo de Ts = 0.05 s, lo que corresponde a una frecuencia de muestreo fs = 20 Hz. Las pruebas tienen una duración típica de 600 a 1200 segundos de simulación. Todas las señales registradas, filtradas y estimadas se analizan con esta misma frecuencia.

## 4. Análisis de Señales Registradas

Durante la simulación se registran continuamente los valores crudos de los sensores de proximidad, posiciones angulares de los encoders, velocidades comandadas a los motores, distancias convertidas a metros y señales filtradas y estimadas.

Los archivos se guardan como CSV en la carpeta `laboratorio_2/logs/` con los siguientes nombres:
- `lab2_raw_*.csv` para mediciones crudas
- `lab2_filtered_*.csv` para mediciones filtradas
- `lab2_kalman_*.csv` para estimación Kalman

Cada archivo contiene columnas para tiempo, estado, lecturas sensoriales, velocidades comandadas y las tres versiones de la distancia frontal.

## 5. Estimación del Avance Mediante Encoders

El movimiento del robot se estima a partir de los encoders usando las siguientes fórmulas.

El desplazamiento lineal es delta_s = r × (d_left + d_right) / 2, donde r = 0.0205 m y d_left, d_right son los cambios angulares de cada rueda.

El desplazamiento angular es delta_theta = r × (d_right - d_left) / axle_length, donde axle_length = 0.0573 m es la distancia entre ejes.

El desplazamiento lineal delta_s se utiliza como entrada de predicción en el filtro de Kalman. Si el robot avanza delta_s metros, la distancia frontal debería disminuir en delta_s metros.

## 6. Filtro Simple Aplicado

Se implementa un filtro de promedio móvil exponencial (EMA) sobre las mediciones frontales de distancia. La ecuación es y_k = α × x_k + (1 - α) × y_{k-1}, donde y_k es la salida filtrada, x_k es la medición cruda y α = 0.25 es el factor de suavizado.

Con α = 0.25, el filtro retiene el 75% del valor anterior y añade el 25% de la medición actual. Esto reduce el ruido pero introduce un pequeño retraso en la respuesta.

La clase `ExponentialMovingAverage` en `estimation.py` (líneas 8-23) implementa este filtro.

## 7. Implementación del Filtro de Kalman

Se implementa un filtro de Kalman escalar (1D) que fusiona información del movimiento del robot (predicción por encoders) con mediciones directas de sensores (corrección).

El modelo matemático define el estado como x_k = x_{k-1} + u_k + w_k, donde w_k ~ N(0, Q), y la medición como z_k = x_k + v_k, donde v_k ~ N(0, R).

En este contexto, x_k es la distancia frontal estimada, u_k es el cambio de distancia predicho por encoders, z_k es la lectura del sensor frontal, Q = 1e-4 es la varianza del proceso y R = 5e-3 es la varianza de medición.

La clase `Kalman1D` en `estimation.py` (líneas 27-69) implementa este filtro.

## 8. Etapas del Filtro de Kalman: Predicción y Corrección

El filtro de Kalman ejecuta dos etapas en cada iteración.

En la etapa de predicción, se estima la nueva distancia frontal basándose en cuánto se ha movido el robot: x_pred = x_anterior + u, P_pred = P_anterior + Q. Si delta_s_m = 0.05 m, entonces u = -0.05 m.

En la etapa de corrección, se ajusta la predicción anterior usando una medición real. Se calcula la innovacion = z - x_pred, la ganancia = P_pred / (P_pred + R), y finalmente x_nuevo = x_pred + ganancia × innovacion, P_nuevo = (1 - ganancia) × P_pred.

La ganancia de Kalman determina cuánto confiar en la medición versus la predicción. Si R es grande (sensor ruidoso), la ganancia tiende a 0 y confía más en la predicción. Si R es pequeño (sensor preciso), la ganancia tiende a 1 y confía más en la medición. Si P_pred es grande (incertidumbre alta), la ganancia aumenta y confía más en la medición.

El resultado es una estimación más estable que cualquiera de las dos fuentes por separado.

## 9. Lógica de Navegación Reactiva Implementada

El robot implementa una máquina de estados con tres estados: FORWARD, BACKUP y TURN.

En el estado FORWARD, el robot avanza a velocidad 0.55 × MAX_SPEED (3.44 rad/s) mientras la distancia frontal sea mayor que 0.17 m. Si la distancia es menor, transiciona a BACKUP.

En el estado BACKUP, cuando se detecta un obstáculo, el robot retrocede a velocidad 0.45 × MAX_SPEED durante 17 pasos (0.85 segundos) para proporcionar espacio antes de girar.

En el estado TURN, el robot gira 90° sobre su propio eje utilizando control por posición de los encoders. La dirección del giro se decide según los sensores laterales. Si delta_side = side_left - side_right y |delta_side| < 0.01 m, el robot gira hacia el lado más libre. Si los lados son similares, usa sensores frontales para desempate. Una vez completado el giro (tolerancia de 0.005 rad), transiciona a FORWARD.

La función `_reactive_step` en `Ruedas.py` (líneas 214-318) implementa esta lógica.

## 10. Fuente de Control Configurable

El comportamiento del robot puede cambiar modificando la variable CONTROL_SOURCE en Ruedas.py:

"raw" usa mediciones crudas de sensores frontales, lo que produce un comportamiento más reactivo pero con oscilaciones. "filtered" usa mediciones filtradas con EMA, produciendo un comportamiento suavizado. "kalman" usa estimación del filtro de Kalman, lo que resulta en un comportamiento más estable y predictivo.

Cada ejecución se registra en un CSV diferente para permitir la comparación entre las tres estrategias.

## 11. Parámetros Configurables

En `laboratorio_2/controllers/Ruedas/Ruedas.py` se encuentran los siguientes parámetros ajustables:

CONTROL_SOURCE define la fuente de control (raw, filtered o kalman). SAFE_DISTANCE_M = 0.17 m es el umbral de detección de obstáculos. SIDE_DECISION_DEADBAND_M = 0.01 m es la tolerancia para el desempate lateral. EMA_ALPHA = 0.25 es el factor de suavizado del filtro. KALMAN_P0 = 0.05 es la covarianza inicial, KALMAN_Q = 1e-4 es la varianza del proceso y KALMAN_R = 5e-3 es la varianza de medición.

FORWARD_SPEED_FACTOR = 0.55 define la velocidad de avance. TURN_SPEED_FACTOR = 0.35 define la velocidad de giro. BACKUP_HOLD_STEPS = 17 pasos corresponden a 0.85 segundos a 20 Hz. WHEEL_RADIUS_M = 0.0205 m y AXLE_LENGTH_M = 0.0573 m son parámetros geométricos del robot.

## 12. Escenarios de Prueba

Se han diseñado dos escenarios de prueba. lab2_simple.wbt es un ambiente con pocos obstáculos distribuidos en el espacio que permite validar la navegación reactiva básica en condiciones controladas. lab2_complex.wbt es un ambiente con múltiples obstáculos, pasillos estrechos y geometrías más desafiantes que valida la robustez en condiciones más realistas.

En ambos escenarios se analiza la estabilidad del movimiento, la cantidad de giros innecesarios, la capacidad para evitar colisiones y las diferencias en comportamiento entre las tres estrategias.

## 13. Gráficos Generados

En la carpeta `Analisis/graficos/` se encuentran gráficos que muestran posiciones de encoders, velocidades comandadas, desplazamiento estimado, comparativas entre señales raw y filtradas, estimación del filtro de Kalman, transiciones de estados de navegación, lecturas de sensores laterales, comparación de distancia frontal entre las tres estrategias, órdenes de velocidad y distribución de tiempo en cada estado.

## 14. Instrucciones para Ejecutar

### Requisitos

Para ejecutar las simulaciones se necesita Webots instalado (versión 2023 o superior) y Python 3.7 en adelante.

### Pasos

1. Abrir Webots desde terminal ejecutando `webots &`

2. Cargar un mundo desde el menú File → Open World y seleccionar lab2_simple.wbt o lab2_complex.wbt

3. Opcionalmente, editar `laboratorio_2/controllers/Ruedas/Ruedas.py` para cambiar CONTROL_SOURCE (el valor predeterminado es "kalman")

4. Ejecutar haciendo clic en el botón Play en Webots

5. Los datos se guardan automáticamente en `laboratorio_2/logs/` al finalizar la simulación

Los archivos CSV generados pueden analizarse con Python para generar gráficos y estadísticas comparativas.

## Estado de Implementación

Se han completado todos los componentes principales del laboratorio:
- Lectura de sensores de proximidad y encoders
- Conversión de mediciones crudas a distancia en metros
- Filtro EMA con parámetros configurables
- Filtro de Kalman 1D con predicción y corrección
- Máquina de estados de navegación reactiva con tres estados
- Decisión de dirección de giro basada en sensores laterales
- Registro automático de señales en CSV
- Comparación de comportamiento con tres fuentes de control
- Dos escenarios de prueba (simple y complejo)
- Gráficos de resultados y análisis comparativo de rendimiento

## Resultados Lab 2 - Escenario Complejo

Los siguientes gráficos corresponden al escenario complejo, registrando simulaciones que completan el circuito hasta la meta.

Se muestran el registro de señales crudas de sensores y encoders, la estimación de avance a partir de encoders, la comparación entre el filtro EMA y las mediciones crudas, la estimación del filtro de Kalman superpuesta con raw y EMA, las transiciones de estados de navegación, el uso de sensores laterales para decidir la dirección de giro, la comparación de comportamiento entre las tres estrategias, y el tiempo de llegada a la meta para cada modo de control.

![Encoders y comandos](Analisis/graficos_complejos/encoders_velocidades.png)

![Desplazamiento y orientación](Analisis/graficos_complejos/desplazamiento_encoders.png)

![Comparativa filtros](Analisis/graficos_complejos/comparativa_filtros.png)

![Comparativa filtros full](Analisis/graficos_complejos/comparativa_filtros_full.png)

![Estados de navegación](Analisis/graficos_complejos/estados_navegacion.png)

![Sensores laterales](Analisis/graficos_complejos/sensores_laterales.png)

![Distancia frontal usada](Analisis/graficos_complejos/comparacion_front_used.png)

![Comandos de control](Analisis/graficos_complejos/comparacion_cmd.png)

![Distribución de estados](Analisis/graficos_complejos/comparacion_comportamiento.png)

![Tiempo de llegada a la meta](Analisis/graficos_complejos/tiempo_llegada_meta.png)

## 15. Análisis Comparativo: Raw vs Filtered vs Kalman

### Tabla de Métricas (Escenario Complejo)

| Métrica | RAW | FILTERED | KALMAN |
|---------|-----|----------|--------|
| Tiempo Total (s) | 221.15 | 176.64 | 176.45 |
| Pasos Totales | 6912 | 5521 | 5515 |
| Pasos FORWARD | 6032 | 4641 | 4635 |
| Pasos BACKUP | 272 | 272 | 272 |
| Pasos TURN | 608 | 608 | 608 |
| Transiciones de Estado | 48 | 48 | 48 |
| Distancia Promedio (m) | 0.1961 | 0.1956 | 0.1840 |
| Desv. Est. Distancia (m) | 0.0038 | 0.0047 | 0.0056 |
| Distancia Mínima (m) | 0.1353 | 0.1543 | 0.1632 |
| Distancia Máxima (m) | 0.1973 | 0.1970 | 0.2022 |
| Eventos Colisión Cercana | 32 | 66 | 41 |

### Interpretación de Resultados

En cuanto a la eficiencia de tiempo, RAW tarda 25% más que FILTERED y KALMAN. Las mediciones crudas producen oscilaciones que generan más transiciones y movimientos innecesarios, mientras que FILTERED y KALMAN logran completar el circuito en tiempo similar.

Respecto a la estabilidad del movimiento, RAW tiene la menor varianza pero esto se debe a oscilaciones rápidas sin cambios significativos. KALMAN tiene mayor varianza, indicando una estimación más confiada que se adapta mejor a cambios reales. FILTERED es intermedio en varianza.

En cuanto a la seguridad ante colisiones, KALMAN mantiene la distancia mínima más alta (0.1632 m), siendo más conservador. RAW se acerca más al umbral crítico (0.1353 m) con 32 eventos de colisión cercana. FILTERED tiene la mayoría de eventos cercanos (66), sugiriendo que el filtro EMA suaviza demasiado y retarda la respuesta.

El análisis general muestra que KALMAN completa el circuito en tiempo comparable a FILTERED, mantiene mayor distancia de seguridad y combina predicción con medición para decisiones robustas. RAW es más reactivo pero las oscilaciones causan ineficiencia temporal y múltiples transiciones innecesarias. FILTERED reduce el ruido pero el retraso del EMA causa sobre-corrección y más eventos de colisión cercana.

