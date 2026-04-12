# Laboratorio 1

## Descripción del laboratorio

Este laboratorio consistirá en programar y testear con un robot de nombre e-puck, donde se llevarán <br>
a cabo distintas pruebas en relación a las velocidades de su par de ruedas. La prueba se llevará a <br>
cabo en un mundo simple que tendrá forma de caja y no habrá ningún obstáculo. Las tareas a realizar <br>
serán las siguientes: <br>

### Línea principal del laboratorio

1. Abrir Webots y cargar un robot diferencial (por ejemplo e-puck).
2. Identificar los motores de las ruedas izquierda y derecha.
3. Programar el robot para controlar las velocidades de las ruedas.
4. Ejecutar los siguientes experimentos: <br>
   $$v_r = v_l$$ → **movimiento recto** <br>
   $$v_r \neq v_l$$ → **trayectoria curva** <br>
   $$v_r = −v_l$$ → **rotación en el lugar** <br>
5. Observar y describir la trayectoria del robot.

### Extensión de la línea principal

Modificar las velocidades de las ruedas en cada iteración para simular perturbaciones en los <br>
actuadores, comparar:

- trayectoria ideal
- trayectoria con variaciones

### Desafío

Programar el robot para:

- Dibujar una línea recta
- Dibujar una curva
- Dibujar un círculo
- (Opcional) Dibujar un cuadrado o figura en 8

### Preguntas de análisis

1. ¿Qué ocurre cuando ambas ruedas tienen la misma velocidad?
2. ¿Cómo cambia la trayectoria cuando las velocidades son diferentes?
3. ¿Qué ocurre cuando una rueda gira en sentido opuesto a la otra?
4. ¿Qué tipo de movimiento permite dibujar un círculo?

## Como ejecutar el mundo

Para ejecutar el mundo de simulación, debes dirigirte a la carpeta `worlds` y abrir el archivo con extensión `.wbt` en Webots (puedes utilizar la opción `File -> Open World...` o `Archivo -> Abrir mundo...` dentro del simulador). Este archivo contiene el ambiente donde se encuentra el robot e-puck.

El robot e-puck utiliza como controlador principal el script ubicado en `laboratorio_1/controllers/Ruedas/Ruedas.py`. Este archivo es el punto de entrada que ejecuta la lógica de movimiento y evasión de obstáculos.

Además, el sistema separa el controlador de las ruedas y sensor de proximidad, para que cada archvo tenga un rol concreto:

**Clase `Robot` (`laboratorio_1/controllers/Ruedas/robot.py`)**
Esta clase se encarga del manejo del robot e-puck:

- `__init__`: Inicializa el robot y sus componentes.
- `step`: Avanza un paso en la simulación.
- `move_steps`: Ordena mover el robot en línea recta por 'target_rads' cantidad de radianes (pasos).
- `stop`: Detiene el robot.
- `turn_steps`: Gira el robot en su propio eje cierta cantidad de pasos (radianes).
- `move_circle`: Realiza un movimiento circular.
- `rotate_random`: Gira el robot para evadir un obstáculo, asegurando darle la espalda (quedar cara a cara a lo libre).

**Clase `WheelController` (`laboratorio_1/controllers/Ruedas/wheel.py`)**
Esta clase se encarga del manejo directo de los motores correspondientes a ambas ruedas:

- `__init__`: Inicializa los motores izquierdo y derecho, junto con sus respectivos sensores de posición (encoders), habilitándolos e indicando una posición infinita para controlarlos por límite de velocidad.
- `get_positions`: Retorna una tupla representativa con los radianes exactos que han girado ambas ruedas utilizando la lectura de los sensores.
- `set_velocities`: Método base que asigna las velocidades a los motores.
- `forward` y `backward`: Hacen avanzar o retroceder al robot en línea recta en función de una velocidad máxima y un factor.
- `stop`: Detiene completamente el movimiento de los motores de manera predeterminada.
- `turn_own_axis_left` y `turn_own_axis_right`: Fijan la velocidad de las ruedas en signos opuestos para que el robot pueda girar sobre su propio eje.
- `curve_left` y `curve_right`: Asignan distintas escalas de rotación a las ruedas para generar una trayectoria curva hacia un lado u otro.

**Clase `ProximityController` (`laboratorio_1/controllers/Ruedas/proximity.py`)**
Esta clase se encarga del manejo de los sensores de proximidad del robot:

- `__init__`: Inicializa los 8 sensores de proximidad del robot.
- `get_values`: Retorna una lista con los valores actuales de los 8 sensores.
- `is_obstacle_ahead`: Detecta si hay un obstáculo enfrente.
- `front_obstacle_hits`: Devuelve qué sensores frontales detectan obstáculo.
- `front_obstacle_count`: Devuelve cuántos sensores frontales están activos.

# Resultados obtenidos:

Se ha logrado resolver satisfactoriamente la actividad, respondiendo a cada una de las preguntas y diseñando una serie de clases que organizaron de mejor manera el código para poder trabajar en equipo de manera conjunta y teniendo todos a disposición las mismas funciones para trabajar y trazar los movimientos o rutas del robot.

Pudimos observar por separado las rutas del robot requeridas para completar la actividad, comprendiendo como funcionan las físicas y cámara del mundo, y además que sensores y actuadores tuvimos a disposición en el robot epuck para poder trabajar luego con estos (donde fue verdaderamente importante reconocer los nombres de las ruedas del robot y la cantidad y ubicaciones de cada uno de los sensores de proximidad).




## Preguntas

Respuestas a las preguntas del pdf.

### 1: ¿Qué ocurre cuando ambas ruedas tienen la misma velocidad?

Respuesta: El robot continua en linea recta si es que no se ha definido una ruta distinta (Vr = Vi).

### 2: ¿Cómo cambia la trayectoria cuando las velocidades son diferentes?

Respuesta: El robot rota en la misma direccion que la rueda con menor velocidad (ej: si Vi (velocidad de la rueda izquierda) es menor, girará hacia la izquierda), pero girará en un circulo si la Vi se mantiene constante en el tiempo (Vr != Vi).

### 3: ¿Qué ocurre cuando una rueda gira en sentido opuesto a la otra?

Respuesta: El robot comienza a rotar en el mismo lugar, esto en caso que [Vr != -Vl] En caso que las velocidades sean distintas y en sentidos opuestos, el robot comienza a desplazarse a favor de la rueda de mayor velocidad pero rotando en dirección de la rueda de menor velocidad.


### 4: ¿Qué tipo de movimiento permite dibujar un círculo?

Respuesta: El robot necesita que las ruedas giren al mismo sentido pero con velocidades distintas (Vr != Vl), con esto el robot puede tener una trayectoría curva.
