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
  * trayectoria ideal
  * trayectoria con variaciones

### Desafío
Programar el robot para:
  * Dibujar una línea recta
  * Dibujar una curva
  * Dibujar un círculo
  * (Opcional) Dibujar un cuadrado o figura en 8

### Preguntas de análisis
  1. ¿Qué ocurre cuando ambas ruedas tienen la misma velocidad?
  2. ¿Cómo cambia la trayectoria cuando las velocidades son diferentes?
  3. ¿Qué ocurre cuando una rueda gira en sentido opuesto a la otra?
  4. ¿Qué tipo de movimiento permite dibujar un círculo?

## Como ejecutar el mundo
Dentro de la carpeta ```\worlds``` existe un archivo de extensión wbt; ese es el ambiente donde estará <br> 
el robot e-puck y esa es aquella que debe ser abierta en la opción ```Abrir mundo.....``` dentro de Webots. <br> 
El controlador está dentro de la carpeta ```\controllers\Ruedas``` donde estará un archivo de Python que <br> 
será aquel que se debe editar para realizar lo necesario para realizar el laboratorio.

## Resultados
