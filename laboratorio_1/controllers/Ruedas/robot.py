from controller import Robot
from enum import Enum
import math
import random

from wheel import WheelController
from proximity import ProximityController

class RobotState(Enum):
    RUNNING = "RUNNING"
    STOPPED = "STOPPED"
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"
    OWN_TURN = "OWN_TURN"
    CIRCLE = "CIRCLE"

class EpuckRobot:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Pasamos el timestep a las ruedas para que pueda inicializar los sensores de posición
        self.wheels = WheelController(self.robot, self.timestep)
        self.proximity = ProximityController(self.robot, self.timestep)
        self.state = RobotState.STOPPED

    def step(self):
        return self.robot.step(self.timestep) != -1

    def move_steps(self, target_rads, speed_factor=0.5):
        """
        Ordena mover el robot en línea recta por 'target_rads' cantidad de radianes (pasos).
        Si el sensor detecta un obstáculo antes de terminar los pasos, se detiene
        completamente y cambia su estado a STOPPED.
        """
        self.state = RobotState.RUNNING
        # Obtenemos la medida inicial de los sensores de posición (encoders)
        start_left, start_right = self.wheels.get_positions()
        
        print(f"-> [ESTADO: {self.state.value}] Instrucción recibida: avanzar por {target_rads} pasos (radianes).")
        self.wheels.forward(speed_factor)
        
        while self.step():
            # Condición de control: ¿Hay un obstáculo interrumpiendo el progreso?
            if self.proximity.is_obstacle_ahead(threshold=100.0):
                print(f"¡Obstáculo, deteniéndose...")
                self.stop()
                return False  # Indica que no completó toda la distancia
                
            # ¿Cuántos pasos se ha dado?
            curr_left, curr_right = self.wheels.get_positions()
            
            # Promediamos la distancia recorrida de ambas ruedas (valor absoluto)
            dist_travelled = (abs(curr_left - start_left) + abs(curr_right - start_right)) / 2.0
            
            if dist_travelled >= target_rads:
                print(f"Avance completado con éxito ({target_rads} pasos recorridos).")
                self.stop()
                return True # Indica que completó la distancia
                
        return False
    
    def stop(self):
        """Detiene el robot y cambia su estado a STOPPED."""
        self.wheels.stop()
        self.state = RobotState.STOPPED
        print(f"-> [ESTADO: {self.state.value}] Robot detenido.")

    def turn_steps(self, target_rads, direction='izquierda', speed_factor=0.5, check_obstacle=True):
        """
        Gira en su propio eje cierta cantidad de pasos (radianes). 
        También chequea obstáculos en pleno giro, a menos que check_obstacle sea False.
        """
        self.state = RobotState.OWN_TURN
        start_left, start_right = self.wheels.get_positions()
        
        if direction in ('izquierda', -1):
            direction_label = 'izquierda'
        else:
            direction_label = 'derecha'

        print(f"-> [ESTADO: {self.state.value}] Instrucción recibida: girar {target_rads} pasos a la {direction_label}.")
        if direction_label == 'izquierda':
            self.wheels.turn_own_axis_left(speed_factor)
        else:
            self.wheels.turn_own_axis_right(speed_factor)
            
        while self.step():
            # Condición de control durante el giro
            if check_obstacle and self.proximity.is_obstacle_ahead(threshold=100.0):
                print(f"¡Obstáculo detectado, deteniéndose...")
                self.stop()
                return False
                
            curr_left, curr_right = self.wheels.get_positions()
            
            # Al girar sobre su eje, un motor suma y el otro resta. Tomamos valor absoluto.
            dist_travelled = (abs(curr_left - start_left) + abs(curr_right - start_right)) / 2.0
            
            if dist_travelled >= target_rads:
                print(f"Giro completado con éxito ({target_rads} pasos girados).")
                self.stop()
                return True
                
        return False

    def move_circle(self, radius_steps, speed_factor=0.5):
        """Realiza un movimiento circular."""
        self.state = RobotState.CIRCLE
        print(f"-> [ESTADO: {self.state.value}] Realizando círculo.")
        self.wheels.curve_right(speed_factor) # Ajustable según el radio deseado
        # Simplemente corre por un tiempo o distancia fija
        count = 0
        while self.step() and count < radius_steps * 100:
            if self.proximity.is_obstacle_ahead(threshold=100.0):
                self.stop()
                return False
            count += 1
        self.stop()
        return True

    def rotate_random(self):
        """Gira el robot para evadir un obstáculo, asegurando darle la espalda (quedar cara a cara a lo libre)."""
        values = self.proximity.get_values()
        
        # Leemos los sensores para saber qué lado está más cerca de la pared.
        # Sensores izquierdos: ps7, ps6, ps5. Sensores derechos: ps0, ps1, ps2.
        left_val = values[7] + values[6] + values[5]
        right_val = values[0] + values[1] + values[2]
        
        # Para evitar enfrascarnos en esquinas, giramos hacia el lado más despejado (menor medición)
        if left_val > right_val:
            direction = 1  # Girar a la derecha
        else:
            direction = -1 # Girar a la izquierda
            
        # Le damos un buen giro para darle la espalda al obstáculo de manera segura (ej: ~135 a 180 grados).
        angle = random.uniform(math.pi * 0.75, math.pi)
        
        # Asignamos la equivalencia del radian a movimiento de rueda del e-puck
        wheel_turn_rads = angle * 2.5
        direction_label = 'derecha' if direction == 1 else 'izquierda'

        print(
            f"-> Esquivando (esquina/pared): Rotando ~{angle*180/math.pi:.0f}° a la {direction_label} "
            f"(Izq: {left_val:.1f}, Der: {right_val:.1f})"
        )
        # Importante: check_obstacle=False para que no cancele este giro evasivo mientras sigue cerca de la pared.
        return self.turn_steps(wheel_turn_rads, direction, check_obstacle=False)
