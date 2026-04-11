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
            if self.proximity.is_obstacle_ahead(threshold=90.0):
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
    
    def backward_steps(self, target_rads, speed_factor=0.5):
        """
        Ordena mover el robot en línea recta hacia atrás por 'target_rads' pasos, ´para salir del bucle.
        """
        self.state = RobotState.RUNNING
        start_left, start_right = self.wheels.get_positions()
        
        print(f"-> [ESTADO: {self.state.value}] Instrucción recibida: retroceder por {target_rads} pasos.")
        self.wheels.backward(speed_factor)
        
        while self.step():
            curr_left, curr_right = self.wheels.get_positions()
            
            # Promediamos la distancia recorrida de ambas ruedas
            dist_travelled = (abs(curr_left - start_left) + abs(curr_right - start_right)) / 2.0
            
            if dist_travelled >= target_rads:
                print(f"Retroceso completado ({target_rads} pasos).")
                self.stop()
                return True
                
        return False

    def stop(self):
        """Detiene el robot y cambia su estado a STOPPED."""
        self.wheels.stop()
        self.state = RobotState.STOPPED
        print(f"-> [ESTADO: {self.state.value}] Robot detenido.")

    def turn_steps(self, target_rads, direction='izquierda', speed_factor=0.5):
        """
        Gira en su propio eje cierta cantidad de pasos (radianes). 
        También chequea obstáculos en pleno giro.
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
            if self.proximity.is_obstacle_ahead(threshold=100.0):
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
        """Gira el robot con un ángulo aleatorio acotado por p0/p7."""
        values = self.proximity.get_values()
        p0_active = values[0] > 100.0
        p7_active = values[7] > 100.0

        if p7_active and not p0_active:
            # p7 ve obstáculo a la izquierda, así que conviene girar a la derecha.
            direction = 1
            angle = random.uniform(math.pi * 0.5, math.pi * 0.75)
        elif p0_active and not p7_active:
            # p0 ve obstáculo a la derecha, así que conviene girar a la izquierda.
            direction = -1
            angle = random.uniform(math.pi * 0.5, math.pi * 0.75)
        elif p0_active and p7_active:
            # Si ambos ven obstáculo, usamos el sensor más fuerte para decidir el lado.
            if values[7] > values[0]:
                direction = 1
            elif values[0] > values[7]:
                direction = -1
            else:
                direction = random.choice([-1, 1])
            angle = random.uniform(math.pi * 0.75, math.pi)
        else:
            # Si cualquiera de los dos sensores no ven obstaculo, girar en dirección aleatoria moderada.
            direction = random.choice([-1, 1])
            angle = random.uniform(math.pi * 0.4, math.pi * 0.65)

        wheel_turn_rads = angle * 2.5 # Valor de calibración ficticio
        direction_label = 'derecha' if direction == 1 else 'izquierda'

        print(
            f"-> Rotando ángulo acotado: {angle:.2f} rad, dirección: {direction_label} "
            f"(p0={values[0]:.2f}, p7={values[7]:.2f})"
        )
        return self.turn_steps(wheel_turn_rads, direction)

    def get_position(self):
        return self.gps.getValues() # Devuelve [X, Y, Z]