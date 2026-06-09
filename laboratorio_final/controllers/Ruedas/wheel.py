import math


class WheelController:
    MAX_SPEED = 6.25

    def __init__(self, robot, timestep):
        self.left_motor = robot.getDevice("left wheel motor")
        self.right_motor = robot.getDevice("right wheel motor")
        self.left_sensor = robot.getDevice("left wheel sensor")
        self.right_sensor = robot.getDevice("right wheel sensor")
        self.left_sensor.enable(timestep)
        self.right_sensor.enable(timestep)
        
        # Seteamos la posición requerida en infinito
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

        self.last_left_velocity = 0.0
        self.last_right_velocity = 0.0
        
        self.stop()

    def get_positions(self):
        """Devuelve cuántos radianes ha girado cada rueda: (izquierda, derecha)"""
        return self.left_sensor.getValue(), self.right_sensor.getValue()

    def set_velocities(self, left, right):
        """Método base para asignar velocidades."""
        # Asegura modo control por velocidad (si veníamos de setPosition()).
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.last_left_velocity = float(left)
        self.last_right_velocity = float(right)
        self.left_motor.setVelocity(left)
        self.right_motor.setVelocity(right)

    def set_position_targets(self, left_target_rad: float, right_target_rad: float, max_speed_rad_s: float) -> None:
        """Control por posición: ordena a cada rueda alcanzar un ángulo específico en radianes."""
        left_curr, right_curr = self.get_positions()
        left_target = float(left_target_rad)
        right_target = float(right_target_rad)
        vmax = abs(float(max_speed_rad_s))

        self.last_left_velocity = math.copysign(vmax, left_target - float(left_curr))
        self.last_right_velocity = math.copysign(vmax, right_target - float(right_curr))

        self.left_motor.setVelocity(vmax)
        self.right_motor.setVelocity(vmax)
        self.left_motor.setPosition(left_target)
        self.right_motor.setPosition(right_target)

    def get_last_velocities(self):
        return self.last_left_velocity, self.last_right_velocity

    def forward(self, speed_factor=0.5):
        speed = speed_factor * self.MAX_SPEED
        self.set_velocities(speed, speed)

    def backward(self, speed_factor=0.5):
        speed = -speed_factor * self.MAX_SPEED
        self.set_velocities(speed, speed)

    def stop(self):
        self.set_velocities(0.0, 0.0)

    def turn_own_axis_left(self, speed_factor=0.5):
        speed = speed_factor * self.MAX_SPEED
        self.set_velocities(-speed, speed)

    def turn_own_axis_right(self, speed_factor=0.5):
        speed = speed_factor * self.MAX_SPEED
        self.set_velocities(speed, -speed)

    def curve_left(self, speed_factor=0.5):
        self.set_velocities(speed_factor * 0.5 * self.MAX_SPEED, speed_factor * self.MAX_SPEED)

    def curve_right(self, speed_factor=0.5):
        self.set_velocities(speed_factor * self.MAX_SPEED, speed_factor * 0.5 * self.MAX_SPEED)
