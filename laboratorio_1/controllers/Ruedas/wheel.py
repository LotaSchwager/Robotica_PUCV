class WheelController:
    MAX_SPEED = 6.25

    def __init__(self, robot, timestep):
        self.left_motor = robot.getDevice("left wheel motor")
        self.right_motor = robot.getDevice("right wheel motor")
        
        # Para medir "pasos" obtenidos por los motores, requerimos los encoders o "sensores de posición"
        self.left_sensor = robot.getDevice("left wheel sensor")
        self.right_sensor = robot.getDevice("right wheel sensor")
        self.left_sensor.enable(timestep)
        self.right_sensor.enable(timestep)
        
        # Seteamos la posición requerida en infinito
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        
        self.stop()

    def get_positions(self):
        """Devuelve cuántos radianes ha girado cada rueda: (izquierda, derecha)"""
        return self.left_sensor.getValue(), self.right_sensor.getValue()

    def set_velocities(self, left, right):
        """Método base para asignar velocidades."""
        self.left_motor.setVelocity(left)
        self.right_motor.setVelocity(right)

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
