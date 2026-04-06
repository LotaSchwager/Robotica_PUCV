class ProximityController:
    # El e-puck tiene 8 sensores de proximidad llamados 'ps0' a 'ps7'
    NUM_SENSORS = 8

    def __init__(self, robot, timestep):
        self.sensors = []
        for i in range(self.NUM_SENSORS):
            sensor_name = f'ps{i}'
            sensor = robot.getDevice(sensor_name)
            sensor.enable(timestep)
            self.sensors.append(sensor)

    def get_values(self):
        """Retorna una lista con los valores actuales de los 8 sensores."""
        return [sensor.getValue() for sensor in self.sensors]

    def is_obstacle_ahead(self, threshold=80.0):
        """
        Detecta si hay un obstáculo enfrente.
        En el e-puck:
        - ps0 y ps7 apuntan hacia enfrente.
        - ps1 y ps6 apuntan oblicuamente hacia adelante.
        Los valores aumentan cuando el objeto está más cerca.
        """
        values = self.get_values()
        # Verificamos los dos sensores frontales
        return values[0] > threshold or values[7] > threshold

    def front_obstacle_hits(self, threshold=80.0):
        """Devuelve qué sensores frontales detectan obstáculo."""
        values = self.get_values()
        return values[0] > threshold, values[7] > threshold

    def front_obstacle_count(self, threshold=80.0):
        """Devuelve cuántos sensores frontales están activos."""
        left_hit, right_hit = self.front_obstacle_hits(threshold=threshold)
        return int(left_hit) + int(right_hit)
