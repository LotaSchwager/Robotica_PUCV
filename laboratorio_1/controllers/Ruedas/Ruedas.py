"""Ruedas controller."""

from robot import EpuckRobot, RobotState

def main():
    my_robot = EpuckRobot()

    print("=== Iniciando control por odometría/distancia con Estados ===")
    # El robot monitorea obstáculos internamente y cambia a STOPPED si encuentra uno
    distancia_total = 100.0
    
    print(f"Intentando avanzar {distancia_total} pasos...")
    while True:
        completado = my_robot.move_steps(target_rads=distancia_total, speed_factor=0.6)
        
        if my_robot.state == RobotState.STOPPED and not completado:
            print("Robot parado por obstáculo. Cambiando el angulo de giro para intentar evitarlo.")
            
            # Girar en su propio eje con un ángulo random acotado según los sensores frontales
            # Repetir giros hasta que el camino esté despejado
            while True:
                my_robot.rotate_random()
                
                # Una vez terminado el giro, verificamos si hay obstáculos
                if not my_robot.proximity.is_obstacle_ahead(threshold=100.0):
                    print("✨ Camino despejado, retomando marcha.")
                    break
                else:
                    print("Todavía hay un obstáculo, buscando nueva dirección...")
        else:
            # Si completó la distancia sin detenerse por obstáculos, terminamos este loop
            print("Recorrido completado.")
            break

    # Ejemplo del ciruclo
    # my_robot.move_circle(radius_steps=15)
    

    # Ejemplo de giro opuesto
    # my_robot.turn_steps(target_rads=10.0, direction='derecha')

    # Bucle que mantiene la sesión activa
    while my_robot.step():
        pass


if __name__ == "__main__":
    main()