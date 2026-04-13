"""Ruedas controller."""

from robot import EpuckRobot, RobotState

def main():
    my_robot = EpuckRobot()

    # Linea recta
    my_robot.move_steps(target_rads=10.0)

    # Girar a la izq
    my_robot.turn_steps(target_rads=10.0, direction='izquierda')

    # Movimiento en cuadrado
    my_robot.move_square()

    # Ejemplo del ciruclo
    my_robot.move_circle(radius_steps=15)

    # Otros movimientos

    # Girar a la der
    # my_robot.turn_steps(target_rads=10.0, direction='derecha')

    # Curva a la izq
    # my_robot.curve_left(speed_factor=0.5)

    # Curva a la der
    # my_robot.curve_right(speed_factor=0.5)
    
    # Bucle que mantiene la sesión activa
    while my_robot.step():
        pass


if __name__ == "__main__":
    main()