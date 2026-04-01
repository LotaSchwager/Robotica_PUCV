"""Ruedas controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor

def switcher(option):
    match option:
        # Línea rectal 
        case 'Q1':
            left_velocity = -0.5 * MAX_SPEED
            right_velocity = 0.5 * MAX_SPEED
            print("left_velocity: ", left_velocity)
            print("right_velocity: ", right_velocity,"\n\n")
            left_motor.setVelocity(left_velocity)
            right_motor.setVelocity(right_velocity)
            robot.step(1000)
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
        # Girar en su eje    
        case 'Q2':
            left_velocity = -0.5 * MAX_SPEED
            right_velocity = 0.5 * MAX_SPEED
            print("left_velocity: ", left_velocity)
            print("right_velocity: ", right_velocity, "\n\n")
            left_motor.setVelocity(left_velocity)
            right_motor.setVelocity(right_velocity)
        # Girar hacia una dirección
        case 'Q3':
            left_velocity = 0.5 * MAX_SPEED
            right_velocity = 0.25 * MAX_SPEED
            print("left_velocity: ", left_velocity)
            print("right_velocity: ", right_velocity, "\n\n")
            left_motor.setVelocity(left_velocity)
            right_motor.setVelocity(right_velocity)
        # En caso de no calzar
        case _:
            print("Opción no válida\n\n")


MAX_SPEED = 6.25
robot = Robot()
timestep = int(robot.getBasicTimeStep())
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    switcher('Q1')
    pass