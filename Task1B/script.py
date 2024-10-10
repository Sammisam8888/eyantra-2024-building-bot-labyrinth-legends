import numpy as np
import control



def sysCall_init():
    main_function();   
    global bot_body, left_joint, right_joint, K, setpoint
    # Get handles for the objects in the scene
    bot_body = sim.getObjectHandle('bot_body')
    left_joint = sim.getObjectHandle('left_joint')
    right_joint = sim.getObjectHandle('right_joint')

    # Define the desired setpoint (for example, upright position)
    setpoint = 0  # Target upright position


def sysCall_actuation():
    global bot_body, left_joint, right_joint, K, setpoint

    # Get bot's current position and velocity (angle and angular velocity)
    position = sim.getObjectPosition(bot_body, -1)
    orientation = sim.getObjectOrientation(bot_body, -1)
    _, angular_velocity = sim.getObjectVelocity(bot_body)

    # Calculate error state (x1 = angular position error, x2 = angular velocity error)
    x1 = orientation[1] - setpoint  # Error in angular position (y-axis for tilt)
    x2 = angular_velocity[1]  # Error in angular velocity (y-axis for tilt velocity)


def sysCall_sensing():
    pass


def sysCall_cleanup():
    pass


