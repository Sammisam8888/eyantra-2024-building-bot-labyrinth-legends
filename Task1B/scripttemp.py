from random import random as r

def sysCall_init():
    sim = require('sim')

    global bot_body, left_joint, right_joint
    
    try:
        bot_body = sim.getObject('/body')
        left_joint = sim.getObject('/left_joint')
        right_joint = sim.getObject('/right_joint')
        sim.addStatusbarMessage(f"bot_body handle: {bot_body}, left_joint handle: {left_joint}, right_joint handle: {right_joint}")
        
    except Exception as e:
        sim.addStatusbarMessage("Error accessing objects: " + str(e))
        return

    global desired_position, current_angle, previous_error, integral, kp, ki, kd, min_error, check_kp, check_ki, check_kd
    desired_position = 0.0
    current_angle = 0.0
    previous_error = 0.0
    integral = 0.0
    min_error = float('inf')
    check_kp = 1
    check_kd = 1
    check_ki = 1

    kp = 0.0  
    kd = 0.0  
    ki = 0.0

def sysCall_actuation():
    global bot_body, left_joint, right_joint, desired_position, previous_error, integral, kp, ki, kd, min_error, check_kp, check_ki, check_kd
    
    if check_kp:
        kp += r() # Finer adjustment for kp
    elif check_ki:
        ki += r() * 0.00001  # Finer adjustment for ki
    elif check_kd:
        kd += r() * 0.000001  # Finer adjustment for kd
    else:
        return None
    
    current_angle = sim.getObjectOrientation(bot_body, -1)[1]  # Y-axis orientation (pitch)
    
    error = desired_position - current_angle
    integral += error
    derivative = error - previous_error

    control_output = kp * error + ki * integral + kd * derivative

    max_velocity = 10.0
    control_output = max(min(control_output, max_velocity), -max_velocity)

    sim.setJointTargetVelocity(left_joint, control_output)
    sim.setJointTargetVelocity(right_joint, control_output)

    previous_error = error
    
    if abs(error) < min_error:
        min_error = abs(error)

    # Fine-tuned thresholds for PID parameter adjustments
    if min_error < 0.1:  # Adjust kp until error is less than 0.1
        check_kp = 0
    if min_error < 0.01:  # Adjust ki once the error is less than 0.01
        check_ki = 0 
        check_kp = 0  # Ensure kp is not adjusted further once ki starts tuning
    if min_error < 0.001:  # Adjust kd only when error is extremely small
        check_kd = 0
        check_ki = 0
        check_kp = 0

    print(f"kp: {kp}, ki: {ki}, kd: {kd}, min_error={min_error}")

def sysCall_sensing():
    pass

def sysCall_cleanup():
    print(f"kp: {kp}, ki: {ki}, kd: {kd}, min_error={min_error}")
    pass
