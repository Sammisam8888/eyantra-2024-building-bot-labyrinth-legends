import math

def sysCall_init():
    global bot_base, left_motor, right_motor, Prismatic_joint, arm_joint
    global Kp_rot, Ki_rot, Kd_rot, Kp_trans, Ki_trans, Kd_trans
    global setPt_rot, setPt_trans
    global P_err_rot, I_err_rot, D_err_rot, prevP_err_rot
    global P_err_trans, I_err_trans, D_err_trans, prevP_err_trans
    global curr_Angle, curr_Position
    global translation_velocity, rotation_velocity
    global prevturning, turning
    global count
    count=0
    sim = require('sim')

    # Initialize robot components
    bot_base = sim.getObject('/body')
    left_motor = sim.getObject('/left_joint')
    right_motor = sim.getObject('/right_joint')
    Prismatic_joint = sim.getObject('/Prismatic_joint')  # Replace with actual path
    arm_joint = sim.getObject('/arm_joint')  # Replace with actual path

    # Initialize PID control parameters for rotation
    Kp_rot = 300
    Ki_rot = 10
    Kd_rot = 2000.0

    # Initialize PID control parameters for translation
    Kp_trans = 100
    Ki_trans = 0.05
    Kd_trans = 2500.0

    # Initialize set points
    setPt_rot = 0.0
    setPt_trans = 0.0

    # Initialize PID error variables
    P_err_rot = I_err_rot = D_err_rot = prevP_err_rot = 0.0
    P_err_trans = I_err_trans = D_err_trans = prevP_err_trans = 0.0

    # Initialize position and velocity variables
    curr_Angle = 0.0
    curr_Position = 0.0
    translation_velocity = 0.0
    rotation_velocity = 0.0

    # Initialize turning variables
    turning = 0
    prevturning = 0

def sysCall_actuation():
    global left_motor, right_motor, setPt_rot, curr_Angle, P_err_rot, I_err_rot, D_err_rot, prevP_err_rot
    global Kp_rot, Ki_rot, Kd_rot, P_err_trans, I_err_trans, D_err_trans, prevP_err_trans, curr_Position
    global translation_velocity, rotation_velocity, Kp_trans, Ki_trans, Kd_trans

    # PID control for rotation and translation
    K_rot = Kp_rot * P_err_rot + Ki_rot * I_err_rot + Kd_rot * D_err_rot
    K_trans = Kp_trans * P_err_trans + Ki_trans * I_err_trans + Kd_trans * D_err_trans

    # Calculate motor velocities
    left_motor_velocity = setPt_rot - K_rot + K_trans + translation_velocity - rotation_velocity
    right_motor_velocity = setPt_rot - K_rot + K_trans + translation_velocity + rotation_velocity

    # Set motor velocities
    sim.setJointTargetVelocity(left_motor, left_motor_velocity)
    sim.setJointTargetVelocity(right_motor, right_motor_velocity)
def sysCall_sensing():
    global bot_base, curr_Angle, curr_Position
    global P_err_rot, I_err_rot, D_err_rot, prevP_err_rot
    global P_err_trans, I_err_trans, D_err_trans, prevP_err_trans
    global setPt_rot, setPt_trans, translation_velocity, rotation_velocity
    global turning, prevturning
    global count
    count +=1
    print("timer = ",count)
    # Read current orientation and position
    curr_Angle = sim.getObjectOrientation(bot_base, -1)[0]
    curr_Position = sim.getObjectPosition(bot_base, -1)[1]

    # Update PID errors for rotation
    P_err_rot = curr_Angle - setPt_rot
    I_err_rot += P_err_rot
    D_err_rot = P_err_rot - prevP_err_rot
    prevP_err_rot = P_err_rot

    # Update PID errors for translation
    P_err_trans = curr_Position - setPt_trans
    I_err_trans += P_err_trans
    D_err_trans = P_err_trans - prevP_err_trans
    prevP_err_trans = P_err_trans

    # Handle user input for set point changes
    message, data, _ = sim.getSimulatorMessage()
    turning = 0  # Reset turning to 0 by default
    if message == sim.message_keypress:
        if data[0] == 2007:  # Increase translation set point
            translation_velocity -= 2
        elif data[0] == 2008:  # Decrease translation set point
            translation_velocity += 1.5
        elif data[0] == 2009:  # Rotate left
            rotation_velocity = -2.0
            turning = 1
        elif data[0] == 2010:  # Rotate right
            rotation_velocity = 2.0
            turning = 1
        elif data[0] == 113:  # 'q' key to decrease Prismatic_joint velocity
            sim.setJointTargetVelocity(Prismatic_joint, -0.05)
            #print(sim.getJointVelocity(Prismatic_joint))
        elif data[0] == 101:  # 'e' key to increase Prismatic_joint velocity
            sim.setJointTargetVelocity(Prismatic_joint, 0.05)
        elif data[0] == 119 and sim.getJointVelocity(arm_joint)>= -4.1279057949632285e-08:  # 'w' key to move arm joint forward
            sim.setJointTargetVelocity(arm_joint, 1)
            
        elif data[0] == 115 and sim.getJointVelocity(arm_joint)<= 1.1287718886334449e-06 :  # 's' key to move arm joint backward
            sim.setJointTargetVelocity(arm_joint, -1)
        elif data[0] == 32 :
            translation_velocity +=0.5
        else:
            stopmotors()
    else:
        # Reset rotation velocity for forward motion
        rotation_velocity = 0
        turning = 0
        sim.setJointTargetVelocity(Prismatic_joint, 0)
        sim.setJointTargetVelocity(arm_joint, 0)

    # Update prevturning to reflect the current turning state
    prevturning = turning

def stopmotors():
    global prevturning, turning, translation_velocity, rotation_velocity
    global arm_joint,Prismatic_joint

    if prevturning:
        # Stop motors if the last command was turning
        sim.setJointTargetVelocity(left_motor, 0)
        sim.setJointTargetVelocity(right_motor, 0)
    else:
        # Default forward motion when no explicit stop command
        translation_velocity = 0  # Set a default forward speed
        rotation_velocity = 0
        sim.setJointTargetVelocity(left_motor, translation_velocity - rotation_velocity)
        sim.setJointTargetVelocity(right_motor, translation_velocity + rotation_velocity)

def sysCall_cleanup():
    pass
 
