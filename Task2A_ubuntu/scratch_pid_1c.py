import math

def sysCall_init():
    global bot_base, left_motor, right_motor
    global Kp_rot, Ki_rot, Kd_rot, Kp_trans, Ki_trans, Kd_trans
    global setPt_rot, setPt_trans
    global P_err_rot, I_err_rot, D_err_rot, prevP_err_rot
    global P_err_trans, I_err_trans, D_err_trans, prevP_err_trans
    global curr_Angle, curr_Position
    global translation_velocity, rotation_velocity

    sim = require('sim')

    bot_base = sim.getObject('/body')
    left_motor = sim.getObject('/left_joint')
    right_motor = sim.getObject('/right_joint')

    Kp_rot = 500
    Ki_rot = 1.0
    Kd_rot = 100.0

    Kp_trans = 750
    Ki_trans = 2.0
    Kd_trans = 25000.0

    setPt_rot = 0.0
    setPt_trans = 0.0

    P_err_rot = 0.0
    I_err_rot = 0.0
    D_err_rot = 0.0
    prevP_err_rot = 0.0

    P_err_trans = 0.0
    I_err_trans = 0.0
    D_err_trans = 0.0
    prevP_err_trans = 0.0

    curr_Angle = 0.0
    curr_Position = 0.0

    translation_velocity = 0.0
    rotation_velocity = 0.0

def sysCall_actuation():
    global left_motor, right_motor, setPt_rot, curr_Angle, P_err_rot, I_err_rot, D_err_rot, prevP_err_rot
    global Kp_rot, Ki_rot, Kd_rot, P_err_trans, I_err_trans, D_err_trans, prevP_err_trans, curr_Position
    global translation_velocity, rotation_velocity, Kp_trans, Ki_trans, Kd_trans

    K_rot = Kp_rot * P_err_rot + Ki_rot * I_err_rot + Kd_rot * D_err_rot
    K_trans = Kp_trans * P_err_trans + Ki_trans * I_err_trans + Kd_trans * D_err_trans

    left_motor_velocity = setPt_rot - K_rot + K_trans + translation_velocity - rotation_velocity
    right_motor_velocity = setPt_rot - K_rot + K_trans + translation_velocity + rotation_velocity

    sim.setJointTargetVelocity(left_motor, left_motor_velocity)
    sim.setJointTargetVelocity(right_motor, right_motor_velocity)

def sysCall_sensing():
    global bot_base, curr_Angle, curr_Position
    global P_err_rot, I_err_rot, D_err_rot, prevP_err_rot
    global P_err_trans, I_err_trans, D_err_trans, prevP_err_trans
    global setPt_rot, setPt_trans, translation_velocity, rotation_velocity

    curr_Angle = sim.getObjectOrientation(bot_base, -1)[0]
    curr_Position = sim.getObjectPosition(bot_base, -1)[1]

    P_err_rot = curr_Angle - setPt_rot
    I_err_rot += P_err_rot
    D_err_rot = P_err_rot - prevP_err_rot
    prevP_err_rot = P_err_rot

    P_err_trans = curr_Position - setPt_trans
    I_err_trans += P_err_trans
    D_err_trans = P_err_trans - prevP_err_trans
    prevP_err_trans = P_err_trans

    message, data, _ = sim.getSimulatorMessage()

    if message == sim.message_keypress:
        if data[0] == 2007:
            setPt_trans += 0.0015
        if data[0] == 2008:
            setPt_trans -= 0.0015
        if data[0] == 2009:
            rotation_velocity = -2.0
        if data[0] == 2010:
            rotation_velocity = 10.0
    else:
        translation_velocity = 0.0
        rotation_velocity = 0.0

def sysCall_cleanup():
    pass
