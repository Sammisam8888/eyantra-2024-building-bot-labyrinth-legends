import sympy as sp
import numpy as np
import control

def sysCall_init():
    sim = require('sim')
    global left_motor, right_motor, bot_base, K, Prismatic_joint, arm_joint
    bot_base = sim.getObject('/body')
    left_motor = sim.getObject('/body/left_joint')
    right_motor = sim.getObject('/body/right_joint')
    
    # Initialize the Prismatic_joint and arm_joint handles
    Prismatic_joint = sim.getObject('/body/Prismatic_joint')  # Replace with actual joint path
    arm_joint = sim.getObject('/body/arm_joint')  # Replace with actual arm joint path
    
    # Compute the LQR gain with the finalized A and B matrices
    K = compute_lqr_gain(A, B)

# Define symbolic variables
x1, x2, u = sp.symbols('x1 x2 u')

# Define the differential equations
x1_dot = -x1 + 2 * x1 ** 3 + x2 + 4 * u
x2_dot = -x1 - x2 + 2 * u

# Initialization of equations
eq1 = sp.Eq(x1_dot, 0)
eq2 = sp.Eq(x2_dot, 0)

def find_equilibrium_points():
    # Substitute u = 0 in the differential equations
    eq1_sub = eq1.subs(u, 0)
    eq2_sub = eq2.subs(u, 0)

    # Solve the system of equations for x1 and x2
    equi_points = sp.solve([eq1_sub, eq2_sub], (x1, x2))
    return equi_points

equilibrium_points = find_equilibrium_points()

def find_A_B_matrices(eq_points):
    A_matrix = sp.Matrix([
        [sp.diff(x1_dot, x1), sp.diff(x1_dot, x2)],
        [sp.diff(x2_dot, x1), sp.diff(x2_dot, x2)]
    ])

    B_matrix = sp.Matrix([
        [sp.diff(x1_dot, u)],
        [sp.diff(x2_dot, u)]
    ])

    A_matrices, B_matrices = [], []

    for point in eq_points:
        A_sub = A_matrix.subs({x1: point[0], x2: point[1]})
        B_sub = B_matrix.subs({x1: point[0], x2: point[1]})
        A_matrices.append(A_sub)
        B_matrices.append(B_sub)

    return A_matrices, B_matrices

A_matrices, B_matrices = find_A_B_matrices(equilibrium_points)

def find_eigen_values(A_matrices):
    eigen_values = []
    stability = []

    for a in A_matrices:
        eigval = a.eigenvals()
        eigen_values.append(eigval)
        
        stable = True
        for eig in eigval.keys():
            if eig.as_real_imag()[0] >= 0:
                stable = False
                break
        
        stability.append('Stable' if stable else 'Unstable')

    return eigen_values, stability

eigen_values, stability = find_eigen_values(A_matrices)

# System parameters
M_total = 0.364
R = 1.05
C = 1.01
I_total = 0.00216
COM_x = -40.033
g = 9.81

# Numerical A matrix based on the given parameters
A = np.array([
    [0, 1, 0, 0],
    [0, -C / M_total, (M_total * g * COM_x) / (M_total * R), 0],
    [0, 0, 0, 1],
    [0, -(C * COM_x) / I_total, (M_total * g * COM_x**2) / I_total, 0]
], dtype=np.float64)

# Numerical B matrix based on the given parameters
B = np.array([
    [0],
    [ (M_total * R)],
    [0],
    [COM_x / I_total]
], dtype=np.float64)

# Compute LQR gain with adjusted weighting
def compute_lqr_gain(A, B):
    # Q matrix as a 4x4 square matrix, with large first two values (10000 and 15000)
    Q = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, -100000, 0],
        [0, 0, 0, -1000000]
    ])  # The last two values are smaller (1) for tilt and velocity
    
    R = np.array([[0.001]])
    K, _, _ = control.lqr(A, B, Q, R)
    
    return K

# Adjusted apply_lqr_control function with scaling and velocity limits
def apply_lqr_control(curr_tilt, curr_position):
    global K
    state_vector = np.array([curr_position, 0, curr_tilt, 0])
    
    # Apply scaling factor to the control input
    lqr_control_input = -0.5 * np.dot(K, state_vector)  # Scaling factor to reduce control input magnitude

    # Limit motor velocities to avoid skidding
    left_motor_velocity = np.clip(lqr_control_input[0], -2.0, 2.0)  # Adjust limit as needed
    right_motor_velocity = np.clip(lqr_control_input[0], -2.0, 2.0)
    
    sim.setJointTargetVelocity(left_motor, left_motor_velocity*10)
    sim.setJointTargetVelocity(right_motor, right_motor_velocity*10)

def sysCall_actuation():
    # Get current tilt and position of the bot
    curr_tilt = sim.getObjectOrientation(bot_base, -1)[0]
    curr_position = sim.getObjectPosition(bot_base, -1)[1]
    apply_lqr_control(curr_tilt, curr_position)
    
def sysCall_sensing():
    ############### Keyboard Input ##############
    message, data, data2 = sim.getSimulatorMessage()

    if message == sim.message_keypress:
        if data[0] == 2007:  # forward (up arrow)
            sim.setJointTargetVelocity(left_motor, 100)
            sim.setJointTargetVelocity(right_motor, 100)

        elif data[0] == 2008:  # backward (down arrow)
            sim.setJointTargetVelocity(left_motor, -100)
            sim.setJointTargetVelocity(right_motor, -100)

        elif data[0] == 2009:  # left (left arrow)
            sim.setJointTargetVelocity(left_motor, -100)
            sim.setJointTargetVelocity(right_motor, 100)

        elif data[0] == 2010:  # right (right arrow)
            sim.setJointTargetVelocity(left_motor, 100)
            sim.setJointTargetVelocity(right_motor, -100)

        # 'q' and 'e' keys for adjusting the Prismatic_joint velocity
        elif data[0] == 113:  # 'q' key
            sim.setJointTargetVelocity(Prismatic_joint, -0.1)
        elif data[0] == 101:  # 'e' key
            sim.setJointTargetVelocity(Prismatic_joint, 0.1)
        
        # 'w' and 's' keys for controlling the arm_joint velocity
        elif data[0] == 119:  # 'w' key
            sim.setJointTargetVelocity(arm_joint, 100)  # Arm moves forward
        elif data[0] == 115:  # 's' key
            sim.setJointTargetVelocity(arm_joint, -100)  # Arm moves backward
        
        # If no arrow keys are pressed, apply LQR control
        else:
            # Get current tilt and position of the bot
            curr_tilt = sim.getObjectOrientation(bot_base, -1)[0]
            curr_position = sim.getObjectPosition(bot_base, -1)[1]
            apply_lqr_control(curr_tilt, curr_position)

    else:
        # Stop the motors if no key is pressed (both prismatic and arm joints)
        sim.setJointTargetVelocity(Prismatic_joint, 0)
        sim.setJointTargetVelocity(arm_joint, 0)

        # Stop the robot motors if no key input
        sim.setJointTargetVelocity(left_motor, 100)
        sim.setJointTargetVelocity(right_motor, 100)
    #########################################

def sysCall_cleanup():
    sim.setJointTargetVelocity(left_motor, 0)
    sim.setJointTargetVelocity(right_motor, 0)
