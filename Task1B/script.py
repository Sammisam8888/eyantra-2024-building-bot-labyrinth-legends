import numpy as np
import sympy as sp
import control


# Define the symbolic variables
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

    for i, point in enumerate(eq_points, 1):
        A_sub = A_matrix.subs({x1: point[0], x2: point[1]})
        B_sub = B_matrix.subs({x1: point[0], x2: point[1]})
        A_matrices.append(A_sub)
        B_matrices.append(B_sub)

    return A_matrices, B_matrices


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

        if stable:
            stability.append('Stable')
        else:
            stability.append('Unstable')

    return eigen_values, stability


def compute_lqr_gain(jacobians_A, jacobians_B):
    Q = np.eye(2)
    R = np.array([1])

    A = jacobians_A[0]
    B = jacobians_B[0]

    npA = np.array(A).astype(np.float64)
    npB = np.array(B).astype(np.float64)
    K, _, _ = control.lqr(npA, npB, Q, R)

    return K


def main_function():
    eq_points = find_equilibrium_points()
    if not eq_points:
        print("No equilibrium points found.")
        return [], [], [], [], None

    jacobians_A, jacobians_B = find_A_B_matrices(eq_points)
    eigen_values, stability = find_eigen_values(jacobians_A)
    K = compute_lqr_gain(jacobians_A, jacobians_B)

    return eq_points, jacobians_A, eigen_values, stability, K


def sysCall_init():
    main_function();   
    global bot_body, left_joint, right_joint, K, setpoint
    # Get handles for the objects in the scene
    bot_body = sim.getObjectHandle('bot_body')
    left_joint = sim.getObjectHandle('left_joint')
    right_joint = sim.getObjectHandle('right_joint')

    # Define the desired setpoint (for example, upright position)
    setpoint = 0  # Target upright position

    # Use task1a.py to compute the LQR gain matrix
    eq_points, jacobians_A, _, _, K = main_function()  # Main function from task1a.py
    K = compute_lqr_gain(jacobians_A, jacobians_A)  # Compute LQR gain matrix


def sysCall_actuation():
    global bot_body, left_joint, right_joint, K, setpoint

    # Get bot's current position and velocity (angle and angular velocity)
    position = sim.getObjectPosition(bot_body, -1)
    orientation = sim.getObjectOrientation(bot_body, -1)
    _, angular_velocity = sim.getObjectVelocity(bot_body)

    # Calculate error state (x1 = angular position error, x2 = angular velocity error)
    x1 = orientation[1] - setpoint  # Error in angular position (y-axis for tilt)
    x2 = angular_velocity[1]  # Error in angular velocity (y-axis for tilt velocity)

    # Pack the state vector (x1, x2) for the control law
    state_vector = np.array([x1, x2])

    # Calculate the control input using LQR gain matrix K
    control_input = -np.dot(K, state_vector)

    # Apply control input to the joints
    sim.setJointTargetVelocity(left_joint, control_input)
    sim.setJointTargetVelocity(right_joint, control_input)


def sysCall_sensing():
    pass


def sysCall_cleanup():
    pass


if __name__ == "__main__":
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

    if clientID != -1:
        print('Connected to CoppeliaSim')
        sysCall_init()
        while (true):
            sysCall_actuation()
        

        # Disconnect from the simulator
        sim.simxFinish(clientID)
    else:
        print('Failed connecting to CoppeliaSim')
    

