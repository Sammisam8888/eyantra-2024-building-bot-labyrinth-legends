import sympy as sp
import numpy as np
import control

########## Initialization of variables & provided  
# Define the symbolic variables
x1, x2, u = sp.symbols('x1 x2 u')

# Define the differential equations
x1_dot = sp.symbols('x1_dot')
x2_dot = sp.symbols('x2_dot')
##################################################

# Initialization of equation
eq1 = sp.Eq(x1_dot, -x1 + 2 * x1 ** 3 + x2 + 4 * u)
eq2 = sp.Eq(x2_dot, -x1 - x2 + 2 * u)


def find_equilibrium_points():
    '''
    1. Substitute input(u) = 0 in both equation for finding equilibrium points 
    2. Equate x1_dot, x2_dot equal to zero for finding equilibrium points 
    3. Solve the x1_dot, x2_dot equations for the unknown variables and save the value to the variable namely "equi_points"
    '''
    
    eq1_sub = eq1.subs(u, 0)
    eq2_sub = eq2.subs(u, 0)

    eq11 = sp.Eq(eq1_sub, 0)
    eq21 = sp.Eq(eq2_sub, 0)
    
    equi_points = sp.solve([eq11, eq21], (x1, x2))

    return equi_points


def find_A_B_matrices(eq_points):
    '''
    1. Substitute every equilibrium points that you have already found in the find_equilibrium_points() function 
    2. After substituting the equilibrium points, Save the Jacobian matrices A and B as A_matrices, B_matrices  
    '''
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
        A_sub = A_matrix.subs({x1: point[x1], x2: point[x2]})
        B_sub = B_matrix.subs({x1: point[x1], x2: point[x2]})
        A_matrices.append(A_sub)
        B_matrices.append(B_sub)

    return A_matrices, B_matrices


def find_eigen_values(A_matrices):
    '''
    1. Find the eigen values of all A_matrices (You can use the eigenvals() function of sympy) 
       and append it to the 'eigen_values' list
    2. With the eigen values, determine whether the system is stable or not and
       append the string 'Stable' if system is stable, else append the string 'Unstable'
       to the 'stability' list 
    '''
    eigen_values = []
    stability = []

    for a in A_matrices:
        eigval = a.eigenvals()
        eigen_values.append(eigval)
        
        stable = True
        for eig in eigval.keys():
            if eig.as_real_imag()[0] >= 0:  # eig.as_real_imag() converts (real+imaginary) to (real, imaginary)
                stable = False
                break
        
        if stable:
            stability.append('Stable')
        else:
            stability.append('Unstable')

    return eigen_values, stability


def compute_lqr_gain(jacobians_A, jacobians_B):
    K = 0
    '''
    This function is used to compute the LQR gain matrix K
    1. Use the Jacobian A and B matrix at the equilibrium point (-1,1) and assign it to A and B respectively for computing LQR gain
    2. Compute the LQR gain of the given system equation (You can use lqr() of control module) 
    3. Take the A matrix corresponding to the Unstable Equilibrium point (-1,1) that you have already found for computing LQR gain.
    4. Assign the value of gain to the variable K
    '''

    # Define the Q and R matrices
    Q = np.eye(2)  # State weighting matrix
    R = np.array([1])  # Control weighting matrix

    A = jacobians_A[0]
    B = jacobians_B[0]

    npA = np.array(A).astype(np.float64)
    npB = np.array(B).astype(np.float64)
    K, _, _ = control.lqr(npA, npB, Q, R)

    return K


def main_function():  # Don't change anything in this function 
    # Find equilibrium points
    eq_points = find_equilibrium_points()
    
    if not eq_points:
        print("No equilibrium points found.")
        return [],[],[],[]
    
    # Find Jacobian matrices
    jacobians_A, jacobians_B = find_A_B_matrices(eq_points)
    
    # For finding eigenvalues and stability of the given equation
    eigen_values, stability = find_eigen_values(jacobians_A)
    
    # Compute the LQR gain matrix K
    K = compute_lqr_gain(jacobians_A, jacobians_B)
    
    return eq_points, jacobians_A, eigen_values, stability, K


def task1a_output():
    '''
    This function will print the results that you have obtained 
    '''
    print("Equilibrium Points:")
    for i, point in enumerate(eq_points):
        print(f"  Point {i + 1}: x1 = {point[0]}, x2 = {point[1]}")
    
    print("\nJacobian Matrices at Equilibrium Points:")
    for i, matrix in enumerate(jacobians_A):
        print(f"  At Point {i + 1}:")
        print(sp.pretty(matrix, use_unicode=True))
    
    print("\nEigenvalues at Equilibrium Points:")
    for i, eigvals in enumerate(eigen_values):
        eigvals_str = ', '.join([f"{val}: {count}" for val, count in eigvals.items()])
        print(f"  At Point {i + 1}: {eigvals_str}")
    
    print("\nStability of Equilibrium Points:")
    for i, status in enumerate(stability):
        print(f"  At Point {i + 1}: {status}")
    
    print("\nLQR Gain Matrix K at the selected Equilibrium Point:")
    print(K)


if __name__ == "__main__":
    # Run the main function
   results = main_function()
if results is None:
    print("No equilibrium points found. Exiting.")
    exit()
else:
    eq_points, jacobians_A, eigen_values, stability, K = results
    # print the results
    task1a_output()