results = main_function()
if results is None:
    print("No equilibrium points found. Exiting.")
    exit()
else:
    eq_points, jacobians_A, eigen_values, stability, K = results
    # print the results
    task1a_output()