def sysCall_init():
    sim = require('sim')

    # Initialize the scene objects
    global bot_body, left_joint, right_joint
    
    try:
        # Get object handles for the bot's components
        bot_body = sim.getObject('/body')  # Adjust this path based on your scene
        left_joint = sim.getObject('/left_joint')  # Adjust this path based on your scene
        right_joint = sim.getObject('/right_joint')  # Adjust this path based on your scene
        
        # Debugging messages
        sim.addStatusbarMessage(f"bot_body handle: {bot_body}, left_joint handle: {left_joint}, right_joint handle: {right_joint}")
        
    except Exception as e:
        sim.addStatusbarMessage("Error accessing objects: " + str(e))
        return  # Exit if objects are not found

    # Initialize algorithm-related variables
    global desired_position, current_angle, previous_error, integral, kp, ki, kd
    desired_position = 0.0  # Target upright position for the bot (no tilt)
    current_angle = 0.0
    previous_error = 0.0
    integral = 0.0

    # Adjusted PID coefficients for better balancing
    kp = 0.009824600508967166  # Increase proportional gain to react more to errors
    ki = 4.723273606010673e-05  # Increase integral gain to eliminate steady-state errors
    kd = 7.984421807863955e-06   # Increase derivative gain to dampen oscillations

def sysCall_actuation():
    # This function is executed at each simulation time step
    global bot_body, left_joint, right_joint, desired_position, previous_error, integral, kp, ki, kd

    # Get the current tilt of the bot (assuming upright Y position)
    current_angle = sim.getObjectOrientation(bot_body, -1)[1]  # Y-axis orientation (pitch)

    # Calculate error (desired position is 0, which means no tilt)
    error = desired_position - current_angle
    integral += error
    derivative = error - previous_error

    # Calculate control output (PID)
    control_output = kp * error + ki * integral + kd * derivative

    # Limit control output to avoid excessive speeds
    max_velocity = 10.0  # Adjust this value based on your bot's capabilities
    control_output = max(min(control_output, max_velocity), -max_velocity)

    # Set joint actuation based on control output (apply the same to both joints for balance)
    sim.setJointTargetVelocity(left_joint, control_output)
    sim.setJointTargetVelocity(right_joint, control_output)

    # Update previous error for the next iteration
    previous_error = error

def sysCall_sensing():
    # This function is executed at each simulation time step
    pass  # Add any additional sensing logic if required

def sysCall_cleanup():
    # This function is executed when the simulation ends
    # Any cleanup to take the scene back to its original state after simulation
    pass