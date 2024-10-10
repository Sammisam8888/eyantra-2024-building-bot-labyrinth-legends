
def sysCall_init():
    sim = require('sim')

    # Initialize the scene objects
    global bot_body, left_joint, right_joint
    
    try:
        # Get object handles for the bot's components
        bot_body = sim.getObject('\body')  # Adjust this path based on your scene
        left_joint = sim.getObject('\left_joint')  # Adjust this path based on your scene
        right_joint = sim.getObject('\right_joint')  # Adjust this path based on your scene
        
        # Debugging messages
        sim.addStatusbarMessage(f"bot_body handle: {bot_body}, left_joint handle: {left_joint}, right_joint handle: {right_joint}")
        
    except Exception as e:
        sim.addStatusbarMessage("Error accessing objects: " + str(e))
        return  # Exit if objects are not found

    # Initialize algorithm-related variables
    global desired_position, current_angle, previous_error, integral, kp, ki, kd
    desired_position = 0.0  # Target position for the bot
    current_angle = 0.0
    previous_error = 0.0
    integral = 0.0

    # PID coefficients
    kp = 1.0
    ki = 0.1
    kd = 0.05

def sysCall_actuation():
    # This function is executed at each simulation time step
    global bot_body, left_joint, right_joint, desired_position, previous_error, integral, kp, ki, kd

    # Get the current angle of the bot (assuming upright Z position)
    current_angle = sim.getObjectPosition(bot_body, -1)[1]

    # Calculate error
    error = desired_position - current_angle
    integral += error
    derivative = error - previous_error

    # Calculate control output (PID)
    control_output = kp * error + ki * integral + kd * derivative

    # Set joint actuation based on control output
    sim.setJointTargetVelocity(left_joint, control_output)
    sim.setJointTargetVelocity(right_joint, control_output)

    # Update previous error
    previous_error = error

def sysCall_sensing():
    # This function is executed at each simulation time step
    pass  # Add any additional sensing logic if required

def sysCall_cleanup():
    # This function is executed when the simulation ends
    # Any cleanup to take the scene back to its original state after simulation
    pass