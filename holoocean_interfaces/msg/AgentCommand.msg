# Agent Command message
# Contains header and agent command

# Frame ID in the header is connection to the agent that commands are sent to
std_msgs/Header header

# FOR STANDARD AGENTS:
# Command is a list that matches the length requried by the control scheme for the vehicle
# Different control schemes have lists of different lengths. 
# 
# FOR FOSSEN AGENTS:
# Fin commands angles RADIANS (-deltaMin, deltaMax)
# Positive deflection will pitch up and yaw to starboard side
# Thruster commands (0, maxRPM) with 0 as off
# Array for all control surfaces. Will change length depending on the vehicle and number of control surfaces

float64[] command
