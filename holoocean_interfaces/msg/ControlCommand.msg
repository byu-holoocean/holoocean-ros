# Control message
# Contains header and U_control command

# Frame ID in the header is connection to the agent that commands are sent to or recieved from
std_msgs/Header header

# Fin commands angles RADIANS (-deltaMin, deltaMax)
# Positive deflection will pitch up and yaw to starboard side
# Thruster commands (0, maxRPM) with 0 as off

# Array for all control surfaces. Will change length depending on the vehicle and number of control surfaces
float64[] cs
