# UCommand message
# Contains header and U_control command

std_msgs/Header header

# Fin commands angles degrees (-90, 90) set value to 360 if fin not in use
# Positive deflection will pitch up and yaw to starboard side
float64[4] fin

# Thruster commands (-100, 100) with 0 as off
int64 thruster