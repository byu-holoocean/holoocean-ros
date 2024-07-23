# UCommand message
# Contains header and U_control command

std_msgs/Header header

# Fin commands angles degrees (-90, 90) set value to 360 if fin not in use
float64[4] fin

# Thruster commands (0-100)
int64 thruster