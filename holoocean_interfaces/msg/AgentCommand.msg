# Agent Command message
# Contains header and agent command

# Frame ID in the header is connection to the agent that commands are sent to
std_msgs/Header header

# Command is a list that matches the length requried by the control scheme for the vehicle
# Different control schemes have lists of different lengths. 
# 
float64[] command
