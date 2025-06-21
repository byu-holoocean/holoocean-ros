# DesiredCommand message
# Contains header and data for the command

std_msgs/Header header

# For Depth: desired depth in meters (Capped based on vehicle model)
# For Heading: desired heading in degrees (-180 to 180) NWU frame
# For Speed: desired RPM value of motor (Capped based on vehicle model or parameter)

float64 data