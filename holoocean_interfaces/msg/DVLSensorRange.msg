# DVLSensor message
# Contains header, velocity, and range measurements

std_msgs/Header header

# Range measurements in meters from the 4 sonar beams
float32[4] range
