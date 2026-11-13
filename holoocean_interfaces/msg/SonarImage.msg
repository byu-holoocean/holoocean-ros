# SonarImage message
# Contains header, bins information, and SONAR data

std_msgs/Header header

# Number of azimuth bins
int32 bins_azimuth

# Number of range bins
int32 bins_range

# Range min and max (meters)
float32 range_min
float32 range_max

# Horizontal field of view (degrees)
float32 azimuth

# Vertical field of view (degrees)
float32 elevation

# Array stored as a 1d matrix unravelled on a row by row basis
# data[r * bins_azimuth + a], r = range bin index, a = azimuth bin index
float32[] data
