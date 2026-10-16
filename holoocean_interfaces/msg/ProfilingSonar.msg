# ProfilingSonar message
# Contains timestamp, bins information, and SONAR data

# Timestamp in nanoseconds
std_msgs/Header header

# Number of azimuth bins
int32 bins_azimuth

# Number of range bins
int32 bins_range

# Range min and max
float32 range_min
float32 range_max

# Azimuth angle
float32 azimuth

# Elevation angle
float32 elevation

# Array stored as a 1d matrix unravelled on a row by row basis
float32[] data