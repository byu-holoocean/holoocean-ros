# AcousticBeaconSensor message

# Timestamp in nanoseconds
std_msgs/Header header

# Message type
string msg_type

# From Beacon ID
int32 from_beacon

# To Beacon ID
int32 to_beacon

# payload
uint8[] msg_data

# Azimuth in radians
float32 azimuth

# Elevation in radians
float32 elevation

# Range in meters
float32 range

# Z-coordinate in meters
float32 z
