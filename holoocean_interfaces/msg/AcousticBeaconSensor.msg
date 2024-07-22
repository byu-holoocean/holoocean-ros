# AcousticBeaconSensor message
# Contains timestamp, message type, beacon information, azimuth, elevation, range, and z-coordinate

# Timestamp in nanoseconds
int64 timestamp

# Message type
string msg_type

# Beacon ID
int32 from_beacon

# Azimuth in radians
float32 azimuth

# Elevation in radians
float32 elevation

# Range in meters
float32 range

# Z-coordinate in meters
float32 z
