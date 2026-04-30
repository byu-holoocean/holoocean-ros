# AcousticBeaconSend message

# Timestamp in nanoseconds
std_msgs/Header header

# From Beacon ID
int32 from_beacon

# To Beacon ID
int32 to_beacon

# Message type
string msg_type

# Message
uint8[] msg_data