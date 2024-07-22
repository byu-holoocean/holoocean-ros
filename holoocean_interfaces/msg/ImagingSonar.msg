# ImagingSonar message
# Contains timestamp, bins information, and image data

# Timestamp in nanoseconds
int64 timestamp

# Number of azimuth bins
int32 bins_azimuth

# Number of range bins
int32 bins_range

# Image data separated into two lists
# Could make it one list by flattening array with numpy
float32[] image_azimuth
float32[] image_range