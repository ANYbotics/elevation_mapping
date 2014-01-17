# Header (time and frame)
Header header

# Map resolution in xy plane [m/cell].
float32 resolution

# Map length in x-direction [m].
float32 lengthInX
  
# Map width in y-direction [m].
float32 lengthInY

# Elevation height data [m].
std_msgs/Float32MultiArray elevation

# Variance data [m].
std_msgs/Float32MultiArray variance

# Color data [rgb] (optional).
std_msgs/Int32MultiArray color

# Row start index (default 0).
uint16 outerStartIndex

# Column start index (default 0).
uint16 innerStartIndex