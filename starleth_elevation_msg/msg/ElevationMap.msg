# Header (time and frame)
Header header

# Map resolution in xy plane [m/cell].
float64 resolution

# Map length in x-direction [m].
float64 lengthInX
  
# Map width in y-direction [m].
float64 lengthInY

# Elevation height data [m].
std_msgs/Float64MultiArray elevation

# Variance data [m].
std_msgs/Float64MultiArray variance
std_msgs/Float64MultiArray varianceX
std_msgs/Float64MultiArray varianceY

# Color data [rgb] (optional).
std_msgs/Int32MultiArray color