# Header (time and frame)
Header header

# Map resolution in xy plane [m/cell].
float64 resolution

# Map length in x-direction [m].
float64 lengthInX
  
# Map width in y-direction [m].
float64 lengthInY

# Elevation height data.
std_msgs/Float64MultiArray elevation

# Variance data of the cells in elevationData.
std_msgs/Float64MultiArray variance