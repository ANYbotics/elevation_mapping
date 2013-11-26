# Header (time and frame)
Header header

# Map resolution in xy plane [m/cell].
float64 resolution

# Map length (x-direction) [m].
float64 length
  
# Map width (y-direction) [m].
float64 width

# Elevation height data.
std_msgs/Float64MultiArray elevationData

# Variance data of the cells in elevationData.
std_msgs/Float64MultiArray varianceData