# Header (time and frame)
Header header

# Resolution in xy plane [m/cell].
float32 resolution

# Length in x-direction [m].
float32 lengthInX
  
# Width in y-direction [m].
float32 lengthInY

# Position of the elevation map in the parent frame [m].
geometry_msgs/Point position

# Row start index (default 0).
uint16 outerStartIndex

# Column start index (default 0).
uint16 innerStartIndex

# Elevation means [m].
std_msgs/Float32MultiArray elevation

# Elevation variances [m].
std_msgs/Float32MultiArray variance

# Colors [rgb] (optional).
std_msgs/Int32MultiArray color

# Surface normals x-component [-].
std_msgs/Float32MultiArray surfaceNormalXComponent

# Surface normals y-component [-].
std_msgs/Float32MultiArray surfaceNormalYComponent

# Surface normals z-component [-].
std_msgs/Float32MultiArray surfaceNormalZComponent