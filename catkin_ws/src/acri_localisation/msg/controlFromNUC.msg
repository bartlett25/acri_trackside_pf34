# header: sequence and time-step id
# mid line: rail mid_line comprising of two 3D points
# pose2D: robot pose comprising of [x,y,theta] 
# mode: desired driving mode: 0 - manual, 1 - deploying, 2 - deployed 3 - isolation deploying 4- isolation deployed 5- fault
# inrange: rail is within valid range of vehicle to go into autonomous mode
# voltage24: voltage of 24V battery
# voltage48: voltage of 48V battery

std_msgs/Header header
acri_localisation/railLine midline
geometry_msgs/Pose2D pose2D
std_msgs/UInt32 mode
std_msgs/Bool inrange
std_msgs/Float32 voltage24
std_msgs/Float32 voltage48