# header: sequence and time-step id
# rail: rail mid_line comprising of two 3D points
# pose: robot pose comprising of [x,y,z,roll,pitch,yaw]
# twist: robot twist comprising of [vx,vy,vz,p,q,r] (p,q,r in body coordinate frame)
# mode: desired driving mode: 0 - manual, 1 - auto with tracks, 2 - auto with treads
# inrange: rail is within valid range of vehicle to go into autonomous mode
std_msgs/Header header
acri_localisation/railLine rail
geometry_msgs/Pose pose
geometry_msgs/Twist twist
std_msgs/UInt32 mode
std_msgs/Bool inrange