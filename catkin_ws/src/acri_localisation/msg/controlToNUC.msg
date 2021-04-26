# header: sequence and time-step id
# mode: desired driving mode: 0 - manual, 1 - deploying, 2 - deployed 3 - isolation deploying 4- isolation deployed 5- fault
# voltage24: voltage of 24V battery
# voltage48: voltage of 48V battery
std_msgs/Header header
std_msgs/UInt32 mode
std_msgs/Float32 voltage24
std_msgs/Float32 voltage48