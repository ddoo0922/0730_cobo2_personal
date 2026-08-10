# The executor asking the wrist camera to track and plan a grasp for one object.
#
# Request/response rather than a stream because GraspGenX takes about 1.6 s per
# object on this machine. Running it continuously is not possible, and running it
# speculatively would waste the GPU the webcam detector is sharing.
#
# The start request is published *before* the arm approaches. While it moves,
# vla_robot publishes request-correlated TcpPose samples through an independent
# async controller client. vla_wrist pairs each RGB-D observation with the
# nearest pose and keeps looking until the webcam target is confirmed in base
# coordinates. `cancel` ends that search without publishing a failure plan.

std_msgs/Header header
string request_id
string object_id          # the agent's handle, echoed back for correlation
string class_name         # what the webcam detector called it
geometry_msgs/Point expected_base   # where the webcam thinks it is, metres
float64[6] tcp_posx       # initial TCP sample: mm + ZYZ degrees
string tcp_name
bool cancel
