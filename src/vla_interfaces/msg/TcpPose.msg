# One timestamped selected-TCP sample published by vla_robot.
#
# vla_robot uses its own non-blocking controller service client so the sampling
# timer never enters the synchronous DSR_ROBOT2 API used by the motion worker.
# The wrist node pairs these samples with RGB-D frames to transform a moving
# camera observation into the robot base frame. The controller service has no
# acquisition stamp, so header.stamp is the request/response midpoint; samples
# with an excessive round-trip time are discarded.

std_msgs/Header header
string request_id          # tracking session this sample belongs to
float64[6] posx            # x, y, z millimetres + Doosan ZYZ Euler degrees
string tcp_name            # must match the hand-eye calibration TCP
bool valid
