# One grasp the wrist camera worked out, ready for the arm to execute.
#
# The poses are already Doosan `posx` -- translation in **mm**, orientation as
# **ZYZ Euler degrees** -- and `target_posx` is already the *contact* point, i.e.
# the GraspGenX gripper-base pose advanced by the gripper's 180 mm fingertip
# offset along its own +Z. Publishing the finished command keeps one conversion
# in one place: the alternative, shipping a pose and converting again here, is
# how a 180 mm miss gets introduced twice.
#
# `success` false means the arm must not move. `reason` says why, so the agent
# can tell the user something better than "failed".
# On success, header.stamp is the source RGB-D exposure time and frame_id is
# "base"; it is deliberately not the later GraspGenX completion time.

std_msgs/Header header
string request_id
string object_id
bool success
string reason             # empty on success; human-readable on failure

float64[6] target_posx    # contact pose: x,y,z mm + rx,ry,rz ZYZ degrees
float64[6] pregrasp_posx  # backed off along the approach axis by standoff
float32 confidence        # GraspGenX discriminator score for this grasp
float32 approach_tilt_deg # 0 = straight down; the gate that stands in for
                          # collision checking until a planner exists
int32 candidate_count     # how many grasps survived filtering
int32 cloud_points        # points fed to GraspGenX, for diagnosing thin masks
