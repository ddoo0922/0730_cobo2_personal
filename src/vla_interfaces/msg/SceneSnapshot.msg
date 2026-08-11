# The fixed-logic perception output. This is the only view of the world the
# LLM ever gets -- it never sees the video frames themselves.

std_msgs/Header header
string camera_frame
bool calibration_ok
SceneObject[] objects
# Resolution of the frame objects[].x_min/y_min/x_max/y_max are expressed in.
# 0 means "not known yet" (no frame processed since startup) -- needed so a
# pixel handed to cobot2_ws's select_by_point() (vla-bridge-contract.md #2)
# carries its own reference size instead of assuming one.
uint32 image_width
uint32 image_height
