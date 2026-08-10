# The fixed-logic perception output. This is the only view of the world the
# LLM ever gets -- it never sees the video frames themselves.

std_msgs/Header header
string camera_frame
bool calibration_ok
SceneObject[] objects
