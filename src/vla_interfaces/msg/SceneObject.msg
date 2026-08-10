# One tracked object exactly as the LLM will see it.
#
# `id` is the conversational handle ("apple_17"). It is what the LLM writes
# back in a RobotAction, so it must stay stable while the object is visible.

string id
int64 track_id
string class_name
float32 confidence

string color
float32 color_confidence

# Image-plane box, kept so the GUI can crop this object out of the annotated
# frame when the LLM asks the user which one it means.
float32 x_min
float32 y_min
float32 x_max
float32 y_max

# False when depth was missing or out of range for this object. Such objects
# are still reported so the LLM knows they exist, but they can not be picked.
bool position_valid
geometry_msgs/Point position_base
float32 depth_m
