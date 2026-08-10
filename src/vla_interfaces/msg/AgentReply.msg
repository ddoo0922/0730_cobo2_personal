# What the agent wants to say to the user.

std_msgs/Header header
string kind                  # say | ask_clarification | error | status
string text

# Set by ask_clarification: the GUI crops these objects out of the annotated
# frame and numbers them, so "1번" is a usable answer.
string[] focus_object_ids
