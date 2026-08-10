# One tool call the LLM decided on, addressed to the robot executor.
#
# Only the object handle travels here, never coordinates: the executor looks
# the live position up in its own latest SceneSnapshot at the moment it starts
# moving. A position the LLM saw one API round-trip ago is already stale.

std_msgs/Header header
string action_id
string name          # pick_and_place | pick_and_hold | release
string object_id     # scene object handle; empty for release
string reason        # the LLM's one-line rationale, for the GUI and the log
