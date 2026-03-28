builtin_interfaces/Time stamp

uint8 status

# enum
uint8 IDLE=0
uint8 MOVING=1
uint8 SUCCEEDED=2
uint8 FAILED=3
uint8 BLOCKED=4
uint8 RECOVERING=5

float32 distance_to_goal
float32 remaining_path_length

float32 navigation_time
float32 estimated_time_remaining

geometry_msgs/PoseStamped current_pose

# orchesetra
bool has_goal
bool is_stuck

geometry_msgs/PoseStamped current_goal
 

# debugging / extensibility
string detail