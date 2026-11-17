builtin_interfaces/Time stamp
string robot_id
string state                # IDLE ASSIGNED NAVIGATING_TO_SHELF WAITING_FOR_AISLE POSITIONING_FOR_PICKUP ...
string order_id
string item_id
string nav_status           # IDLE | NAVIGATING | SUCCEEDED | FAILED | RECOVERING
string arm_status           # STOWED | MOVING | PICKING | PLACING | HOLDING | FAULT
string gripper_status       # OPEN | HOLDING | EMPTY_CLOSED | UNKNOWN
string destination
bool has_goal
float64 goal_x
float64 goal_y
float64 goal_yaw
float64 x                   # ground truth from the simulator
float64 y
float64 yaw
float64 speed
float64 battery             # estimate, percent
float64 distance_travelled
uint32 tasks_completed
uint32 tasks_failed
uint32 queue_length
string[] queued_orders
string[] held_resources
string waiting_for
bool healthy
float32[] path_xy           # flattened (x0,y0,x1,y1,...) global plan, sub-sampled
