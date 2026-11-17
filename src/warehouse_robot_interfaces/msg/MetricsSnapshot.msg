builtin_interfaces/Time stamp
float64 sim_time
bool running
bool estop
uint32 orders_total
uint32 orders_completed
uint32 orders_pending
uint32 orders_processing
uint32 orders_late
uint32 orders_failed
float64 orders_per_minute
float64 avg_completion_time
uint32 collisions
uint32 near_misses
uint32 nav_failures
uint32 pickup_failures
uint32 delivery_failures
uint32 deadlock_preventions
uint32 recoveries
string[] robot_ids
float64[] utilization       # 0..1 fraction of time not IDLE
float64[] idle_time         # seconds
uint32[] tasks_completed
float64[] distance_travelled
float64[] avg_task_time
