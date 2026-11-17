# One warehouse order. The order manager owns the authoritative record.
string order_id
string item_id
string item_name
string shelf_id
string aisle_id
string station_id
float64 arrival_time        # simulation seconds
float64 deadline            # simulation seconds
string status               # PENDING | ASSIGNED | IN_PROGRESS | COMPLETED | FAILED
string assigned_robot
float64 started_time        # 0 until a robot starts working on it
float64 completed_time      # 0 until delivered
uint8 attempts
