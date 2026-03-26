# Movement priority constants — used by collision avoidance
int32 PRIORITY_IDLE             = 0  # not moving with intent
int32 PRIORITY_TRAVELLING_EMPTY = 1  # en route to pick something up
int32 PRIORITY_LOADED           = 2  # carrying an item to drop-off
int32 PRIORITY_RETURNING_CHARGE = 3  # returning to charger (highest)

string robot_id
string task_status        # idle | running | error
string task_type          # pickup | delivery | ''
string task_detail
int32  queue_size
int32  movement_priority  # see constants above
