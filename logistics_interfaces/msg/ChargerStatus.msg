string waypoint_id       # charging waypoint this charger serves
bool   engaged           # WM has requested engagement
bool   robot_detected    # charger sensor sees a robot present
bool   charging          # current is flowing (hardware confirmed)
float32 current_ma       # charge current in milliamps (0 when not charging)
float32 voltage_v        # charge voltage (0 when not charging)
