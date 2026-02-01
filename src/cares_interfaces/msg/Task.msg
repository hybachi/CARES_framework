string task_id
string type                 # "SEARCH", "RESCUE", "SCAN"
geometry_msgs/Point location
float32 priority            # 1.0 = High, 0.1 = Low
string[] required_capabilities
float32 min_capability_score