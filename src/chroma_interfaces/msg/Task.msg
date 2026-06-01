# Task.msg
# Decouples mission generation from specific robot assignments

string task_id                      # task identifier
string type                         # e.g. SEARCH, DELIVERY
float32 priority                    # weight for bidding
string[] required_capabilities      # eligibility criteria
float32 min_capability_score        # minimum acceptable health
geometry_msgs/Point location        # task execution coordinate (specific to DELIVERY)
geometry_msgs/Point[] target_area   # task execution area (specific to SEARCH)