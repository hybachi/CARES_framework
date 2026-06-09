# SwarmStatus.msg
# Provides distributed swarm awareness to allow dynamic adaptation

std_msgs/Header header      # message synchronization
string robot_id             # robot agent broadcasting
string robot_type           # e.g. UGV, UAV, etc
Capability[] capabilities   # current robot capabilities