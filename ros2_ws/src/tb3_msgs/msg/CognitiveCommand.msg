std_msgs/Header header
string action       # navigate | explore | search | approach | stop | report
string target       # kitchen | bottle | person | living_room | ...
float32 confidence  # 0.0 - 1.0
string raw_text     # texto original del usuario
