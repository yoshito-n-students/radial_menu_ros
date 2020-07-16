int32 STATE_CLOSED = 0
int32 STATE_OPENED = 1

std_msgs/Header header
string title
int32 state
string[] items
int32 pointed_id # negative value means no item is pointed
int32[] selected_ids