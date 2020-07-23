std_msgs/Header header
string title
bool is_enabled
string[] items
int32[] widths        # widths[depth] indicates number of items at that depth.
                      # An empty array means depth of all items is zero.
int32 pointed_id      # An invalid id means no item is pointed
int32[] selected_ids