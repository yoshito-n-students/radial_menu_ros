std_msgs/Header header
string title
bool is_enabled
string[] items
int32[] widths        # widths[depth] indicates number of items at that depth.
                      # The sum of elements in widths must equal to the number of items.
int32 pointed_id      # An invalid id means no item is pointed
int32[] selected_ids