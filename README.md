# radial_menu_ros

Radial menu on ROS1 for quick, accurate and intuitive selection by a joystick from 10+ items

![](https://raw.githubusercontent.com/yoshito-n-students/radial_menu_ros/images/images/screenshot_sub_menu_v0_1_0.png)

## Tested environments
* ROS Kinetic on Ubuntu 16.04
* ROS Melodic on Ubuntu 18.04
* ROS Noetic on Ubuntu 20.04

## Pkg: radial_menu_backend
### Nodelet: Backend
* Updates radial menu state based on joystick input
* To integrate with your system, subscribe both joy and menu state messages using [message_filters::TimeSynchronizer](http://wiki.ros.org/message_filters#Time_Synchronizer), watch which menu items are selected and process the joy messages when the menu is disabled (i.e. the menu does not own the joy messages). See [example_integration](radial_menu_example/src/example_integration.cpp).

#### <u>Subscribed topics</u>
**joy** (sensor_msgs/Joy)

#### <u>Published topics</u>
**radial_menu_state** ([radial_menu_msgs/State](radial_menu_msgs/msg/State.msg))
* The stamp in a message is copied from the source joy message

#### <u>Parameters</u>
**~menu** (struct, required)
* Tree structure of the menu like below
```YAML
# A simple example
menu:
    MoveCmd:     # title
        - Front  # items
        - Left
        - Back
        - Right
```
```YAML
# A complex example
menu:
    Reboot:           # title
        - Base:       # items
            - Wheels  # subitems
            - Cameras:
                - Front
                - Front Left
                - Rear
                - Front Right
        - Arm:
            - Joints
            - ...
```

**~allow_multi_selection** (bool, default: false)
* If false, current selected items will be deselected when a new item is selected

**~reset_on_enabling** (bool, default: false)
* If true, current selected items will be deselected when enebling the menu

**~reset_on_disabling** (bool, default: false)
* If true, current selected items will be deselected when disabling the menu

**~auto_select** (bool, default: false)
* If true, the last pointed item will be autonomously selected. Close the menu before unpoint not to select.

**~enable_button** (int, default: 1)
* Button to keep enabling the menu
* defaultly PS4's circle button

**~select_button** (int, default: 5)
* Button to select an item
* defaultly PS4's R1 button

**~ascend_button** (int, default: 4)
* Button to ascend the menu
* defaultly PS4's L1 button

**~pointing_axis_v** (int, default: 1)
* Vertical axis to point an item
* defaultly PS4's LEFT Y axis

**~pointing_axis_h** (int, default: 0)
* Horizontal axis to point an item
* defaultly PS4's LEFT X axis

**~invert_pointing_axis_v** (bool, default: false)

**~invert_pointing_axis_h** (bool, default: false)

**~pointing_axis_threshold** (double, default: 0.5)
* Threshold value of axis input to enable pointing

## Pkg: radial_menu_rviz
### Rviz plugin: RadialMenu
* Visualizes subscribed menu states as a radial menu
* Shows the menu when the menu is being enabled

### Rviz plugin: HorizontalMenu
* Visualizes subscribed menu states as a single-lined menu 
* Always shows the menu

## Pkg: radial_menu_msgs
* Defines [State](radial_menu_msgs/msg/State.msg) message type

## Pkg: radial_menu
* A meta-package depending radial_menu_backend, radial_menu_rviz, radial_menu_msgs for future release

## Pkg: radial_menu_example
* Provides a [full example](radial_menu_example/launch/example_full.launch) which requires a joystick and a [Rviz frontend example](radial_menu_example/launch/example_rviz.launch) which does not