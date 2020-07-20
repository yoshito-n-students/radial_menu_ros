# radial_menu_ros

Radial menu for quick, accurate and intuitive selection from 10+ items

![](https://raw.githubusercontent.com/yoshito-n-students/radial_menu_ros/images/images/screenshot_v0_0_2.png)

## Tested environments
* ROS Kinetic on Ubuntu 16.04
* ROS Melodic on Ubuntu 18.04
* ROS Noetic on Ubuntu 20.04

## Pkg: radial_menu_backend
### Node: example_backend
* Updates radial menu state based on joystick input
* Fork this node and add codes to integrate with your system

#### <u>Subscribed topics</u>
**joy** (sensor_msgs/Joy)

#### <u>Published topics</u>
**radial_menu_state** (radial_menu_msgs/State)

#### <u>Parameters</u>
**~allow_multi_selection** (bool, default: false)
* If true, current selected items will be deselected when a new item is selected

**~deselect_on_opening** (bool, default: false)
* If true, current selected items will be deselected when opening the menu

**~deselect_on_closing** (bool, default: false)
* If true, current selected items will be deselected when closing the menu

**~auto_select** (bool, default: false)
* If true, the last pointed item will be autonomously selected

**~open_button** (int, default: 1)
* Button to keep opening the menu
* defaultly PS4's open button

**~select_button** (int, default: 11)
* Button to select an item
* defaultly PS4's L3 button

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
* Shows the menu when the menu is being opened

### Rviz plugin: HorizontalMenu
* Visualizes subscribed menu states as a single-lined menu 
* Always shows the menu

## Pkg: radial_menu_msgs
* Defines [State](radial_menu_msgs/msg/State.msg) message type

## Pkg: radial_menu
* A meta-package depending radial_menu_backend, radial_menu_rviz, radial_menu_msgs for future release

## Pkg: radial_menu_example
* Provides a [full example](radial_menu_example/launch/example_full.launch) which requires a joystick and a [Rviz frontend example](radial_menu_example/launch/example_rviz.launch) which does not