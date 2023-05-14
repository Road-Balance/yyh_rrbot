# yyh_rrbot

```
ros2 launch gz_rrbot_ros2 empty_world.launch.py 

ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
- 1.57
- -1.57" -1
```

```
ros2 control set_controller_state forward_position_controller stop
ros2 control unload_controller forward_position_controller

ros2 control load_controller joint_state_broadcaster
ros2 control load_controller forward_position_controller

ros2 control set_controller_state joint_state_broadcaster configure
ros2 control set_controller_state forward_position_controller configure

ros2 control set_controller_state joint_state_broadcaster start
ros2 control set_controller_state forward_position_controller start

ros2 control list_controllers

```