## Stability Checker 
Using Pinocchio C++ and ROS2, this `stability_checker` node subscribes to the `/joint_states` topic to calculate the robot's Center of Mass.

To run the node:
```
ros2 run stability_checker stability_checker --ros-args -p urdf_file_path:='<path to urdf>' 
```

Note that it will not work if supplied a .xacro file that requires arguments. Use `xacro` to build the full URDF file and link it to that.