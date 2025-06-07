# ros2 control

```
ros2 topic pub -1 /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{ data: [1.57] }"
```

```
ros2 topic pub -1 /velocity_controller/commands std_msgs/msg/Float64MultiArray "{ data: [0.2] }"
```

```
gz topic -e -t /world/my_world/pose/info
```

```
                                  gz service -s /echo \
                                    --reqtype gz.msgs.StringMsg \
                                    --reptype gz.msgs.StringMsg \
                                    --timeout 2000 \
                                    --req 'data: "Hello"'
```



```
gz service --timeout 10000 -s /world/my_world/set_pose \
--reptype gz.msgs.Boolean \
--reqtype gz.msgs.Pose \
--req 'name: "green_simple_box" position {
  x: 5
  y: 0
  z: 0.5
}'
```

```
ros2 service call /world/my_world/set_pose ros_gz_interfaces/srv/SetEntityPose \
"{entity: {name: 'green_simple_box'}, \
pose: {position: {x: 10.0, y: 0.0, z: 0.5}, 
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

```

```
ros2 service call /world/my_world/set_pose ros_gz_interfaces/srv/SetEntityPose \
"{entity: {name: 'green_simple_box'}, \
pose: {position: {x: 5.0} 
}}"

```


### Gimbal

```bash title="terminal1"
source env.sh
ros2 launch two_axis_description effort_control.launch.py
```

```bash title="terminal2"
source env.sh
ros2 run tutorial_application my_control.py
```

```bash title="terminal3"
source env.sh
ros2 topic pub -1 /set_point std_msgs/msg/Float32 "{ data: -1.0 }"
```

```bash title="terminal1"
source env.sh
rqt
```