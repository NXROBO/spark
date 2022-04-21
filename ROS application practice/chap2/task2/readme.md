> ```
> 本任务无代码，调用的是ROS内部的包，指令如下
> $ roscore
> $ rosrun turtlesim turtlesim_node
> $ rostopic list
> $ rostopic echo /turtle1/pose
> 
> $ rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist '{linear: {x: 0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'
> $ rosmsg show geometry_msgs/Twist
> 
> > $ rosbag record -a -O cmd_vel_record
> > $ rosbag play cmd_vel_record.bag
> > $ rosservice call /spawn '{x: 1.0, y: 1.0, theta: 1.0, name:''}'
> 
> > $ rosrun turtlesim turtle_teleop_key
> ```
>
> 
