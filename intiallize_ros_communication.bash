# Initialize a ros2 parameter bridge for each thruster
ros2 run ros_gz_bridge parameter_bridge /beaumont/for_star_top@std_msgs/msg/Int32@gz.msgs.Int32 & disown
ros2 run ros_gz_bridge parameter_bridge /beaumont/for_star_bot@std_msgs/msg/Int32@gz.msgs.Int32 & disown
ros2 run ros_gz_bridge parameter_bridge /beaumont/for_port_top@std_msgs/msg/Int32@gz.msgs.Int32 & disown
ros2 run ros_gz_bridge parameter_bridge /beaumont/for_port_bot@std_msgs/msg/Int32@gz.msgs.Int32 & disown
ros2 run ros_gz_bridge parameter_bridge /beaumont/aft_star_top@std_msgs/msg/Int32@gz.msgs.Int32 & disown
ros2 run ros_gz_bridge parameter_bridge /beaumont/aft_star_bot@std_msgs/msg/Int32@gz.msgs.Int32 & disown
ros2 run ros_gz_bridge parameter_bridge /beaumont/aft_port_top@std_msgs/msg/Int32@gz.msgs.Int32 & disown
ros2 run ros_gz_bridge parameter_bridge /beaumont/aft_port_bot@std_msgs/msg/Int32@gz.msgs.Int32 & disown

# Initialize a ros2 parameter bridge for the claws
ros2 run ros_gz_bridge parameter_bridge /Beaumont/claws/cmd_vel@std_msgs/msg/Float64@gz.msgs.Double & disown
