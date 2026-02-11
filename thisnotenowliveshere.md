ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 --dev 192.168.8.210


ros2 launch combat_strategizer letsgo.launch.py


ros2 topic pub /weapon/armed std_msgs/Bool "{data: true}"
ros2 topic pub /weapon/armed std_msgs/Bool "{data: false}"