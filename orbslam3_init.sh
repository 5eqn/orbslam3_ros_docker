python /decompress_node.py &
python /imu_node.py &
python /image_server.py &
python /angle_server.py &
roslaunch ros_tcp_endpoint endpoint.launch
