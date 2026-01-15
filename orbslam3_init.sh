python /decompress_node.py &
python /imu_node.py &
python /image_server.py &
python /angle_server.py &
python /param_server.py &
python /pose_server.py &
roslaunch ros_tcp_endpoint endpoint.launch &
sleep 1
roslaunch orb_slam3_ros pi.launch
