FROM lucasmogsan/orbslam3_ros:latest
LABEL maintainer="subcat2077@gmail.com"
COPY CMakeLists.txt ./src/orb_slam3_ros/CMakeLists.txt
COPY ros_mono_inertial.cc ./src/orb_slam3_ros/src/ros_mono_inertial.cc
COPY ros_mono.cc ./src/orb_slam3_ros/src/ros_mono.cc
RUN catkin build
ADD allan_variance_ros ./src/allan_variance_ros
RUN catkin build allan_variance_ros
ADD ROS-TCP-Endpoint ./src/ROS-TCP-Endpoint
RUN catkin build ros_tcp_endpoint
COPY pi.yaml ./src/orb_slam3_ros/config/Monocular/pi.yaml
COPY pi.launch ./src/orb_slam3_ros/launch/pi.launch
COPY decompress_node.py /decompress_node.py
COPY orbslam3_init.sh /orbslam3_init.sh
