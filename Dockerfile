FROM lucasmogsan/orbslam3_ros:latest
LABEL maintainer="subcat2077@gmail.com"

USER root
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
    --no-upgrade \
    --no-install-recommends \
    -o Acquire::Retries=8 \
    python3-scipy \
    ipython3 \
    python3-wxgtk4.0 \
    python3-tk \
    python3-igraph \
    python3-pyx \
    libsuitesparse-dev \
    doxygen \
    libv4l-dev && \
    rm -rf /var/lib/apt/lists/*
USER aut
ADD kalibr ./src/kalibr
RUN catkin config --extend /opt/ros/noetic
RUN catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN catkin build kalibr

ADD allan_variance_ros ./src/allan_variance_ros
RUN catkin build allan_variance_ros

ADD ROS-TCP-Endpoint ./src/ROS-TCP-Endpoint
RUN catkin build ros_tcp_endpoint

COPY CMakeLists.txt ./src/orb_slam3_ros/CMakeLists.txt
COPY GetTrackingData.srv ./src/orb_slam3_ros/srv/GetTrackingData.srv
COPY common.h ./src/orb_slam3_ros/include/common.h
COPY common.cc ./src/orb_slam3_ros/src/common.cc
COPY ros_mono_inertial.cc ./src/orb_slam3_ros/src/ros_mono_inertial.cc
COPY ros_mono.cc ./src/orb_slam3_ros/src/ros_mono.cc
COPY orb_slam3 ./src/orb_slam3_ros/orb_slam3
RUN catkin build orb_slam3_ros

COPY pi.yaml ./src/orb_slam3_ros/config/Monocular/pi.yaml
COPY pi.launch ./src/orb_slam3_ros/launch/pi.launch
COPY decompress_node.py /decompress_node.py
COPY imu_node.py /imu_node.py
COPY image_server.py /image_server.py
COPY angle_server.py /angle_server.py
COPY entrypoint.sh /entrypoint.sh
COPY orbslam3_init.sh /orbslam3_init.sh
COPY ubt_msgs_py ./devel/lib/python3/dist-packages/ubt_msgs
COPY ubt_msgs ./devel/share/ubt_msgs
