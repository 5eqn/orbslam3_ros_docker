FROM lucasmogsan/orbslam3_ros:latest
LABEL maintainer="subcat2077@gmail.com"
COPY CMakeLists.txt ./src/orb_slam3_ros/CMakeLists.txt
COPY ros_mono_inertial.cc ./src/orb_slam3_ros/src/ros_mono_inertial.cc
RUN catkin build
