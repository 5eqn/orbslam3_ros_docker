# ORB-SLAM3 ROS Docker

## Usage

```sh
./build.sh
docker compose up
```

## Introduction (zh-cn)

为了实现Yanshee机器人在未知环境中的自主定位与导航，本研究集成了基于视觉的即时定位与地图构建系统。考虑到机器人平台原有的软件生态系统基于较为早期的ROS1（Robot Operating System）版本，而现代SLAM算法依赖的诸多库（如OpenCV、Eigen、Pangolin等）版本迭代迅速，极易产生依赖冲突，传统的手动编译配置方法变得异常困难且不可重现。因此，本研究创新性地采用基于Docker的微服务架构来部署三维建图模块，将复杂的SLAM系统及其全部依赖环境封装在独立的容器中，确保了系统的一致性与可移植性。

本模块的核心是ORB-SLAM3算法，这是一个支持单目、双目、RGB-D以及单目惯性（Mono-Inertial）等多种传感器模式的先进视觉SLAM系统。在本项目中，我们将利用机器人头部的单目摄像头和惯性测量单元数据，运行其单目惯性模式。该模式融合了视觉特征点与IMU的角速度、加速度信息，能够在快速运动、纹理缺失或纯旋转等对纯视觉SLAM极具挑战的场景下，提供鲁棒且尺度一致的位置与姿态估计，并实时生成稀疏的三维环境地图。

然而，ORB-SLAM3的官方ROS封装版本（orb_slam3_ros）对ROS1、OpenCV（特定版本，如3.x或4.x）以及其他第三方库（如DBoW2, g2o, Pangolin）有着极为严格的依赖要求。在2025年的当下，试图在现代操作系统上直接为古旧的ROS1 Kinetic/Melodic版本手动编译配置这套工具链，会遭遇大量的库版本不兼容、头文件缺失和编译错误问题，构建过程几乎无法成功。Docker容器技术完美地解决了这一困境。我们基于一个预先配置好的基础镜像（lucasmogsan/orbslam3_ros）构建自定义镜像。该基础镜像已经包含了正确版本的ROS、OpenCV、Eigen、Pangolin等全部依赖。在Dockerfile中，我们只需替换特定的ROS节点源代码（ros_mono_inertial.cc）和构建配置文件（CMakeLists.txt），然后执行catkin build即可完成项目定制化编译，整个过程完全可复现且与环境隔离。

系统架构通过docker-compose.yml文件定义。我们创建了两个主要服务：orbslam3服务作为ROS主节点，运行roscore，并提供ORB-SLAM3算法的执行环境；kalibr服务则基于另一个官方镜像，为未来可能的相机-IMU联合标定任务预留。这两个服务通过一个自定义的Docker网络（ros-network）互联，确保它们能够使用ROS的TCP通信协议发现彼此并进行数据交换。通过将ROS Master的端口（11311）映射到宿主机，Unity仿真环境或机器人上的其他ROS节点可以轻松地作为外部客户端，向容器内的ORB-SLAM3节点发布图像/IMU话题并订阅其输出的相机位姿和地图点话题。这种设计不仅彻底解决了依赖地狱问题，还使得SLAM模块成为一个独立、可插拔的微服务，极大地简化了系统集成、部署和后续维护的复杂度。





## 三维建图模块关键组件

### 1. 核心算法架构

#### ORB-SLAM3 单目惯性模式
```
数据流：单目图像 + IMU数据 → 特征提取与匹配 → 视觉-惯性紧耦合优化 → 实时位姿与稀疏地图
```
- **前端特征处理**：使用ORB特征检测器提取图像关键点，生成旋转和尺度不变的特征描述子
- **视觉-惯性紧耦合**：采用最大后验概率估计，联合优化视觉重投影误差和IMU预积分误差
- **多地图系统**：支持地图合并与重定位，具备长期运行鲁棒性
- **四种并行线程**：
  - Tracking线程：实时处理每帧图像，估算相机位姿
  - Local Mapping线程：局部地图构建与优化
  - Loop Closing线程：回环检测与全局优化
  - Full BA线程：完整的束调整优化

### 2. Docker容器化架构

#### 2.1 镜像分层结构
```
Layer 4: Application Layer (本项目的自定义配置)
    ├── ros_mono_inertial.cc (ROS节点主程序)
    └── CMakeLists.txt (构建配置文件)
    
Layer 3: ORB-SLAM3 ROS Wrapper
    ├── ORB_SLAM3核心库接口封装
    └── ROS消息转换与发布

Layer 2: ORB-SLAM3 Core
    ├── Thirdparty依赖 (DBoW2, g2o)
    ├── 特征提取与匹配引擎
    └── 优化与地图管理系统

Layer 1: Base Environment
    ├── ROS1 Noetic/Kinetic
    ├── OpenCV 3.4/4.5
    ├── Eigen 3.3
    └── Pangolin可视化库
```

#### 2.2 网络通信架构
```
+-------------------+      +-------------------+      +-------------------+
|   Unity仿真环境   |      |   Docker容器网络  |      |   宿主机/其他节点  |
|  (ROS Client)     |<---->|  (ros-network)    |<---->|   (ROS Client)    |
+-------------------+      +-------------------+      +-------------------+
         |                         |                          |
         |                         v                          |
         |                +-------------------+               |
         +--------------->|   orbslam3服务    |<--------------+
                          |  (ROS Master +    |
                          |   ORB-SLAM3节点)  |
                          +-------------------+
                                   |
                                   v
                          +-------------------+
                          |   kalibr服务      |
                          |  (相机标定工具)    |
                          +-------------------+
```

### 3. 关键ROS接口函数

#### 3.1 数据订阅回调函数
```cpp
// 图像回调函数
void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg);

// IMU回调函数  
void ImageGrabber::GrabImu(const sensor_msgs::ImuConstPtr& imu_msg);

// 时间同步回调
void ImageGrabber::SyncWithImu();
```

#### 3.2 数据处理流水线
```cpp
// 主要处理流程
1. 图像预处理: cv_bridge转换 → 灰度化 → 直方图均衡化
2. IMU预积分: 角速度和加速度积分 → 预积分协方差更新
3. 特征跟踪: ORB特征提取 → 特征匹配 → 运动模型预测
4. 位姿估计: 视觉-惯性紧耦合优化 → 关键帧决策
5. 地图更新: 局部地图点三角化 → 局部束调整 → 回环检测
```

#### 3.3 发布话题接口
```cpp
// 位姿输出
ros::Publisher pub_camera_pose;           // 发布相机位姿 (geometry_msgs/PoseStamped)
ros::Publisher pub_tf;                     // 发布TF变换

// 地图输出  
ros::Publisher pub_map_points;            // 发布地图点 (sensor_msgs/PointCloud2)
ros::Publisher pub_keyframes;             // 发布关键帧位姿
```

### 4. 算法关键模块

#### 4.1 IMU预积分模块
```cpp
class PreintegratedImuMeasurements {
public:
    void IntegrateNewMeasurement();       // 积分新的IMU测量值
    void PropagateState();                // 传播状态估计
    void ComputeJacobians();              // 计算雅可比矩阵
private:
    Eigen::Matrix3d dR;                   // 旋转预积分
    Eigen::Vector3d dv;                   // 速度预积分  
    Eigen::Vector3d dp;                   // 位置预积分
    Eigen::Matrix<double, 9, 9> Covariance; // 预积分协方差
};
```

#### 4.2 视觉-惯性优化器
```cpp
class Optimizer {
public:
    // 视觉-惯性局部束调整
    static void LocalInertialBA();
    
    // 全局视觉-惯性束调整
    static void FullInertialBA();
    
    // 仅视觉优化
    static void PoseOptimization();
private:
    // 使用g2o优化框架
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType* linearSolver;
};
```

#### 4.3 地图管理系统
```cpp
class Atlas {
public:
    void CreateNewMap();                  // 创建新地图
    void ChangeMap();                     // 切换活动地图
    void MergeMaps();                     // 合并多个地图
    
private:
    std::vector<Map*> mvMaps;            // 多地图存储
    Map* mpCurrentMap;                   // 当前活动地图
    unsigned int mnLastMapId;            // 最后地图ID
};
```

### 5. Docker编排关键配置

#### 5.1 环境变量配置
```yaml
environment:
  - ROS_HOSTNAME=orbslam3          # 容器主机名
  - ROS_MASTER_URI=http://orbslam3:11311  # ROS主节点URI
  - ROS_IP=orbslam3                # 容器IP标识
  - LD_LIBRARY_PATH=/opt/ros/noetic/lib:/ORB_SLAM3/lib
```

#### 5.2 网络配置
```yaml
networks:
  ros-network:
    driver: bridge
    ipam:
      config:
        - subnet: 172.28.0.0/16    # 固定子网段
```

#### 5.3 卷挂载配置（可选）
```yaml
volumes:
  # 数据集挂载
  - /path/to/dataset:/data/dataset:ro
  
  # 配置文件挂载
  - ./config:/ORB_SLAM3/config:ro
  
  # 输出结果挂载
  - ./results:/output:rw
```

### 6. 性能优化特性

#### 6.1 实时性保障
- **异步数据处理**：图像与IMU数据异步接收，通过时间戳同步
- **多线程并行**：四线程架构充分利用多核CPU
- **自适应特征提取**：根据场景复杂度动态调整特征点数量
- **选择性优化**：仅对关键帧和关联地图点进行优化

#### 6.2 内存管理
- **滑动窗口优化**：仅保持最近N个关键帧在优化窗口中
- **地图点剔除**：定期剔除质量低的地图点
- **关键帧稀疏化**：删除冗余关键帧以控制地图规模

#### 6.3 鲁棒性增强
- **初始化鲁棒性**：视觉-惯性联合初始化，避免纯视觉初始化失败
- **动态物体处理**：通过一致性检查过滤动态物体特征点
- **光照适应性**：自动曝光补偿与特征点自适应阈值

### 7. 监控与调试接口

#### 7.1 可视化输出
- **实时轨迹显示**：使用Pangolin库实时显示相机轨迹
- **地图点可视化**：3D稀疏点云实时渲染
- **关键帧显示**：显示当前跟踪状态和关键帧

#### 7.2 性能监控
- **帧处理时间统计**：记录每帧的处理时间
- **内存使用监控**：监控地图点和关键帧数量
- **定位精度评估**：与真实轨迹的误差计算

#### 7.3 日志系统
- **不同级别日志**：DEBUG, INFO, WARN, ERROR等级别
- **ROS日志集成**：通过rosconsole输出日志
- **文件日志记录**：关键数据持久化存储

这个Docker化的ORB-SLAM3模块通过精心的架构设计，成功解决了传统SLAM系统部署困难的问题，为Yanshee机器人提供了稳定、高效的三维环境感知能力。