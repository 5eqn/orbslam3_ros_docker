#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CompressedImageToImage:
    def __init__(self):
        rospy.init_node('compressed_to_raw', anonymous=True)
        
        # 创建CV桥接器
        self.bridge = CvBridge()
        
        # 订阅压缩图像话题
        self.compressed_sub = rospy.Subscriber(
            '/cv_camera/image_compressed', 
            CompressedImage, 
            self.compressed_callback,
            queue_size=1
        )
        
        # 发布解压后的图像话题
        self.image_pub = rospy.Publisher(
            '/camera/image_raw', 
            Image, 
            queue_size=1
        )
        
        rospy.loginfo("开始转换压缩图像为RGB原始图像...")
        rospy.loginfo("订阅: /cv_camera/image_compressed")
        rospy.loginfo("发布: /camera/image_raw")
        
    def compressed_callback(self, msg):
        try:
            # 将压缩图像消息转换为OpenCV图像
            # 注意：压缩图像数据在msg.data中
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                rospy.logwarn("无法解码压缩图像")
                return
            
            # 将BGR转换为RGB（OpenCV默认读取为BGR）
            cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # 将OpenCV图像转换为ROS Image消息
            image_msg = self.bridge.cv2_to_imgmsg(cv_image_rgb, "rgb8")
            
            # 复制时间戳和帧ID以保持同步
            image_msg.header.stamp = msg.header.stamp
            image_msg.header.frame_id = msg.header.frame_id
            
            # 发布图像
            self.image_pub.publish(image_msg)
            
        except CvBridgeError as e:
            rospy.logerr("CV桥接错误: %s", str(e))
        except Exception as e:
            rospy.logerr("处理图像时出错: %s", str(e))
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        converter = CompressedImageToImage()
        converter.run()
    except rospy.ROSInterruptException:
        pass