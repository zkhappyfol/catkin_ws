#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
# 【新】导入我们自己创建的自定义消息类型
from my_first_robot.msg import DetectedObject

class PerceptionNode:
    def __init__(self):
        rospy.init_node('perception_node', anonymous=True)
        self.bridge = CvBridge()

        # 【新】定义我们要识别的颜色和它们的HSV范围
        self.color_ranges = {
            'green': (np.array([40, 40, 40]), np.array([80, 255, 255])),
            'blue':  (np.array([100, 150, 0]), np.array([140, 255, 255])),
            'yellow':(np.array([20, 100, 100]), np.array([30, 255, 255]))
        }

        # 【新】创建一个发布者，发布我们自定义的“感知报告”
        self.object_publisher = rospy.Publisher('/detected_objects', DetectedObject, queue_size=10)

        # 订阅摄像头图像话题
        self.image_subscriber = rospy.Subscriber('/my_camera/rgb/image_raw', Image, self.image_callback)

        rospy.loginfo("多目标感知节点已启动，正在分析视野...")

    def image_callback(self, ros_image_message):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image_message, "bgr8")
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 【新】循环遍历我们定义的所有颜色
        for color, (lower, upper) in self.color_ranges.items():

            # 为当前颜色创建掩码
            mask = cv2.inRange(hsv_image, lower, upper)

            # 寻找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                # 如果找到了这种颜色的物体
                main_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(main_contour)

                # 我们设定一个最小面积阈值，过滤掉噪点
                if area > 500:
                    M = cv2.moments(main_contour)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        # 【新】创建一个自定义消息的实例
                        detection_msg = DetectedObject()
                        detection_msg.color = color
                        detection_msg.pixel_x = cx
                        detection_msg.pixel_y = cy
                        detection_msg.area = area

                        # 【新】发布这个“感知报告”
                        self.object_publisher.publish(detection_msg)
                        rospy.loginfo(f"检测到物体! 颜色: {color}, 坐标: ({cx}, {cy}), 面积: {area:.0f}")

# Python程序的标准入口点
if __name__ == '__main__':
    try:
        perceptor = PerceptionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass