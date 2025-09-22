#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class ColorDetector:
    def __init__(self):
        # 初始化节点
        rospy.init_node('color_detector_node', anonymous=True)
        
        # 创建一个CvBridge的实例
        self.bridge = CvBridge()
        
        # 创建一个发布者，用于发送移动指令到/cmd_vel
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 订阅TurtleBot3的摄像头图像话题
        self.image_subscriber = rospy.Subscriber('/my_camera/rgb/image_raw', Image, self.image_callback)
        
        rospy.loginfo("智能搜寻节点已启动，正在搜寻绿色目标...")

    def image_callback(self, ros_image_message):
        """
        核心处理函数：感知、思考、行动
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image_message, "bgr8")
        except Exception as e:
            rospy.logerr(e)
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])
        mask = cv2.inRange(hsv_image, lower_green, upper_green)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        move_cmd = Twist()

        if len(contours) > 0:
            # --- 状态一：发现目标 ---
            main_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(main_contour)

            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                image_center_x = cv_image.shape[1] / 2
                error_x = cx - image_center_x
                
                # --- 决策逻辑 ---
                alignment_threshold = 20  # 对准的容忍阈值（像素）

                if abs(error_x) > alignment_threshold:
                    # 子状态 1.1: 未对准。此时【只】转向，不前进。
                    rospy.loginfo(f"发现目标！正在对准... 误差: {error_x}")
                    move_cmd.linear.x = 0.0
                    move_cmd.angular.z = -0.02 * error_x
                else:
                    # 子状态 1.2: 已对准。此时【只】前进，不转向。
                    rospy.loginfo(f"目标已对准！正在前进...")
                    move_cmd.linear.x = 0.2
                    move_cmd.angular.z = 0.0
        
        else:
            # --- 状态二：未发现目标 ---
            # 原地旋转搜寻
            rospy.loginfo("未发现目标，正在旋转搜寻...")
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.3 # 逆时针旋转

        # 最终发布移动指令
        self.cmd_vel_publisher.publish(move_cmd)

# Python程序的标准入口点
if __name__ == '__main__':
    try:
        detector = ColorDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass