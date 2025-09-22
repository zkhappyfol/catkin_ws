#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class ColorDetector:
    def __init__(self):
        rospy.init_node('color_detector_node', anonymous=True)
        self.bridge = CvBridge()
        
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 我们使用带有我们自己摄像头的 launch 文件，所以话题名是 /my_camera/rgb/image_raw
        self.image_subscriber = rospy.Subscriber('/my_camera/rgb/image_raw', Image, self.image_callback)
        
        self.move_cmd = Twist()
        self.is_running = False

        # --- 就是这行，确保它在这里 ---
        self.last_error_x = 0
        # --------------------------------

        rospy.loginfo("视觉伺服节点已启动，正在等待摄像头上线...")

    def image_callback(self, ros_image_message):
        if not self.is_running:
            rospy.loginfo("摄像头已连接！视觉伺服系统开始运行！")
            self.is_running = True

        cv_image = self.bridge.imgmsg_to_cv2(ros_image_message, "bgr8")
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])
        mask = cv2.inRange(hsv_image, lower_green, upper_green)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # (在 image_callback 函数中)
        if len(contours) > 0:
            main_contour = max(contours, key=cv2.contourArea)

    # --- 新增的“终点判断”逻辑 ---
            contour_area = cv2.contourArea(main_contour)
            goal_area_threshold = 60000 # 面积阈值，当绿色方块面积大于这个值时，认为已到达。这个值需要实验来微调。

            if contour_area > goal_area_threshold:
                # 状态三：已到达。停止所有动作。
                self.move_cmd.linear.x = 0.0
                self.move_cmd.angular.z = 0.0
                rospy.loginfo(f"目标已到达！面积: {contour_area:.0f}。任务完成，原地待命。")

            else:
                M = cv2.moments(main_contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    image_center_x = cv_image.shape[1] / 2

                    # --- 全新的PD控制逻辑 ---
                    # P项：比例控制 (和之前一样)
                    error_x = cx - image_center_x

                    # D项：微分控制 (我们的“刹车”)
                    # 误差的变化率 = 当前误差 - 上一次的误差
                    error_derivative = error_x - self.last_error_x

                    # 更新“上一次的误差”
                    self.last_error_x = error_x

                    # 定义我们的P和D增益系数 (这些需要反复调试来找到最佳值)
                    p_gain = -0.01  # 比例增益，稍微减小一点
                    d_gain = -0.02 # 微分增益，用来提供“阻尼”

                    # 最终的转向速度 = P项的贡献 + D项的贡献
                    angular_speed = p_gain * error_x + d_gain * error_derivative

                    self.move_cmd.angular.z = angular_speed
                    self.move_cmd.linear.x = 0.1

                    rospy.loginfo(f"目标已锁定！误差: {error_x} | 前进: {self.move_cmd.linear.x:.2f} | 转向: {self.move_cmd.angular.z:.2f}")
                    # --- PD控制逻辑结束 ---

        else:
            # 搜寻状态保持不变
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.3
            rospy.loginfo("正在搜寻目标...")
        
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.is_running:
                # 持续不断地发布最新的移动指令
                self.cmd_vel_publisher.publish(self.move_cmd)
            rate.sleep()

if __name__ == '__main__':
    try:
        detector = ColorDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass