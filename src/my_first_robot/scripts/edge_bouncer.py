#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math # 我们可能会用到一些数学运算

class EdgeBouncer:
    def __init__(self):
        # 1. 初始化节点、发布者和订阅者
        rospy.init_node('edge_bouncer_node', anonymous=True)
        
        # 创建一个发布者，用来发送速度指令
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # 创建一个订阅者，用来接收乌龟的位置信息
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # self.pose 用来存储乌龟的当前位置
        self.pose = Pose()
        
        # 设置循环频率
        self.rate = rospy.Rate(10)
        
        rospy.loginfo("智能小乌龟已启动，将自动避开墙壁！")

    def pose_callback(self, data):
        """回调函数：当接收到位置信息时，更新self.pose"""
        self.pose = data
        # 为了让乌龟能更准确地知道自己的位置，我们稍微调整一下角度的显示范围
        self.pose.theta = self.pose.theta if self.pose.theta > 0 else self.pose.theta + 2 * math.pi

    def move_loop(self):
        """主循环：包含“思考”和“行动”的逻辑"""
        # 设置安全边界
        safe_boundary = 1.0 
        
        while not rospy.is_shutdown():
            # 创建一个速度消息
            move_cmd = Twist()
            
            # --- 思考 (Think) ---
            # 检查是否太靠近上下左右四个边缘
            is_near_edge = (self.pose.x < safe_boundary or
                           self.pose.x > 11.0 - safe_boundary or
                           self.pose.y < safe_boundary or
                           self.pose.y > 11.0 - safe_boundary)
            
            # --- 行动 (Act) ---
            if is_near_edge:
                # 如果靠近边缘，就停止前进，并开始旋转
                rospy.loginfo("检测到边缘，正在转向...")
                move_cmd.linear.x = 1.0
                move_cmd.angular.z = 1.5 # 以1.5 rad/s的速度旋转
            else:
                # 如果在安全区域，就直线前进
                move_cmd.linear.x = 2.0 # 以2.0 m/s的速度前进
                move_cmd.angular.z = 0.0

            # 发布最终的指令
            self.velocity_publisher.publish(move_cmd)
            
            # 按照设定频率休眠
            self.rate.sleep()

# Python程序的标准入口点
if __name__ == '__main__':
    try:
        # 创建一个EdgeBouncer类的实例
        bouncer = EdgeBouncer()
        # 调用主循环函数
        bouncer.move_loop()
    except rospy.ROSInterruptException:
        pass
