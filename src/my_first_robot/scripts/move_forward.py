#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class TurtleBotMover:
    def __init__(self):
        # 初始化节点
        rospy.init_node('turtle_mover_safe', anonymous=True)

        # 创建发布者
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 注册一个在节点关闭时执行的函数
        rospy.on_shutdown(self.shutdown_hook)

        # 设置循环频率
        self.rate = rospy.Rate(10)

        rospy.loginfo("安全移动节点已启动，按 Ctrl+C 停止。")

    def move_forward(self):
        """循环发布前进指令"""
        move_cmd = Twist()
        move_cmd.linear.x = 0.2
        move_cmd.angular.z = 0.0

        while not rospy.is_shutdown():
            self.velocity_publisher.publish(move_cmd)
            self.rate.sleep()

    def shutdown_hook(self):
        """在节点关闭时被调用的函数"""
        rospy.loginfo("节点正在关闭...")

        # 创建一个全零的速度指令
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0

        # 发布停止指令，确保机器人停下
        self.velocity_publisher.publish(stop_cmd)
        rospy.loginfo("已发送停止指令，机器人已安全停止。")
        # 稍微等待一下，确保指令发出
        rospy.sleep(1)

# Python程序的标准入口点
if __name__ == '__main__':
    try:
        # 创建一个类的实例
        mover = TurtleBotMover()
        # 调用主循环
        mover.move_forward()
    except rospy.ROSInterruptException:
        pass
