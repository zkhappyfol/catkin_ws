#!/usr/bin/env python3

# 导入我们需要的库
import rospy
from geometry_msgs.msg import Twist # Twist 是一种标准的消息类型，专门用来表示速度指令

def move_turtle_forward():
    """
    这个函数是我们的主程序。
    它会初始化节点，创建一个发布者，然后循环发布速度指令。
    """
    try:
        # 1. 初始化一个ROS节点，并给它取一个独一无二的名字 'turtle_mover'
        rospy.init_node('turtle_mover', anonymous=True)

        # 2. 创建一个发布者(Publisher)
        #    - 它要发布到 '/turtle1/cmd_vel' 话题
        #    - 发布的消息类型是 Twist
        #    - queue_size=10 是一个标准的缓冲大小
        #velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 3. 创建一个空的 Twist 消息，为它赋值
        move_cmd = Twist()
        # 我们希望小乌龟直线前进
        move_cmd.linear.x = 0.2  # 设置x方向的线速度为2.0
        move_cmd.angular.z = 0.0 # 设置z方向的角速度为0（不转弯）

        # 4. 设置一个循环频率，比如每秒10次 (10 Hz)
        rate = rospy.Rate(10)
        
        rospy.loginfo("指令已发出: 让小乌龟前进！按 Ctrl+C 停止。")

        # 5. 写一个循环，只要ROS系统在运行，就持续发布消息
        while not rospy.is_shutdown():
            # 发布我们创建的速度指令消息
            velocity_publisher.publish(move_cmd)
            # 按照我们设定的10Hz频率进行休眠
            rate.sleep()

    except rospy.ROSInterruptException:
        # 这个异常会在我们按下 Ctrl+C 时触发，是正常退出的方式
        pass

# 这是Python程序的标准入口点
if __name__ == '__main__':
    move_turtle_forward()
