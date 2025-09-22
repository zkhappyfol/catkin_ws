#!/usr/bin/env python3

# 导入我们需要的库
import rospy
# 小乌龟的位置消息类型叫做 Pose，我们需要从 turtlesim.msg 导入它
from turtlesim.msg import Pose

def pose_callback(pose_message):
    """
    这是一个回调函数。
    当订阅者从 /turtle1/pose 话题接收到一条新消息时，ROS会自动调用这个函数。
    接收到的消息会作为参数(pose_message)传递给这个函数。
    """
    # Pose 消息里面包含了x, y坐标和朝向theta等信息
    x = pose_message.x
    y = pose_message.y
    theta = pose_message.theta
    
    # 我们把坐标信息格式化后打印出来，保留两位小数让它更整洁
    rospy.loginfo(f"乌龟当前位置: x={x:.2f}, y={y:.2f}, 朝向: theta={theta:.2f}")

def subscribe_to_pose():
    """
    这个函数是我们的主程序。
    它会初始化节点，创建一个订阅者，然后进入待命状态。
    """
    try:
        # 1. 初始化一个ROS节点，名字叫 'turtle_pose_subscriber'
        rospy.init_node('turtle_pose_subscriber', anonymous=True)

        # 2. 创建一个订阅者(Subscriber)
        #    - 订阅的话题名是 '/turtle1/pose'
        #    - 消息的类型是 Pose
        #    - 当收到消息后，自动调用我们上面定义的 pose_callback 函数
        rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
        
        rospy.loginfo("情报侦察员已就位，正在监听乌龟位置...")

        # 3. rospy.spin()
        #    这行代码让我们的节点“原地待命”，不会在启动后立刻退出。
        #    它就在这里静静地等待消息的到来，然后调用回调函数进行处理。
        #    这是所有订阅者节点都必须有的。
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

# Python程序的标准入口点
if __name__ == '__main__':
    subscribe_to_pose()
