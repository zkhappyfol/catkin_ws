#!/usr/bin/env python3
import re
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from my_first_robot.locations.semantic_map import LOCATIONS # 从我们的“地址簿”导入坐标

class CommandCenter:
    def __init__(self):
        # 初始化节点
        rospy.init_node('command_center_node', anonymous=True)
        
        # 创建一个连接到move_base服务器的Action Client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("正在等待move_base action服务器连接...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("move_base action服务器已连接！")

    def go_to_location(self, location_name):
        """根据地点名称，发送导航目标"""
        if location_name not in LOCATIONS:
            rospy.logwarn(f"错误：地点 '{location_name}' 不在我的知识库里。")
            return

        # 从我们的地址簿里获取坐标
        target_coords = LOCATIONS[location_name]
        
        # 创建一个导航目标 (MoveBaseGoal)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map" # 目标是在map坐标系下
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # 设置目标点的x, y坐标
        goal.target_pose.pose.position.x = target_coords[0]
        goal.target_pose.pose.position.y = target_coords[1]
        
        # 设置目标点的朝向 (我们用一个简单的w=1.0的四元数表示)
        goal.target_pose.pose.orientation.w = target_coords[2]
        
        # 发送目标给move_base
        rospy.loginfo(f"收到指令！正在前往: {location_name} at {target_coords}")
        self.move_base_client.send_goal(goal)
        
        # 等待机器人到达目标点
        self.move_base_client.wait_for_result()
        
        # 打印出结果
        rospy.loginfo(f"已到达 {location_name}！")
        
    # (这个函数在你的CommandCenter类里面)
    def parse_command(self, command):
    	"""
    	使用正则表达式解析指令，提取意图和实体。
    	目前只支持导航意图。
    	返回 (地点名称) 或者 None。
    	"""
    	# 定义一个匹配导航指令的正则表达式模式
    	# 这个模式会匹配 "go to/navigate to/move to <某个地点>" 这样的句子
    	# re.IGNORECASE 参数让它不区分大小写
    	    match = re.search(r'(go|navigate|move) to (.*)', command, re.IGNORECASE)

    	if match:
        # 如果匹配成功，group(2)就是我们括号里匹配到的第二个部分，也就是地点名称
            location = match.group(2).strip() # .strip() 是为了去除可能存在的前后空格

        # 检查提取出的地点是否在我们已知的地址簿里
            if location in LOCATIONS:
                return location

        # 如果没有匹配成功，或者地点未知，就返回None
        return None

    def command_loop(self):
    """主循环，接收用户输入的指令"""
        while not rospy.is_shutdown():
            try:
                command = input("请输入指令 (e.g., 'go to room_center' or 'exit'): ")

                if command.lower() == 'exit':
                    rospy.loginfo("收到退出指令。")
                    break

                # 调用我们新的解析函数
                location_name = self.parse_command(command)

                if location_name:
                    # 如果成功解析出地点，就执行导航
                    self.go_to_location(location_name)
                else:
                    # 否则，提示无法理解
                    rospy.logwarn("无法理解的指令。请尝试 'go to <地点名>' 或 'navigate to <地点名>'")

            except EOFError:
                break

# Python程序的标准入口点
if __name__ == '__main__':
    try:
        center = CommandCenter()
        center.command_loop()
    except rospy.ROSInterruptException:
        pass
