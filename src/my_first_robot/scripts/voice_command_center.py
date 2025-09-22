#!/usr/bin/env python3
import rospy
import actionlib
import re

# ROS 导航和语音相关的消息类型
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist

# 语音识别库
import speech_recognition as sr
from aip import AipSpeech

# 我们自己的“地址簿”
from my_first_robot.locations.semantic_map import LOCATIONS

# --- 把你从百度AI平台获取的密钥再次填在这里 ---
BAIDU_APP_ID = "7016284"
BAIDU_API_KEY = "O6LruUInl7NWhH3kT2cjHN8p"
BAIDU_SECRET_KEY = "4hLwemfe1LG4GE61ahxwYgZoT6JdWd0n"
# -----------------------------------------------

class VoiceCommandCenter:
    def __init__(self):
        # 初始化节点
        rospy.init_node('voice_command_center_node', anonymous=True)
        
        # 1. 初始化百度AI客户端
        self.baidu_client = AipSpeech(BAIDU_APP_ID, BAIDU_API_KEY, BAIDU_SECRET_KEY)
        
        # 2. 初始化语音识别器 (Recognizer) 和麦克风
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        rospy.loginfo("正在进行环境噪音校准，请保持安静...")
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
        rospy.loginfo("环境噪音校准完成。")

        # 3. 初始化 move_base 动作客户端
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("正在等待 move_base action 服务器连接...")
        connection_success = self.move_base_client.wait_for_server(rospy.Duration(15))
        if not connection_success:
            rospy.logerr("连接 move_base action 服务器失败！")
            rospy.signal_shutdown("无法连接到 move_base 服务器")
        else:
            rospy.loginfo("move_base action 服务器已连接！")
            
    def listen_and_recognize(self):
        """监听麦克风并返回识别出的文本"""
        with self.microphone as source:
            rospy.loginfo("请下达指令...")
            try:
                # 监听5秒钟
                audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5)
                rospy.loginfo("收到音频，正在用百度识别...")
                
                # 获取原始音频数据
                raw_data = audio.get_raw_data(convert_rate=16000, convert_width=2)
                
                # 调用百度API
                result = self.baidu_client.asr(raw_data, 'pcm', 16000, {'dev_pid': 1537,})

                if result and result.get('err_no') == 0:
                    text = result['result'][0]
                    rospy.loginfo(f"识别结果: '{text}'")
                    return text
                else:
                    rospy.logwarn(f"百度语音识别失败。详情: {result}")
                    return None

            except sr.WaitTimeoutError:
                rospy.loginfo("监听超时，没有检测到语音输入。")
                return None
            except Exception as e:
                rospy.logerr(f"识别过程中发生未知错误: {e}")
                return None

    # (这个函数在你的VoiceCommandCenter类里面，请用它替换掉旧的同名函数)
    def parse_command(self, command_text):
        """
        使用更灵活的规则解析语音识别后的文本指令。
        返回 (地点名称) 或者 None。
        """
        # 第一步：预处理文本。统一转成小写，并去掉句首句尾的空格和中英文句号。
        processed_text = command_text.lower().strip().rstrip('。').rstrip('.')

        # 第二步：尝试匹配中文指令模式，比如 "带我去 房间中心"
        match = re.search(r'(带我去|前往|开到)\s*(.*)', processed_text)
        if match:
            location = match.group(2).strip() # 提取 "房间中心"
            if location in LOCATIONS:
                rospy.loginfo(f"中文指令匹配成功！目标：{location}")
                return location

        # 第三步：尝试匹配标准英文指令模式, 比如 "go to room center"
        match = re.search(r'(go to|navigate to|move to)\s*(.*)', processed_text)
        if match:
            location = match.group(2).strip() # 提取 "room center"
            if location in LOCATIONS:
                rospy.loginfo(f"英文指令匹配成功！目标：{location}")
                return location

        # 第四步：终极武器！针对识别错误，直接在句子里寻找我们已知的地名
        # 这个方法可以成功处理像 "gotoroomcenter" 或者 "我想去charging_station啊" 这样的模糊指令
        for name in LOCATIONS.keys():
            if name in processed_text:
                rospy.loginfo(f"模糊匹配成功！在句中找到已知地点：{name}")
                return name

        # 如果所有规则都匹配失败
        rospy.logwarn(f"无法从指令 '{command_text}' 中解析出已知地点。")
        return None
    
    def go_to_location(self, location_name):
        """根据地点名称，发送导航目标"""
        target_coords = LOCATIONS[location_name]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = target_coords[0]
        goal.target_pose.pose.position.y = target_coords[1]
        goal.target_pose.pose.orientation.w = target_coords[2]
        
        self.move_base_client.send_goal(goal)
        rospy.loginfo(f"指令已发出！正在派机器人前往: {location_name}")
        # 这里我们发送后不等待，让机器人自己去，我们可以继续听下一个指令
        
    def run(self):
        """主循环，不断地听、解析、执行"""
        while not rospy.is_shutdown():
            # 1. 听
            recognized_text = self.listen_and_recognize()
            
            # 2. 解析
            if recognized_text:
                location = self.parse_command(recognized_text)
                
                # 3. 执行
                if location:
                    self.go_to_location(location)
            
            # 短暂休息一下，避免CPU占用过高
            rospy.sleep(1)


if __name__ == '__main__':
    try:
        center = VoiceCommandCenter()
        center.run()
    except rospy.ROSInterruptException:
        pass
