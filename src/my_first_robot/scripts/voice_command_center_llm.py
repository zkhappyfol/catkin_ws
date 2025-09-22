#!/usr/bin/env python3
import rospy
import actionlib
import re
import os
# (找到文件的import区域)
import json
from openai import OpenAI

# (在import区域)
from openai import OpenAI # 我们使用OpenAI的库结构，但连接到DeepSeek的服务器
# ...
# (在parse_command函数内部)
client = OpenAI(api_key=os.environ.get("DEEPSEEK_API_KEY"), base_url="https://api.deepseek.com")

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

    # (这个新函数在你的VoiceCommandCenter类里面)
    def parse_command(self, command_text):
        """
        使用DeepSeek大语言模型来解析指令，提取意图和实体。
        """
        rospy.loginfo("正在调用DeepSeek LLM进行语义理解...")
        try:
        # --- 初始化DeepSeek客户端 ---
        # 强烈建议将密钥设置为环境变量，但为了教学我们先直接写在代码里
        # client = DeepSeekClient(api_key="sk-45c019e4ee3c4931b112029dafbeb648")
        # --- 初始化DeepSeek客户端 (使用最新的OpenAI兼容模式) ---
            client = OpenAI(
                api_key=os.environ.get("DEEPSEEK_API_KEY"),
                base_url="https://api.deepseek.com"
            )
        # ----------------------------------------------------

        # 获取我们所有已知的地点名字
            known_locations = list(LOCATIONS.keys())

        # 精心设计我们的Prompt
            prompt = f"""
            你是一个智能机器人的任务解析器。
            用户的指令是：“{command_text}”。
            已知的有效地点列表是：{known_locations}。
            请根据用户指令，判断其意图是否为“导航”(navigate)。
            如果是导航意图，请从已知地点列表中，找出用户最想去的那个地点。
            请严格按照下面的JSON格式返回结果，不要有任何多余的解释：
            {{
              "intent": "navigate" or "unknown",
              "location": "找到的地点名" or null
            }}
            """

        # 调用DeepSeek API
            response = client.chat.completions.create(
                model="deepseek-chat", # 使用DeepSeek的在线模型
                messages=[
                    {"role": "system", "content": "You are a helpful robot assistant that provides JSON output."},
                    {"role": "user", "content": prompt}
                ],
                response_format={"type": "json_object"}
            )

        # 解析返回的JSON结果
            response_json = json.loads(response.choices[0].message.content)

            rospy.loginfo(f"LLM解析结果: {response_json}")

            if response_json.get("intent") == "navigate" and response_json.get("location") in known_locations:
                return response_json.get("location")
            else:
                rospy.logwarn("LLM无法解析出有效的导航指令。")
                return None

        except Exception as e:
            rospy.logerr(f"调用DeepSeek API时发生错误: {e}")
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
