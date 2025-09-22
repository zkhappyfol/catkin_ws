#!/usr/bin/env python3
import rospy
import actionlib
import re
import os
import json
import threading # 【新】导入线程库

# ROS 消息类型
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

# 核心库
import speech_recognition as sr
from aip import AipSpeech
from openai import OpenAI # 用于连接DeepSeek/Qwen
import cv2
import numpy as np
from cv_bridge import CvBridge

# 我们自己的“地址簿” (虽然这次不用导航，但保留着)
from my_first_robot.locations.semantic_map import LOCATIONS

# --- 密钥配置 ---
BAIDU_APP_ID = "7016284"
BAIDU_API_KEY = "O6LruUInl7NWhH3kT2cjHN8p"
BAIDU_SECRET_KEY = "4hLwemfe1LG4GE61ahxwYgZoT6JdWd0n"
# -----------------

class MissionCommander:
    def __init__(self):
        rospy.init_node('mission_commander_node', anonymous=True)
        
        # --- 初始化所有模块 ---
        # 1. 百度语音客户端 (耳朵)
        self.baidu_client = AipSpeech(BAIDU_APP_ID, BAIDU_API_KEY, BAIDU_SECRET_KEY)
        
        # 2. 语音识别器和麦克风
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        rospy.loginfo("正在进行环境噪音校准...")
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
        rospy.loginfo("校准完成。")

        # 3. 视觉模块
        self.bridge = CvBridge()
        self.image_subscriber = rospy.Subscriber('/my_camera/rgb/image_raw', Image, self.image_callback)
        self.color_ranges = {
            'green': (np.array([40, 40, 40]), np.array([80, 255, 255])),
            'blue':  (np.array([100, 150, 0]), np.array([140, 255, 255])),
            'yellow':(np.array([20, 100, 100]), np.array([30, 255, 255]))
        }
        self.detected_object_info = None # 储存当前看到的物体信息

        # 4. 运动控制模块
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.last_error_x = 0 # 【已添加】为PD控制器做准备
        
        # 5. 任务状态机
        self.current_target_color = None # 当前要寻找的目标颜色
        self.robot_state = "IDLE" # IDLE, SEARCHING, APPROACHING

        # --- 【新】为多线程准备 ---
        self.last_command = None # 储存最新听到的指令
        self.command_lock = threading.Lock() # 线程锁，防止数据冲突
        # 创建并启动我们的“耳朵”线程
        self.listener_thread = threading.Thread(target=self.background_listener)
        self.listener_thread.daemon = True # 设置为守护线程，主程序退出时它也退出
        self.listener_thread.start()
        # --------------------------

        rospy.loginfo(">>> 任务指挥中心已上线，等待语音指令... <<<")

    def image_callback(self, ros_image_message):
        """视觉处理回调函数，只负责更新看到了什么"""
        cv_image = self.bridge.imgmsg_to_cv2(ros_image_message, "bgr8")
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        self.detected_object_info = None # 先清空上次的发现

        # 只寻找我们当前任务指定的目标颜色
        if self.current_target_color and self.current_target_color in self.color_ranges:
            lower, upper = self.color_ranges[self.current_target_color]
            mask = cv2.inRange(hsv_image, lower, upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) > 0:
                main_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(main_contour)
                if area > 500:
                    M = cv2.moments(main_contour)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        # 更新发现的物体信息
                        self.detected_object_info = {'color': self.current_target_color, 'cx': cx, 'area': area, 'image_center_x': cv_image.shape[1] / 2}

    def listen_and_recognize(self):
        """监听并识别语音指令"""
        # ... (这个函数和我们之前的voice_command_center.py里的一样) ...
        # (为了简洁，这里省略，请从之前的代码复制过来)
        """监听麦克风并返回识别出的文本"""
        with self.microphone as source:
            rospy.loginfo("请下达指令...")
            try:
                # 监听8秒钟
                audio = self.recognizer.listen(source, timeout=8, phrase_time_limit=8)
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
            
    def background_listener(self):
        """【修正后】后台监听线程，它的职责就是循环调用listen_and_recognize"""
        rospy.loginfo("后台监听线程已启动。")
        while not rospy.is_shutdown():
            # 调用我们唯一的语音识别函数
            recognized_text = self.listen_and_recognize()
            
            if recognized_text:
                # 把听到的指令安全地存起来
                with self.command_lock:
                    self.last_command = recognized_text
            # 短暂休息一下，避免在没有语音时CPU空转
            rospy.sleep(0.5)

    def parse_with_llm(self, command_text):
        rospy.loginfo("正在调用DeepSeek LLM进行语义理解...")
        try:
            client = OpenAI(api_key=os.environ.get("DEEPSEEK_API_KEY"), base_url="https://api.deepseek.com")
            known_objects = {
                "绿色方块": "green",
                "蓝色球体": "blue",
                "黄色圆柱体": "yellow"
            }
            prompt = f"用户指令是：“{command_text}”。已知的物体有：{list(known_objects.keys())}。请判断用户想找哪个物体，并返回该物体对应的英文颜色名。请严格用JSON格式返回，例如{{'intent':'find_object', 'color':'green'}}或{{'intent':'unknown', 'color':null}}。"
            
            response = client.chat.completions.create(
                model="deepseek-chat", messages=[{"role": "user", "content": prompt}],
                response_format={"type": "json_object"}
            )
            response_json = json.loads(response.choices[0].message.content)
            rospy.loginfo(f"LLM解析结果: {response_json}")
            
            if response_json.get("intent") == "find_object" and response_json.get("color") in self.color_ranges:
                return response_json
        except Exception as e:
            rospy.logerr(f"调用DeepSeek API时发生错误: {e}")
        return None

    def run(self):
        """【修正后】主任务循环，不再直接调用听的函数"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            move_cmd = Twist()

            # --- 检查“潜意识”里有没有新指令 ---
            current_command = None
            with self.command_lock:
                if self.last_command:
                    current_command = self.last_command
                    self.last_command = None # 指令只处理一次
            
            if current_command:
                # 如果有新指令，就用LLM解析
                parsed_result = self.parse_with_llm(current_command)
                if parsed_result and isinstance(parsed_result, dict):
                    intent = parsed_result.get("intent")
                    if intent == "find_object":
                        target_color = parsed_result.get("color")
                        if target_color in self.color_ranges:
                            self.current_target_color = target_color
                            self.robot_state = "SEARCHING"
                            self.last_error_x = 0
                            rospy.loginfo(f"收到新任务！开始寻找'{target_color}'。")
                    elif intent == "stop":
                        rospy.loginfo("收到停止指令！任务中断，返回待命状态。")
                        self.robot_state = "IDLE"
                        self.current_target_color = None
                        # 确保机器人停下
                        self.cmd_vel_publisher.publish(Twist()) 
                
                
            # --- 主状态机逻辑 (和之前类似) ---
            if self.robot_state == "SEARCHING":
                if self.detected_object_info:
                    rospy.loginfo("发现目标！切换到接近模式。")
                    self.robot_state = "APPROACHING"
                else:
                    # 旋转搜寻
                    move_cmd.angular.z = 0.3
                    rospy.loginfo("正在搜寻目标...")

            elif self.robot_state == "APPROACHING":
                if not self.detected_object_info:
                    rospy.loginfo("目标丢失！返回搜寻模式。")
                    self.robot_state = "SEARCHING"
                else:
                    # 使用我们成熟的视觉伺服逻辑
                    error_x = self.detected_object_info['cx'] - self.detected_object_info['image_center_x']
                    move_cmd.angular.z = -0.02 * error_x
                    move_cmd.linear.x = 0.2
                    
                    # 检查是否到达
                    if self.detected_object_info['area'] > 30000:
                        rospy.loginfo("已到达目标！任务完成，返回待命状态。")
                        self.robot_state = "IDLE"
                        self.current_target_color = None
                        move_cmd = Twist() # 发送停止指令
                    else:
                        # 【已添加】完整的PD控制器逻辑
                        error_x = self.detected_object_info['cx'] - self.detected_object_info['image_center_x']
                        error_derivative = error_x - self.last_error_x
                        self.last_error_x = error_x
                        p_gain = -0.01
                        d_gain = -0.02
                        angular_speed = p_gain * error_x + d_gain * error_derivative
                        
                        move_cmd.angular.z = angular_speed
                        move_cmd.linear.x = 0.2
                        rospy.loginfo(f"目标已锁定！面积: {self.detected_object_info['area']:.0f} | 误差: {error_x} | 正在接近...")
            
            # 只有在非IDLE状态才发布移动指令
            if self.robot_state != "IDLE":
                self.cmd_vel_publisher.publish(move_cmd)

            rate.sleep()

if __name__ == '__main__':
    # (为了简洁，这里省略了完整的try/except块，请保留你之前的版本)
    commander = MissionCommander()
    commander.run()