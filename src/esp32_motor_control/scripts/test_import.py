#!/usr/bin/env python
# -*-coding:utf8-*-
import sys
import os

# 手动添加路径（调试用）
workspace_path = os.path.expanduser('~/catkin_ws/devel/lib/python2.7/dist-packages')
if workspace_path not in sys.path:
    sys.path.insert(0, workspace_path)

print("Python路径:")
for path in sys.path:
    print("  " + path)

print("\n尝试导入消息...")
try:
    from esp32_motor_control.msg import MotorCommand
    print("✓ 消息导入成功!")
    
    # 测试创建消息实例
    msg = MotorCommand()
    msg.speed = 100
    msg.steps = 500
    msg.direction = True
    msg.enable = True
    print("✓ 消息创建成功!")
    
except ImportError as e:
    print("✗ 导入失败:", e)
    print("\n检查生成的文件...")
    msg_path = os.path.join(workspace_path, 'esp32_motor_control/msg')
    if os.path.exists(msg_path):
        print("找到msg目录，内容:")
        for f in os.listdir(msg_path):
            print("  " + f)
    else:
        print("未找到msg目录:", msg_path)
