import time
import threading
from common.ros_robot_controller_sdk import Board
from common.action_group_controller import ActionGroupController

board = Board()
AGC = ActionGroupController(board)

print('''
**********************************************************
*********功能:，动作组控制例程(function: action group control)********
**********************************************************
----------------------------------------------------------
Official website:http://www.hiwonder.com
Online mall:https://huaner.tmall.com/
----------------------------------------------------------
----------------------------------------------------------
Usage:
    python3 action_group_control_demo.py
----------------------------------------------------------
Version: --V1.2  2020/12/23
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！(Press "Ctrl+C" to exit the program. If it fails, please try again multiple times!)
----------------------------------------------------------
''')

# 动作组需要保存在路径/home/pi/spiderpi/action_groups下(the action group needs to be saved under the path "/home/pi/spiderpi/action_groups")
AGC.run_action_group('stand_low')  # 参数为动作组的名称，不包含后缀，以字符形式传入(The parameter is the name of the action group, without the suffix, transmitted in the form of the string)
AGC.run_action_group('go_forward_low', times=2)  # 第二个参数为运行动作次数，默认1, 当为0时表示循环运行(The second parameter is the number of times to run the action, with a default value of 1. When it is 0, it means running the action repeatedly)

time.sleep(1)

th = threading.Thread(target=AGC.run_action_group, args=('turn_right_low', 0), daemon=True)  # 运行动作函数是阻塞式的，如果要循环运行一段时间后停止，请用线程来开启(The run action function is blocking. If you want to run it repeatedly for a period of time and then stop, please use a thread to start it)
th.start()
time.sleep(3)
AGC.stop_servo()  # 3秒后发出停止指令(send stop command after 3 seconds)
while th.is_alive(): # 等待动作完全停止(wait for the action to completely stop)
    time.sleep(0.01)
AGC.run_action_group('stand_low')
