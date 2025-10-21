#!/usr/bin/python3
# coding=utf8
import sys
import time
import common.ros_robot_controller_sdk as rrc

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
print('''
**********************************************************
********************PWM舵机测试(PWM servo test)***************************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！(Press 'Ctrl+C' to exit the program, if it fails, please try again several times!)
----------------------------------------------------------
''')
board = rrc.Board()
board.set_buzzer(1900, 0.1, 0.9, 1)# 设置蜂鸣器响0.1秒(set the buzzer to sound for 0.1 seconds)
board.set_rgb([[1, 255, 0, 0], [2, 255, 0, 0]]) # 设置扩展板上的彩灯(set the RGB light on the expansion board)
board.pwm_servo_set_position(0.3, [[1, 1800]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[1, 1500]]) 
board.set_rgb([[1, 0, 255, 0], [2, 0, 255, 0]]) # 设置扩展板上的彩灯(set the RGB light on the expansion board)
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[1, 1200]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[1, 1500]]) 
board.set_rgb([[1, 0, 0, 255], [2, 0, 0, 255]]) # 设置扩展板上的彩灯(set the RGB light on the expansion board)
time.sleep(1)

board.pwm_servo_set_position(0.3, [[2, 1200]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[2, 1500]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[2, 1800]])
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[2, 1500]]) 
time.sleep(1)

board.pwm_servo_set_position(0.3, [[3, 1200]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[3, 1500]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[3, 1800]])
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[3, 1500]]) 
time.sleep(1)

board.pwm_servo_set_position(0.3, [[4, 1200]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[4, 1500]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[4, 1800]])
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[4, 1500]]) 
time.sleep(1)

board.pwm_servo_set_position(0.3, [[5, 1200]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[5, 1500]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[5, 1800]])
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[5, 1500]]) 
time.sleep(1)

board.pwm_servo_set_position(0.3, [[6, 1200]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[6, 1500]]) 
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[6, 1800]])
time.sleep(0.3)
board.pwm_servo_set_position(0.3, [[6, 1500]]) 
time.sleep(1)

board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]]) # 设置扩展板上的彩灯(set the RGB light on the expansion board)
