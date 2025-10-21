#!/usr/bin/python3
# coding=utf8
import sys
from common.ros_robot_controller_sdk import Board
from sensor.ultrasonic_sensor import Ultrasonic

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
board = Board()
ultrasonic = Ultrasonic()  

# 初始位置(initial position)
def initMove():
    ultrasonic.setRGBMode(0)
    ultrasonic.setRGB(1, (0, 0, 0))
    ultrasonic.setRGB(2, (0, 0, 0)) 
    #board.bus_servo_set_position(0.5, [[1, 1500], [2, 1500]])

def reset():
    return None

def init():
    initMove()
    print("RemoteControl Init")
    return None

def start():
    print("RemoteControl Start")
    return None

def stop():
    print("RemoteControl Stop")
    return None

def exit():
    print("RemoteControl Exit")
    return None

def run(img):
    return img