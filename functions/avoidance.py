#!/usr/bin/python3
#coding=utf8
#第5章 AI视觉基础课程/第7课 自动避障(5.AI Vision Basic Lesson\Lesson 7 Obstacle Avoidance)
import os
import sys
import cv2
import time
import threading
import numpy as np
import pandas as pd
from common import yaml_handle
from common import kinematics
from calibration.camera import Camera 
from calibration.CalibrationConfig import *
from sensor.ultrasonic_sensor import Ultrasonic
import arm_ik.arm_move_ik as AMK


if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)


def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

load_config()

Threshold = 40.0 # 默认阈值40cm(default threshold is 40cm)
TextColor = (0, 255, 255)
TextSize = 12

__isRunning = False
distance = 0

def reset():
    ak.setPitchRangeMoving((0, 15, 30), 0, -90, 100, 1)

 
def init():
    reset()
    print('Avoidance Init')

def exit():
    global __isRunning
    
    ultrasonic.setRGBMode(0)
    ultrasonic.setRGB(0, (0, 0, 0))
    ultrasonic.setRGB(1, (0, 0, 0))
    __isRunning = False
    print('Avoidance Exit')

def setThreshold(args):
    global Threshold
    Threshold = args[0]
    return (True, (Threshold,))

def getThreshold(args):
    global Threshold
    return (True, (Threshold,))

def start():
    global __isRunning
    __isRunning = True
    print('Avoidance Start')

def stop():
    global __isRunning
    __isRunning = False
    ik.stand(ik.initial_pos)
    print('Avoidance Stop')

def move():
    while True:
        if __isRunning:
            if 0 < distance < Threshold:
                while distance < 25: # 小于25cm时后退(back up when the distance is less than 25cm)
                    ik.back(ik.initial_pos, 2, 80, 50, 1)
                for i in range(6): # 左转6次，每次15度，一共90度(Turn left 6 times with 15 degrees each time, a total of 90 degrees)
                    if __isRunning:
                        ik.turn_left(ik.initial_pos, 2, 50, 50, 1)
            else: 
                ik.go_forward(ik.initial_pos, 2, 80, 50, 1)
        else:
            time.sleep(0.01)

# 运行子线程(run sub-thread)
threading.Thread(target=move, daemon=True).start()

distance_data = []

def run(img):
    global __isRunning
    global Threshold
    global distance
    global distance_data

    if __isRunning:
        
        # 数据处理，过滤异常值(process data and filter abnormal values)
        distance_ = ultrasonic.getDistance() / 10.0
        distance_data.append(distance_)
        data = pd.DataFrame(distance_data)
        data_ = data.copy()
        u = data_.mean()  # 计算均值(calculate mean)
        std = data_.std()  # 计算标准差(calculate standard deviation)

        data_c = data[np.abs(data - u) <= std]
        distance = data_c.mean()[0]
        if len(distance_data) == 5:
            distance_data.remove(distance_data[0])

        cv2.putText(img, "Dist:%.1fcm" % distance, (30, 480 - 30), cv2.FONT_HERSHEY_SIMPLEX, 1.2, TextColor, 2)
    return img
if __name__ == '__main__':
    from common.ros_robot_controller_sdk import Board


    board = Board()
    ik = kinematics.IK(board)
    ultrasonic = Ultrasonic()
    ak = AMK.ArmIK()

    #加载参数(load parameters)
    param_data = np.load(calibration_param_path + '.npz')

    #获取参数(obtain parameters)
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)

    init()
    start()
    camera = Camera()
    camera.camera_open()
    
    
    while True:
        img = camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)           
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    camera.camera_close()
    cv2.destroyAllWindows()
