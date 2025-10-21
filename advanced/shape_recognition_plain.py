#!/usr/bin/python3
# coding=utf8
# 第6章 进阶课程\1.进阶课程之形状识别玩法课程\第1课 单种颜色形状识别(6.AI Vision Advanced Lesson\1.Shape Recognition\Lesson 1 Shape Recognition under Single Color)
import sys
import cv2
import math
import time
import signal
import threading
import numpy as np
from common import yaml_handle
from calibration.camera import Camera
from calibration.CalibrationConfig import *
from common import kinematics
from common.ros_robot_controller_sdk import Board
from common.action_group_controller import ActionGroupController
import arm_ik.arm_move_ik as AMK
from sensor.ultrasonic_sensor import Ultrasonic
import sensor.dot_matrix_sensor as DMS



if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# 蓝色形状识别(blue shape recognition)

# 点阵接口：扩展板io7、io8(dot matrix interface: expansion board io7 and io8)
    
board = Board()
ik = kinematics.IK(board)
ultrasonic = Ultrasonic()
agc = ActionGroupController(board)
ak = AMK.ArmIK()
tm = DMS.TM1640(dio=7, clk=8)

lab_data = None
move_st = True

# 读取颜色阈值函数(read color threshold and parameters of coordinate transformation)
def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

# 初始位置(initial position)
def init_move():
    ultrasonic.setRGBMode(0)
    ultrasonic.setRGB(0, (0, 0, 0))
    ultrasonic.setRGB(1, (0, 0, 0))
    ik.stand(ik.initial_pos)
    ak.setPitchRangeMoving((0, 12, 18), -60, -90, 100, 2)

# 找出面积最大的轮廓(find the contour with the maximum area)
def get_area_maxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None
    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 50:
                area_max_contour = c
    return area_max_contour, contour_area_max

shape_length = 0


    
# 主要控制函数(main control function)
def move():
    #global shape_length, board
    while move_st:
        if shape_length == 3:
            print('三角形')
            board.set_buzzer(2400, 0.1, 0.4, 1)  # 以2400Hz的频率，0.1秒开始响，0.4秒停止响，重复1次(The buzzer sounds at a frequency of 2400Hz for 0.1 seconds, followed by a pause of 0.4 seconds, and it repeats this pattern once)
            time.sleep(3)
            
        elif shape_length == 4:
            print('矩形')
            board.set_buzzer(2400, 0.1, 0.4, 2)  # 以2400Hz的频率，0.1秒开始响，0.4秒停止响，重复2次(The buzzer sounds at a frequency of 2400Hz for 0.1 seconds, followed by a pause of 0.4 seconds, and it repeats this pattern twice)
            time.sleep(3)
            
        elif shape_length >= 6:
            print('圆')
            board.set_buzzer(2400, 0.1, 0.4, 3)  # 以2400Hz的频率，0.1秒开始响，0.4秒停止响，重复3次(The buzzer sounds at a frequency of 2400Hz for 0.1 seconds, followed by a pause of 0.4 seconds, and it repeats this pattern three times)
            time.sleep(3)
            
        else:
            time.sleep(1)

# 运行子线程(run sub-thread)
# th = threading.Thread(target=move)
# th.setDaemon(True)
# time.sleep(3)
# th.start()  # 确保在初始化后启动线程(ensure to start the sub-thread after initialization)
threading.Thread(target=move, args=(), daemon=True).start()


shape_list = []

# 退出关闭函数(exit the close function)
def Stop(signum, frame):
    global move_st
    move_st = False
    tm.display_buf = [0] * 16
    tm.update_display()
    print('关闭中...')
    agc.run_action('lift_down')

signal.signal(signal.SIGINT, Stop)

# 主要图像处理函数(main image processing function)
def run(img):
    global shape_length, shape_list
    img_h, img_w = img.shape[:2]
    frame_gb = cv2.GaussianBlur(img, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
    max_area = 0
    color_area_max = None
    areaMaxContour_max = 0
    color = 'blue'
    frame_mask = cv2.inRange(frame_lab,
                             (lab_data[color]['min'][0],
                              lab_data[color]['min'][1],
                              lab_data[color]['min'][2]),
                             (lab_data[color]['max'][0],
                              lab_data[color]['max'][1],
                              lab_data[color]['max'][2]))
    opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
    areaMaxContour, area_max = get_area_maxContour(contours)
    if area_max > 200:
        cv2.drawContours(img, areaMaxContour, -1, (0, 0, 255), 2)
        epsilon = 0.035 * cv2.arcLength(areaMaxContour, True)
        approx = cv2.approxPolyDP(areaMaxContour, epsilon, True)
        shape_list.append(len(approx))
        if len(shape_list) == 24:
            shape_length = int(round(np.mean(shape_list)))
            shape_list = []
            #print(shape_length)
    else:
        shape_length = 0
    return img

if __name__ == '__main__':
    #加载参数(load parameter)
    param_data = np.load(calibration_param_path + '.npz')

    #获取参数(obtain parameter)
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)

    load_config()
    init_move()
    
    camera = Camera()
    camera.camera_open()
    while move_st:
        img = camera.frame
        if img is not None:
            frame = img.copy()
            frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)  # 畸变矫正(distortion correction)
            Frame = run(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                Stop()
                break
        else:
            time.sleep(0.01)
    camera.camera_close()
    cv2.destroyAllWindows()
