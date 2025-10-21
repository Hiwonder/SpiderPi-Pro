#!/usr/bin/python3
# coding=utf8
#第6章 A视觉进阶课程\1.进阶课程之形状识别玩法课程\第2课 形状识别(6.AI Vision Advanced Lesson\1.Shape Recognition\Lesson 2 Shape Recognition)
import sys
import cv2
import math
import time
import signal
import threading
import numpy as np
from calibration.camera import Camera
from calibration.CalibrationConfig import *
from common import yaml_handle
from common import kinematics
from common.ros_robot_controller_sdk import Board
from common.action_group_controller import ActionGroupController
import arm_ik.arm_move_ik as AMK
from sensor.ultrasonic_sensor import Ultrasonic
import sensor.dot_matrix_sensor as DMS

 
# 形状识别(shape recognition)

# 点阵接口：扩展板io7、io8(dot matrix interface: expansion board io7 and io8)

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)



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
# 参数为要比较的轮廓的列表(parameter is the list of contour to be compared)
def get_area_maxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # 历遍所有轮廓(traverse through all contours)
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积(calculate contour area)
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 50:  # 只有在面积大于50时，最大面积的轮廓才是有效的，以过滤干扰(Only when the area is greater than the set value, the contour with the maximum area is considered valid to filter out interference)
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓(return the largest contour)

shape_length = 0

# 主要控制函数(main control function)
def move():
    global shape_length
    
    while move_st:
        if shape_length == 3:
            print('三角形')
            ## 显示'三角形'(display 'triangle')
            tm.display_buf = (0x80, 0xc0, 0xa0, 0x90, 0x88, 0x84, 0x82, 0x81,
                              0x81, 0x82, 0x84,0x88, 0x90, 0xa0, 0xc0, 0x80)
            tm.update_display()
            
        elif shape_length == 4:
            print('矩形')
            ## 显示'矩形'(display 'rectangle')
            tm.display_buf = (0x00, 0x00, 0x00, 0x00, 0xff, 0x81, 0x81, 0x81,
                              0x81, 0x81, 0x81,0xff, 0x00, 0x00, 0x00, 0x00)
            tm.update_display()
            
        elif shape_length >= 6:           
            print('圆')
            ## 显示'圆形'(display 'circle')
            tm.display_buf = (0x00, 0x00, 0x00, 0x00, 0x1c, 0x22, 0x41, 0x41,
                              0x41, 0x22, 0x1c,0x00, 0x00, 0x00, 0x00, 0x00)
            tm.update_display()
            
        else:
            ## 清屏(clear the screen)
            tm.display_buf = [0] * 16
            tm.update_display()
            print('None')
            
       
        
# 运行子线程(run sub-thread)
threading.Thread(target=move, args=(), daemon=True).start()


shape_list = []

# 退出关闭函数(exit the close function)
def Stop(signum, frame):
    global move_st
    move_st = False
    tm.display_buf = [0] * 16
    tm.update_display()
    print('关闭中...')
    agc.runActionGroup('lift_down')

signal.signal(signal.SIGINT, Stop)

# 主要图像处理函数(main image processing function)
def run(img):
    global shape_length, shape_list
    
    img_h, img_w = img.shape[:2]
    frame_gb = cv2.GaussianBlur(img, (3, 3), 3)      
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to LAB space)
    max_area = 0
    color_area_max = None    
    areaMaxContour_max = 0

    for i in lab_data:
        if i != 'white' and i != 'black':
            frame_mask = cv2.inRange(frame_lab,
                             (lab_data[i]['min'][0],
                              lab_data[i]['min'][1],
                              lab_data[i]['min'][2]),
                             (lab_data[i]['max'][0],
                              lab_data[i]['max'][1],
                              lab_data[i]['max'][2]))  #对原图像和掩模进行位运算(perform bitwise operation to original image and mask)
            opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6,6),np.uint8))  #开运算(opening operation)
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6,6),np.uint8)) #闭运算(Closing operation)
            contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #找出轮廓(find contours)
            areaMaxContour, area_max = get_area_maxContour(contours)  #找出最大轮廓(find the largest contour)
            if areaMaxContour is not None:
                if area_max > max_area:#找最大面积(find the maximum area)
                    max_area = area_max
                    color_area_max = i
                    areaMaxContour_max = areaMaxContour
    if max_area > 200:                   
        cv2.drawContours(img, areaMaxContour_max, -1, (0, 0, 255), 2)
        # 识别形状(shape recognition)
        # 周长  0.035 根据识别情况修改，识别越好，越小(Perimeter 0.035. Adjust according to the detection performance, the better the detection, the smaller the value)
        epsilon = 0.035 * cv2.arcLength(areaMaxContour_max, True)
        # 轮廓相似(contours are similar)
        approx = cv2.approxPolyDP(areaMaxContour_max, epsilon, True)
        shape_list.append(len(approx))
        if len(shape_list) == 24:
            shape_length = int(round(np.mean(shape_list)))                            
            shape_list = []
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

