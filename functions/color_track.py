#!/usr/bin/python3
# coding=utf8
#第5章 AI视觉基础课程/第3课 颜色追踪(5.AI Vision Basic Lesson\Lesson 3 Target Tracking)
import sys
import cv2
import math
import time
import numpy as np
from common import misc
from common.pid import PID
from common import yaml_handle
from calibration.camera import Camera 
from calibration.CalibrationConfig import *
from sensor.ultrasonic_sensor import Ultrasonic
import arm_ik.arm_move_ik as AMK


debug = False

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

lab_data = None
servo_data = None
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

load_config()

__target_color = ('green',)
# 设置检测颜色(set target color)
def setTargetColor(target_color):
    global __target_color

    __target_color = target_color
    return (True, ())

# 找出面积最大的轮廓(find the contour with the maximum area)
# 参数为要比较的轮廓的列表(parameter is the list of contour to be compared)
def get_area_maxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0

    area_max_contour = None
    max_area = 0

    for c in contours:  # 历遍所有轮廓(traverse through all contours)
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积(calculate contour area)
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 50:  # 只有在面积大于设定时，最大面积的轮廓才是有效的，以过滤干扰(Only when the area is greater than the set value, the contour with the maximum area is considered valid to filter out interference)
                area_max_contour = c
                max_area = contour_area_temp

    return area_max_contour, max_area  # 返回最大的轮廓(return the largest contour)

x_dis = 500
y_dis = 260
# 初始位置(initial position)
def init_move():
    ultrasonic.setRGBMode(0)
    ultrasonic.setRGB(0, (0, 0, 0))
    ultrasonic.setRGB(1, (0, 0, 0))     
    ak.setPitchRangeMoving((10, 15, 30), 0, -90, 100, 1)
    board.bus_servo_set_position(0.5, [[24, y_dis] , [21, x_dis]])

x_pid = PID(P=0.1, I=0.001, D=0.008)#pid初始化(PID initialization)
y_pid = PID(P=0.1, I=0.02, D=0.008)
# 变量重置(reset variables)
def reset():
    global x_dis, y_dis
    global __target_color
              
    x_dis = 500
    y_dis = 260
    x_pid.clear()
    y_pid.clear()
    __target_color = ()
    ultrasonic.setRGBMode(0)
    ultrasonic.setRGB(1, (0, 0, 0))
    ultrasonic.setRGB(2, (0, 0, 0))
    init_move()

# app初始化调用(call app initialization)
def init():
    print("ColorTrack Init")
    load_config()
    reset()

__isRunning = False
# app开始玩法调用(call app start game)
def start():
    global __isRunning
    __isRunning = True
    print("ColorTrack Start")

# app停止玩法调用(call app stop game)
def stop():
    global __isRunning
    __isRunning = False
    reset()
    print("ColorTrack Stop")

# app退出玩法调用(call app exit game)
def exit():
    global __isRunning
    __isRunning = False
    #ik.stand(ik.initial_pos)
    print("ColorTrack Exit")

def his_hqul_color(img):
    ycrcb = cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB)
    channels = cv2.split(ycrcb)
    cv2.equalizeHist(channels[0], channels[0])
    cv2.merge(channels, ycrcb)
    img_eq = cv2.cvtColor(ycrcb, cv2.COLOR_YCR_CB2BGR)
    return img_eq

size = (320, 240)
def run(img):
    global x_dis, y_dis
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    if not __isRunning or __target_color == ():
        return img

    cv2.line(img, (int(img_w/2 - 10), int(img_h/2)), (int(img_w/2 + 10), int(img_h/2)), (0, 255, 255), 2)
    cv2.line(img, (int(img_w/2), int(img_h/2 - 10)), (int(img_w/2), int(img_h/2 + 10)), (0, 255, 255), 2)

    img_hisEqul = his_hqul_color(img_copy)
   
    frame_resize = cv2.resize(img_hisEqul, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (5, 5), 5)   
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to LAB space)
    
    area_max = 0
    areaMaxContour = 0
    for i in lab_data:
        if i in __target_color:
            detect_color = i
            frame_mask = cv2.inRange(frame_lab,
                                         (lab_data[i]['min'][0],
                                          lab_data[i]['min'][1],
                                          lab_data[i]['min'][2]),
                                         (lab_data[i]['max'][0],
                                          lab_data[i]['max'][1],
                                          lab_data[i]['max'][2]))  #对原图像和掩模进行位运算(perform bitwise operation to original image and mask)
            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀(erode)
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #膨胀(dilate)
            if debug:
                cv2.imshow(i, dilated)
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓(find contours)
            areaMaxContour, area_max = get_area_maxContour(contours)  # 找出最大轮廓(find the largest contour)

    if area_max > 50:  # 有找到最大面积(the maximum area has been found)
        (centerX, centerY), radius = cv2.minEnclosingCircle(areaMaxContour) #获取最小外接圆(obtain the minimum circumscribed circle)
        centerX = int(misc.map(centerX, 0, size[0], 0, img_w))
        centerY = int(misc.map(centerY, 0, size[1], 0, img_h))
        radius = int(misc.map(radius, 0, size[0], 0, img_w))
        cv2.circle(img, (int(centerX), int(centerY)), int(radius), range_rgb[detect_color], 2)
        
        # use_time = 0
        x_pid.SetPoint = img_w/2  #设定(set)
        x_pid.update(centerX)  #当前(current)
        dx = int(x_pid.output)
        # use_time = abs(dx*0.00025)
        x_dis += dx  #输出(output)
        
        x_dis = 0 if x_dis < 0 else x_dis          
        x_dis = 1000 if x_dis > 1000 else x_dis
            
        y_pid.SetPoint = img_h/2
        y_pid.update(centerY)
        dy = int(y_pid.output)
        # use_time = round(max(use_time, abs(dy*0.00025)), 5)
        y_dis += dy
        
        y_dis = 0 if y_dis < 0 else y_dis
        y_dis = 1000 if y_dis > 1000 else y_dis    
        
        if not debug:
            board.bus_servo_set_position(0.02, [[24, y_dis], [21, x_dis]])
            time.sleep(0.02)
            
    return img

if __name__ == '__main__':
    from common.ros_robot_controller_sdk import Board

    board = Board()
    ultrasonic = Ultrasonic()
    ak = AMK.ArmIK()

    #加载参数(load parameter)
    param_data = np.load(calibration_param_path + '.npz')

    #获取参数(obtain parameter)
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)

    debug = False
    if debug:
        print('Debug Mode')
    
    init()
    start()
    camera = Camera()
    camera.camera_open() 
    __target_color = ('green',)
   
    while True:
        img = camera.frame
        if img is not None:
            frame = img.copy()
            frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)  # 畸变矫正(distortion correction)
            Frame = run(frame)           
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    camera.camera_close()
    cv2.destroyAllWindows()
