#!/usr/bin/python3
# coding=utf8
#第6章 A视觉进阶课程\4.进阶课程之巡线搬运玩法课程\第1课 色块分拣(6.AI Vision Advanced Lesson\4.Line Following and Transferring\Lesson 1 Color Sorting)
import sys
import cv2
import math
import time
import threading
import numpy as np
from common import misc
from common.pid import PID
from common import yaml_handle
from calibration.camera import Camera 
from calibration.CalibrationConfig import *
from sensor.ultrasonic_sensor import Ultrasonic
import arm_ik.arm_move_ik as AMK
from common import kinematics
from common.ros_robot_controller_sdk import Board
from sensor.ultrasonic_sensor import Ultrasonic
debug = False

board = Board()
ik = kinematics.IK(board)
ultrasonic = Ultrasonic()
ak = AMK.ArmIK()
# 颜色分拣(Color Sorting)

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
def load_config():
    global lab_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

load_config()

# 找出面积最大的轮廓(find the contour with the maximum area)
# 参数为要比较的轮廓的列表(parameter is the list of contour to be compared)
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0

    area_max_contour = None
    max_area = 0

    for c in contours:  # 历遍所有轮廓(traverse through all contours)
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积(calculate contour area)
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp >= 100:  # 只有在面积大于设定时，最大面积的轮廓才是有效的，以过滤干扰(Only when the area is greater than the set value, the contour with the maximum area is considered valid to filter out interference)
                area_max_contour = c
                max_area = contour_area_temp

    return area_max_contour, max_area  # 返回最大的轮廓, 面积(Return the area of the largest contour)

# 初始位置(initial position)
def initMove():
    ultrasonic.setRGBMode(0)
    ultrasonic.setRGB(0, (0, 0, 0))
    ultrasonic.setRGB(1, (0, 0, 0))  
    ak.setPitchRangeMoving((0, 12, 30), 0, -90, 100, 1)

color_list = []
detect_color = 'None'
action_finish = True
draw_color = range_rgb["black"]
# 变量重置(reset variables)
def reset():
    global draw_color
    global color_list
    global detect_color
    global action_finish
    
    color_list = []
    detect_color = 'None'
    action_finish = True
    draw_color = range_rgb["black"]
    
# 初始化(initialization)
def init():
    print("ColorSorting Init")
    load_config()
    initMove()

__isRunning = False
# 开始调用(start calling)
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorSorting Start")

# 机械臂运动函数(robotic arm movement function)
def move():
    global draw_color
    global detect_color
    global action_finish
    
    coord = {'blue':  ( 8, 24, -5.5),
             'green': (-8, 24, -5.5)}
    
    while True:
        if debug:
            return
        if __isRunning:
            if detect_color != 'None':
                action_finish = False
                if detect_color == 'red':
                    for i in range(3): # 识别到红色左右摇头(If red is recognized, shake the head left and right)
                        board.bus_servo_set_position(0.3,[[21,560]])
                        time.sleep(0.3)
                        board.bus_servo_set_position(0.3,[[21,440]])
                        time.sleep(0.3)
                    board.bus_servo_set_position(0.1,[[21,500]])
                    time.sleep(0.1)
                    detect_color = 'None'
                    draw_color = range_rgb["black"]                    
                    time.sleep(1)
                elif detect_color == 'green' or detect_color == 'blue': # 识别到蓝色或者绿色(blue or green is recognized)
                    board.bus_servo_set_position(0.6,[[25,120]])
                    time.sleep(0.6)
                    ak.setPitchRangeMoving((0, 18, 30), 0, -90, 100, 0.5) # 机械臂往前伸(extend the robotic arm forward)
                    time.sleep(2)
                    board.bus_servo_set_position(0.6,[[25,450]])
                    time.sleep(1)
                    x,y,z  = coord[detect_color] # 读取放置坐标(read placing coordinate)
                    pul, _ = ak.setPitchRange((x,y,z), -90, -80) # 逆运动学求解(calculate with inverse kinematics)
                    if pul is not None:
                        servo21 = pul['servo21']
                        board.bus_servo_set_position(0.5,[[21,servo21]])
                        time.sleep(0.5)
                    ak.setPitchRangeMoving((x,y,-2), -90, -90, 100, 1.6) # 机械臂运动到目标放置坐标上方(robotic arm moves above the target placing coordinate)
                    time.sleep(1.6)
                    ak.setPitchRangeMoving((x,y,z), -90, -90, 100, 0.6) # 到达放置目标(reach the target place for placing)
                    time.sleep(0.6)
                    board.bus_servo_set_position(0.6,[[25,120]])
                    time.sleep(0.8)
                    ak.setPitchRangeMoving((0, 12, 30), 0, -90, 100, 1.5) # 机械臂回初始姿态(robotic arm returns to the initial status)
                    time.sleep(1)
                    board.bus_servo_set_position(0.5,[[25,1000]])
                    time.sleep(1)
                    detect_color = 'None'
                    draw_color = range_rgb["black"]                    
                    time.sleep(1)
                else:
                    time.sleep(0.01)                
                action_finish = True                
                detect_color = 'None'
            else:
               time.sleep(0.01)
        else:
            time.sleep(0.01)

# 运行子线程(run sub-thread)
threading.Thread(target=move, args=(), daemon=True).start()

# 图像处理主要函数(color recognition function)
size = (320, 240)
def run(img):
    global draw_color
    global color_list
    global detect_color
    global action_finish
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    if not __isRunning:
        return img

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)      
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to LAB space)

    max_area = 0
    color_area_max = None    
    areaMaxContour_max = 0
    
    if action_finish:
        for i in lab_data:
            if i != 'black' and i != 'white':
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
                contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #找出轮廓(find contours)
                areaMaxContour, area_max = getAreaMaxContour(contours)  #找出最大轮廓(find the largest contour)
                if areaMaxContour is not None:
                    if area_max > max_area:#找最大面积(find the largest area)
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour
        if max_area > 100:  # 有找到最大面积(the maximum area has been found)
            ((centerX, centerY), radius) = cv2.minEnclosingCircle(areaMaxContour_max)  # 获取最小外接圆(obtain the minimum circumscribed circle)
            centerX = int(misc.map(centerX, 0, size[0], 0, img_w))
            centerY = int(misc.map(centerY, 0, size[1], 0, img_h))
            radius = int(misc.map(radius, 0, size[0], 0, img_w))            
            cv2.circle(img, (centerX, centerY), radius, range_rgb[color_area_max], 2) #画圆(draw the circle)

            if color_area_max == 'red':  #红色最大(red is the largest)
                color = 1
            elif color_area_max == 'green':  #绿色最大(green is the largest)
                color = 2
            elif color_area_max == 'blue':  #蓝色最大(blue is the largest)
                color = 3
            else:
                color = 0
            color_list.append(color)

            if len(color_list) == 3:  #多次判断(determine multiple times)
                # 取平均值(obtain mean)
                color = int(round(np.mean(np.array(color_list))))
                color_list = []
                if color == 1:
                    detect_color = 'red'
                    draw_color = range_rgb["red"]
                elif color == 2:
                    detect_color = 'green'
                    draw_color = range_rgb["green"]
                elif color == 3:
                    detect_color = 'blue'
                    draw_color = range_rgb["blue"]
                else:
                    detect_color = 'None'
                    draw_color = range_rgb["black"]               
        else:
            detect_color = 'None'
            draw_color = range_rgb["black"]
            
    cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
    
    return img

if __name__ == '__main__':

    #加载参数(load parameter)
    param_data = np.load(calibration_param_path + '.npz')

    #获取参数(obtain parameter)
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
