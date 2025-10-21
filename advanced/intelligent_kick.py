#!/usr/bin/python3
# coding=utf8
#第6章 A视觉进阶课程\2.进阶课程之智能踢球玩法课程\第2课 智能踢“球”(6.AI Vision Advanced Lesson\2.Kick the Ball\Lesson 2 Kick the Ball)
import sys
import cv2
import math
import numpy as np
import time
import threading
from calibration.camera import Camera
from calibration.CalibrationConfig import *
from common import misc
from common.pid import PID
from common import yaml_handle
from common import kinematics
from common.ros_robot_controller_sdk import Board
from sensor.ultrasonic_sensor import Ultrasonic
import arm_ik.arm_move_ik as AMK

debug = False

board = Board()
ik = kinematics.IK(board)
ultrasonic = Ultrasonic()
ak = AMK.ArmIK()

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

__target_color = ('green',)

x_pid = PID(P=0.1, I=0.08, D=0.008)#pid初始化(PID Initialization)
y_pid = PID(P=0.1, I=0.08, D=0.008)
X_pid = PID(P=0.2, I=0.01, D=0.008)
Y_pid = PID(P=0.8, I=0.01, D=0.008)

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'violet':(255,0,255),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

lab_data = None
def load_config():
    global lab_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

# 初始姿态参数定义(Initialize posture parameter definition)
# 姿态数据为3x6的数组，表示6条腿的的末端的x，y，z坐标，单位mm(The posture data is a 3x6 array, representing the x, y, z coordinates of the end points of the 6 legs, in the unit of millimeters)
# 头部朝前的方向为x轴， 头部朝前位置为负方向，右边为y轴正， 竖直朝上为z轴正， 从中间两条腿的连线的中点做垂线与上下板相交，取连线中心为零点(The direction facing forward is the x-axis, with the forward position being negative, the right side is the positive y-axis, and the vertical upward is the positive z-axis. A perpendicular line is drawn from the center point of the line connecting the two middle legs to intersect with the upper and lower plates, and the center of the line is taken as the zero point)
# 第一条腿表示头部朝前时左上角所在腿, 逆时针表示1-6(The first leg represents the upper left leg when the head is facing forward, numbered 1-6 counterclockwise)
current_pos = [[-199.53, -177.73, -100.0],
               [ -35.0, -211.27, -100.0],
               [199.53, -177.73, -100.0],
               [199.53,  177.73, -100.0],
               [ -35.0,  211.27, -100.0],
               [-199.53, 177.73, -100.0]]

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
y_dis = 170
# 初始位置(initial position)
def init_move():
    ultrasonic.setRGBMode(0)
    ultrasonic.setRGB(0, (0, 0, 0))
    ultrasonic.setRGB(1, (0, 0, 0))
    ik.stand(ik.initial_pos, t=500)
    ak.setPitchRangeMoving((0, 10, 16), -45, -90, 100, 1)
    board.bus_servo_set_position(0.5, [[24, y_dis, ], [21, x_dis, ]])
  

state = False
# 变量重置(reset variables)
def reset():
    global x_dis, y_dis, state
       
    x_dis = 500
    y_dis = 170
    x_pid.clear()
    y_pid.clear()
    X_pid.clear()
    Y_pid.clear()
    state = False
    ultrasonic.setRGBMode(0)
    ultrasonic.setRGB(0, (0, 0, 0))
    ultrasonic.setRGB(1, (0, 0, 0))

# 初始化调用(call initialization)
def init():
    init_move()
    load_config()
    reset()

def hisEqulColor(img):
    ycrcb = cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB)
    channels = cv2.split(ycrcb)
    cv2.equalizeHist(channels[0], channels[0])
    cv2.merge(channels, ycrcb)
    img_eq = cv2.cvtColor(ycrcb, cv2.COLOR_YCR_CB2BGR)
    return img_eq


search = True
p = [0, 0, 0]
o = [0, 0, 0]
# 机器人移动函数(robot movement function)
def move():
    global x_dis, y_dis, state
    
    d_x,d_y = 5,100
    direction = 0
    peak = False
    initial = True
    move_st = False
    dx,dy,dz = 0,0,0
    ago_direction = 'right'
    
    while True:
        if state:
            X_pid.SetPoint = 500    #设定(set)
            X_pid.update(x_dis)     #当前(current)
            dx = int(X_pid.output)
            
            Y_pid.SetPoint = 170    #设定(set)
            Y_pid.update(y_dis)     #当前(current)
            dy = int(Y_pid.output)
            dz = abs(dy)
            
            dx = 10 if dx > 10 else dx
            dx = -10 if dx < -10 else dx
            dy = 100 if dy > 100 else dy
            dy = -100 if dy < -100 else dy
            dz = 50 if dz > 50 else dz
            dz = 20 if dz < 20 else dz
            
            if abs(dx) > 5 or dy > 20 or dy < -10:
                if dy < -10:
                    direction = 0
                elif dy > 20 and abs(dx) < 15:
                    direction = 180
                    dx = -dx
                    
                if abs(dy) > 50:
                    amplitude = int(abs(dy)-abs(dx)*2)
                    
                elif abs(dy) < 20:
                    amplitude = int(abs(dy) + abs(dx)*1.5)
                    
                else:
                    amplitude = abs(dy)
                if dx > 5:
                    ago_direction = 'right'
                elif dx < -5:
                    ago_direction = 'left'
                ik.setStepMode(ik.initial_pos, 2, 2, amplitude*1.5, dz, direction, 0, dx/10, p, o, 50, 1)
                move_st = True
            
            else:
                ik.setStepMode(ik.initial_pos, 2, 2, 70, 20, 0, 0, 0, p, o, 50, 2)  # 朝前直走(go straight forward)
                ik.stand(ik.initial_pos, t=500)
                if x_dis <= 510:
                    ik.stand(current_pos)
                    time.sleep(0.5)
                    # 抬起右前脚(lift right front leg)
                    current_pos[5] = [-199.53, 177.73, -60]
                    ik.stand(current_pos) 
                    time.sleep(0.2)
                    # 右脚踢球(kick the ball with right leg)
                    current_pos[5] = [-340, -30, -80]
                    ik.stand(current_pos,200) 
                    time.sleep(0.5)
                    # 回到初始姿态(return to initial position)
                    current_pos[5] = [-199.53, 177.73, -100]
                    ik.stand(current_pos)
                    ik.stand(ik.initial_pos, t=500)
                else:
                    ik.stand(current_pos)
                    time.sleep(0.5)
                    # 抬起左前脚(lift left front leg)
                    current_pos[0] = [-199.53, 177.73, -60]
                    ik.stand(current_pos) 
                    time.sleep(0.2)
                    # 左脚踢球(kick the ball with left leg)
                    current_pos[0] = [-340, 30, -80]
                    ik.stand(current_pos,200) 
                    time.sleep(0.5)
                    # 回到初始姿态(return to initial position)
                    current_pos[0] = [-199.53, -177.73, -100]
                    ik.stand(current_pos)
                    ik.stand(ik.initial_pos, t=500)
        else:
            if move_st:
                move_st = False
                ik.stand(ik.initial_pos, t=500)
            if x_dis >= 800:
                d_x = -5
                y_dis += d_y
                y_dis = 260 if y_dis > 260 else y_dis
                y_dis = 160 if y_dis < 160 else y_dis
                board.bus_servo_set_position(0.2, [[24, y_dis]])
                if initial:
                    initial = False
                else:
                    peak = True
            elif x_dis <= 200:
                d_x = 5
                y_dis += d_y
                y_dis = 260 if y_dis > 260 else y_dis
                y_dis = 160 if y_dis < 160 else y_dis
                board.bus_servo_set_position(0.2, [[24, y_dis]])
                peak = True
                
            if abs(x_dis - 500) <= 10 and peak:     
                if y_dis >= 260:
                    d_y = -100
                    if ago_direction == 'right':
                        ik.turn_right(ik.initial_pos, 2, 30, 120, 3)
                    elif ago_direction == 'left':
                        ik.turn_left(ik.initial_pos, 2, 30, 120, 3)
                    ik.stand(ik.initial_pos, t = 300)
                    peak = False
                elif y_dis <= 160:
                    d_y = 100
                
            x_dis += d_x

            board.bus_servo_set_position(0.05, [[21, x_dis]])
            time.sleep(0.05)


# 运行子线程(run sub-thread)
threading.Thread(target=move, args=(), daemon=True).start()



size = (320, 240)
def run(img):
    global x_dis, y_dis, state, search
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    if __target_color == ():
        return img

    cv2.line(img, (int(img_w/2 - 10), int(img_h/2)), (int(img_w/2 + 10), int(img_h/2)), (0, 255, 255), 2)
    cv2.line(img, (int(img_w/2), int(img_h/2 - 10)), (int(img_w/2), int(img_h/2 + 10)), (0, 255, 255), 2)
    img_hisEqul = hisEqulColor(img_copy)
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
        if search:
            if abs(centerX - img_w/2) < 50:
                search = False
        else:
            if abs(centerX - img_w/2) > 15: 
                x_pid.SetPoint = img_w/2  #设定(set)
            else:
                x_pid.SetPoint = centerX
            x_pid.update(centerX)  #当前(current)
            dx = int(x_pid.output)
            x_dis += dx  #输出(output)
             
            if abs(centerY - img_h/2) > 15:
                y_pid.SetPoint = img_h/2
            else:
                y_pid.SetPoint = centerY
            y_pid.update(centerY)
            dy = int(y_pid.output)
            y_dis += dy
            
            x_dis = 0 if x_dis < 0 else x_dis          
            x_dis = 1000 if x_dis > 1000 else x_dis
            y_dis = 100 if y_dis < 100 else y_dis
            y_dis = 240 if y_dis > 240 else y_dis    
            
            if not debug:
                state = True
                board.bus_servo_set_position(0.02, [[24, y_dis], [21, x_dis]])
                print("y_dis",y_dis)
                time.sleep(0.02)
            
    else:
        state = False
        search = True
            
    return img

if __name__ == '__main__':
    from sensor.ultrasonic_sensor import Ultrasonic

    ultrasonic = Ultrasonic()

    #加载参数(load parameter)
    param_data = np.load(calibration_param_path + '.npz')

    #获取参数(obtain parameter)
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)

    init()
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
