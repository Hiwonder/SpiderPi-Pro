#!/usr/bin/python3
# coding=utf8
#第5章 AI视觉基础课程/第6课 标签识别(5.AI Vision Basic Lesson\Lesson 6 Tag Recognition)
import sys
import math
import threading
import time
import cv2
import numpy as np
from common import yaml_handle
from calibration.camera import Camera 
from calibration.CalibrationConfig import *
from common import kinematics
import common.apriltag as apriltag


debug = False

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

lab_data = None
servo_data = None
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

load_config()

# 初始位置(initial position)
def initMove():
    ultrasonic.setRGBMode(0)
    ultrasonic.setRGB(0, (0, 0, 0))
    ultrasonic.setRGB(1, (0, 0, 0)) 

    ak.setPitchRangeMoving((0, 12, 35), 30, -90, 100, 1)

tag_id = None
__isRunning = False
action_finish = True
# 变量重置(reset variables)
def reset():      
    global tag_id
    global action_finish
    
    tag_id = 0
    action_finish = True
    
# app初始化调用(call app initialization)
def init():
    print("Apriltag Init")
    initMove()

# app开始玩法调用(call app start game)
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("Apriltag Start")

# app停止玩法调用(call app stop game)
def stop():
    global __isRunning
    __isRunning = False
    print("Apriltag Stop")

# app退出玩法调用(call app exit game)
def exit():
    global __isRunning
    __isRunning = False
    agc.run_action('stand_low')
    print("Apriltag Exit")

def move():
    global tag_id
    global action_finish  

    LOCK_SERVOS={'21':500, '22':750, '23':33, '24':466}

    while True:
        if debug:
            return
        if __isRunning:
            if tag_id is not None:
                action_finish = False
                time.sleep(0.5)
                if tag_id == 1:               
                    agc.run_action_group('wave',lock_servos=LOCK_SERVOS)#招手(wave)
                    tag_id = None
                    time.sleep(1)                  
                    action_finish = True                
                elif tag_id == 2:                    
                    agc.run_action_group('stepping',lock_servos=LOCK_SERVOS)#原地踏步(stepping)
                    tag_id = None
                    time.sleep(1)
                    action_finish = True          
                elif tag_id == 3:                   
                    agc.run_action_group('twist_l',lock_servos=LOCK_SERVOS)#扭腰(twist)
                    tag_id = None
                    time.sleep(1)
                    action_finish = True
                else:
                    action_finish = True
                    time.sleep(0.01)
            else:
               time.sleep(0.01)
        else:
            time.sleep(0.01)

# 运行子线程(run sub-thread)
threading.Thread(target=move, args=(), daemon=True).start()


# 检测apriltag(detect apriltag)
detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
def apriltagDetect(img):   
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray, return_image=False)

    if len(detections) != 0:
        for detection in detections:                       
            corners = np.rint(detection.corners)  # 获取四个角点(obtain the four corner points)
            cv2.drawContours(img, [np.array(corners, np.int64)], -1, (0, 255, 255), 2)

            tag_family = str(detection.tag_family, encoding='utf-8')  # 获取tag_family(obtain tag_family)
            tag_id = int(detection.tag_id)  # 获取tag_id(obtain tag_id)

            object_center_x, object_center_y = int(detection.center[0]), int(detection.center[1])  # 中心点(center point)
            
            object_angle = int(math.degrees(math.atan2(corners[0][1] - corners[1][1], corners[0][0] - corners[1][0])))  # 计算旋转角(calculate rotation angle)
            
            return tag_family, tag_id
            
    return None, None

def run(img):
    global tag_id
    global action_finish
     
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    if not __isRunning:
        return img
    
    tag_family, tag_id = apriltagDetect(img) # apriltag检测(detect apriltag)
    
    if tag_id is not None:
        cv2.putText(img, "tag_id: " + str(tag_id), (10, img.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)
        cv2.putText(img, "tag_family: " + tag_family, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)
    else:
        cv2.putText(img, "tag_id: None", (10, img.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)
        cv2.putText(img, "tag_family: None", (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)
    
    return img

if __name__ == '__main__':
    from common.ros_robot_controller_sdk import Board
    from sensor.ultrasonic_sensor import Ultrasonic
    from common.action_group_controller import ActionGroupController
    import arm_ik.arm_move_ik as AMK


    board = Board()
    ik = kinematics.IK(board)  # 实例化逆运动学库(instantiate inverse kinematics library)
    ultrasonic = Ultrasonic()
    agc = ActionGroupController(board)
    ak = AMK.ArmIK()
    #加载参数(load parameters)
    param_data = np.load(calibration_param_path + '.npz')

    #获取参数(obtain parameters)
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
