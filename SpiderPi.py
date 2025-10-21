#!/usr/bin/python3
# coding=utf8
import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
import cv2
import time
import queue
import logging
import threading
import rpc_server
import mjpg_Server
import numpy as np
from common import kinematics
from sensor.ultrasonic_sensor import Ultrasonic
import functions.Running as Running
import functions.avoidance as Avoidance
import functions.lab_adjust as lab_adjust
import functions.color_track as ColorTrack
import functions.face_detect as FaceDetect
import functions.color_detect as ColorDetect
import functions.visual_patrol as VisualPatrol
import functions.remote_control as RemoteControl
import functions.apriltag_detect as ApriltagDetect
from calibration.camera import Camera
from common.ros_robot_controller_sdk import Board
from common.action_group_controller import ActionGroupController
import arm_ik.arm_move_ik as AMK
# 主线程，已经以后台的形式开机自启(Main thread, already started on boot as a background process)
# 自启方式systemd，自启文件/etc/systemd/system/spiderpi.service(Systemd auto-start method, auto-start file /etc/systemd/system/spiderpi.service)
# sudo systemctl stop spiderpi  此次关闭(close this time)
# sudo systemctl disable spiderpi 永久关闭(close permanently)
# sudo systemctl enable spiderpi 永久开启(start permanently)
# sudo systemctl start spiderpi 此次开启(enable this time)
board = Board()
agc = ActionGroupController(board)
ik = kinematics.IK(board)  # 实例化逆运动学库(instantiate inverse kinematics library)
ak = AMK.ArmIK()

ultrasonic = Ultrasonic()
QUEUE_RPC = queue.Queue(10)
ultrasonic = Ultrasonic()  #超声波传感器(ultrasonic sensor)
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
def startSpiderPi():
    RemoteControl.ik = kinematics.IK(board) 
    RemoteControl.agc = ActionGroupController(board)
    RemoteControl.ak = AMK.ArmIK()
    
    Avoidance.board = board
    Avoidance.ik = kinematics.IK(board) 

    ColorTrack.board = board

    lab_adjust.board = board

    ColorDetect.board = board
    ColorDetect.agc = ActionGroupController(board)
    ColorDetect.ik = kinematics.IK(board) 

    FaceDetect.board = board
    FaceDetect.agc = ActionGroupController(board)
    FaceDetect.ik = kinematics.IK(board) 
    FaceDetect.ak = AMK.ArmIK()

    VisualPatrol.board = board
    VisualPatrol.agc = ActionGroupController(board)
    VisualPatrol.ik = kinematics.IK(board) 
    VisualPatrol.ak = AMK.ArmIK()
    
    ApriltagDetect.board = board
    ApriltagDetect.ik = kinematics.IK(board) 
    ApriltagDetect.agc = ActionGroupController(board)
    ApriltagDetect.ak = AMK.ArmIK()
    
    rpc_server.board = board
    rpc_server.QUEUE = QUEUE_RPC
    rpc_server.QUEUE = QUEUE_RPC
    rpc_server.ultrasonic = ultrasonic
    rpc_server.Running = Running
    rpc_server.face_detect = FaceDetect
    rpc_server.lab_adjust = lab_adjust
    rpc_server.color_track = ColorTrack
    rpc_server.color_detect = ColorDetect   
    rpc_server.visual_patrol = VisualPatrol
    rpc_server.RemoteControl = RemoteControl
    rpc_server.apriltag_detect = ApriltagDetect
    rpc_server.avoidance = Avoidance
    Running.FUNCTIONS[1] = RemoteControl
    Running.FUNCTIONS[2] = ColorDetect
    Running.FUNCTIONS[3] = ColorTrack
    Running.FUNCTIONS[4] = VisualPatrol
    Running.FUNCTIONS[5] = FaceDetect
    Running.FUNCTIONS[6] = ApriltagDetect
    Running.FUNCTIONS[7] = Avoidance
    Running.FUNCTIONS[9] = lab_adjust

    Avoidance.ultrasonic = ultrasonic
    ColorTrack.ultrasonic = ultrasonic
    FaceDetect.ultrasonic = ultrasonic
    ColorDetect.ultrasonic = ultrasonic
    VisualPatrol.ultrasonic = ultrasonic
    RemoteControl.ultrasonic = ultrasonic
    ApriltagDetect.ultrasonic = ultrasonic
    
    RemoteControl.init()

    threading.Thread(target=rpc_server.startRPCServer,
                     daemon=True).start()  # rpc服务器(rpc server)
    threading.Thread(target=mjpg_Server.startMjpgServer,
                     daemon=True).start()  # mjpg流服务器(mjpg stream server)
    
    loading_picture = cv2.imread('/home/pi/spiderpi/functions/loading.jpg')
    cam = Camera()  # 相机读取(camera read)
    
    Running.cam = cam
    
    while True:
        time.sleep(0.03)

        # 执行需要在本线程中执行的RPC命令(execute the RPC command to be performed in this thread)
        while True:
            try:
                req, ret = QUEUE_RPC.get(False)
                event, params, *_ = ret
                ret[2] = req(params)  # 执行RPC命令(execute RPC command)
                event.set()
            except BaseException as e:
                #print(e)
                break
        #####
        # 执行功能玩法程序：(execute function game program)
        try:
            if Running.RunningFunc > 0 and Running.RunningFunc <= 9:
                if cam.frame is not None:
                    frame = cam.frame.copy() # 畸变矫正(distortion correction)
                    img = Running.CurrentEXE().run(frame)
                    if Running.RunningFunc == 9:
                        mjpg_Server.img_show = np.vstack((img, frame))
                    else:
                        mjpg_Server.img_show = img
                else:
                    mjpg_Server.img_show = loading_picture
            else:
                cam.frame = None
        except KeyboardInterrupt:
            break
        except BaseException as e:
            print(e)

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)
    startSpiderPi()
    
