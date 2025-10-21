#!/usr/bin/python3
# coding=utf8
import os
import sys
import time
import math
import copy
import logging
import threading
import queue
from werkzeug.serving import run_simple
from werkzeug.wrappers import Request, Response
from jsonrpc import JSONRPCResponseManager, dispatcher
from calibration.camera import Camera
from common.ros_robot_controller_sdk import Board
from sensor.ultrasonic_sensor import Ultrasonic
from common.action_group_controller import ActionGroupController
from action_group_dict import action_group_dict
from common import kinematics
import functions.avoidance as Avoidance
import functions.lab_adjust as lab_adjust
import functions.color_track as ColorTrack
import functions.face_detect as FaceDetect
import functions.color_detect as ColorDetect
import functions.visual_patrol as VisualPatrol
import functions.remote_control as RemoteControl
import functions.apriltag_detect as ApriltagDetect
import functions.Running as Running
import arm_ik.arm_move_ik as AMK
#import extend.self_transport as Transport
#import extend.pick_action as Pick
# 远程调用api，框架jsonrpc(remotely call API using the JSON-RPC framework)
# 主要用于手机端和电脑端的客户端调用(this is mainly used for client calls from mobile and desktop devices)

board = Board()
AGC = ActionGroupController(board)
ik = kinematics.IK(board)  # 实例化逆运动学库(instantiate inverse kinematics library)
ultrasonic = Ultrasonic()
ak = AMK.ArmIK()
QUEUE_RPC = queue.Queue(10)
ak.setPitchRangeMoving((0, 15, 30), 0, -90, 100, 2)
board.set_buzzer(2400, 0.1, 0.9, 1)
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

__RPC_E01 = "E01 - Invalid number of parameter!"
__RPC_E02 = "E02 - Invalid parameter!"
__RPC_E03 = "E03 - Operation failed!"
__RPC_E04 = "E04 - Operation timeout!"
__RPC_E05 = "E05 - Not callable"

old_time = time.time()
@dispatcher.add_method
def SetArmCoord(*args):
    global old_time
    ret = (True, (), 'SetArmCoord')
    try:
        x = round(args[0],1)
        y = round(args[1],1)
        z = round(args[2],1)
        print(x,y,z)
        use_times = 500
        angle = -60
        if time.time() - old_time >= use_times/1000:
            ak.setPitchRangeMoving((x, y, z), angle, -90, 100, use_times)
            print(f"当前位置: { ak.setPitchRangeMoving}, angle: {angle}, use_times: {use_times}, ")
            old_time = time.time()
        
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'SetArmCoord')
    return ret

@dispatcher.add_method
def SetArmReset():
    global old_time
    ret = (True, (), 'SetArmReset')
    print('SetArmReset')
    try:
        angle = 0
        use_times = 1500
        if time.time() - old_time >= use_times/1000:
            board.bus_servo_set_position(int(use_times/2), [[25, 500]])
            ak.setPitchRangeMoving((0, 10, 30), angle, -90, 100, use_times)
            old_time = time.time()
        
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'SetArmReset')
    return ret

@dispatcher.add_method
def SetPWMServo(*args, **kwargs):
    ret = (True, (), 'SetPWMServo')
    arglen = len(args)
    if 0 != (arglen % 2):
        return (False, __RPC_E01, 'SetPWMServo')
    try:
        servos = args[2:arglen:2]
        pulses = args[3:arglen:2]
        use_times = args[0]
        for s in servos:
            if s < 1 or s > 2:
                return (False, __RPC_E02, 'SetPWMServo')
        dat = zip(servos, pulses)
        for (s, p) in dat:
            board.set_pwm_servo_pulse(s, p, use_times)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'SetPWMServo')
    return ret

@dispatcher.add_method
def SetBusServoPulse(*args, **kwargs):
    ret = (True, (), 'SetBusServoPulse')   
    arglen = len(args)
    if (args[1] * 2 + 2) != arglen or arglen < 4:
        return (False, __RPC_E01, 'SetBusServoPulse')
    try:
        servos = args[2:arglen:2]
        pulses = args[3:arglen:2]
        use_times = args[0]
        for s in servos:
           if s < 1 or s > 25:
                return (False, __RPC_E02)
        dat = zip(servos, pulses)
        for (s, p) in dat:
            board.bus_servo_set_position(use_times/1000, [[s, p]])
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'SetBusServoPulse')
    return ret

@dispatcher.add_method
def SetBusServoDeviation(*args):
    ret = (True, (), 'SetBusServoDeviation')
    arglen = len(args)
    if arglen != 2:
        return (False, __RPC_E01, 'SetBusServoDeviation')
    try:
        servo = args[0]
        deviation = args[1]
        board.bus_servo_set_position(servo, deviation)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'SetBusServoDeviation')
    return ret

@dispatcher.add_method
def GetBusServosDeviation(args):
    ret = (True, (), 'GetBusServosDeviation')
    data = []
    if args != "readDeviation":
        return (False, __RPC_E01, 'GetBusServosDeviation')
    try:
        for i in range(1, 7):
            dev = board.get_bus_servo_deviation(i)
            if dev is None:
                dev = 999
            data.append(dev)
        ret = (True, data, 'GetBusServosDeviation')
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'GetBusServosDeviation')
    return ret 

@dispatcher.add_method
def SaveBusServosDeviation(args):
    ret = (True, (), 'SaveBusServosDeviation')
    if args != "downloadDeviation":
        return (False, __RPC_E01, 'SaveBusServosDeviation')
    try:
        for i in range(1, 7):
            dev = board.save_bus_servo_deviation(i)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'SaveBusServosDeviation')
    return ret 

@dispatcher.add_method
def UnloadBusServo(args):
    ret = (True, (), 'UnloadBusServo')
    if args != 'servoPowerDown':
        return (False, __RPC_E01, 'UnloadBusServo')
    try:
        for i in range(1, 7):
            board.unload_bus_servo(i)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'UnloadBusServo')
    return ret

@dispatcher.add_method
def GetBusServosPulse(args):
    ret = (True, (), 'GetBusServosPulse')
    data = []
    if args != 'angularReadback':
        return (False, __RPC_E01, 'GetBusServosPulse')
    try:
        for i in range(1, 7):
            pulse = board.get_bus_servo_pulse(i)
            if pulse is None:
                ret = (False, __RPC_E04, 'GetBusServosPulse')
                return ret
            else:
                data.append(pulse)
        ret = (True, data, 'GetBusServosPulse')
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'GetBusServosPulse')
    return ret 

@dispatcher.add_method
def StopActionGroup():
    ret = (True, (), 'StopActionGroup')
    try:     
        AGC.stop_action_group()
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'StopActionGroup')
    return ret

servo_move = False

th1 = None
th2 = None
th3 = None
th4 = None
have_move = True
control_lock = False
@dispatcher.add_method
def Move(*args):
    global th1
    global have_move
    
    ret = (True, (), 'Move') 

    print('args:', args)
    mode = int(args[0])
    movement_direction = float(args[1])
    rotation = float(args[2])
    #speed = speed = float(1000 * (10 / int(400)))
    speed = float(1000 * (8 / int(args[3])))
    times = int(args[4])
    
    if th2 is not None:
        if th2.is_alive():
            return ret
    if th3 is not None:
        if th3.is_alive():
            return ret
    if th4 is not None:
        if th4.is_alive():
            return ret

    if control_lock:
        return ret

    if len(args) != 5:
        return (False, __RPC_E01, 'Move')
    try:
        if times == 1:
            if have_move:
                ik.stopMove()
                have_move = False
        else:
            if th1 is None:
                th1 = threading.Thread(target=ik.move, args=(ik.current_pos, mode, 80, movement_direction, -rotation, speed, times))
                print(f"当前位置: {ik.current_pos}, 模式: {mode}, 占位符: 80, 移动方向: {movement_direction}, 旋转角度: {rotation}, 速度: {speed}, 次数: {times}")
                th1.start()
                have_move = True
            else:
                if not th1.is_alive():
                    th1 = threading.Thread(target=ik.move, args=(ik.current_pos, mode, 80, movement_direction, rotation, speed, times))
                    th1.start()
                    have_move = True

    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'Move')
    return ret



@dispatcher.add_method
def Transport(args):
    global th4
    
    ret = (True, (), 'Transport')
    if th1 is not None:
        if th1.is_alive():
            return ret

    if th4 is None:
        th4 = threading.Thread(target=Pick.pick, args=(int(args), 2))
        th4.start()
    else:
        if not th4.is_alive():
            th4 = threading.Thread(target=Pick.pick, args=(int(args), 2))
            th4.start()
    return ret

@dispatcher.add_method
def Stand(*args):
    global th3
    
    ret = (True, (), 'Stand')
    
    if th4 is not None:
        if th4.is_alive():
            return ret
    if th2 is not None:
        if th2.is_alive():
            return ret
    if th1 is not None:
        if th1.is_alive():
            return ret
    
    if control_lock:
        return ret
    
    height = int(args[0])
    mode = int(args[1])
    t = int(args[2])

    if len(args) != 3:
        return (False, __RPC_E01, 'Stand')
    try:
        if t == 800:
            if mode == 2:
                pos = copy.deepcopy(ik.initial_pos)
            else:
                pos = copy.deepcopy(ik.initial_pos_quadruped)
        else:
            if mode == 2:
                if height > 160:
                    ik.current_pos = copy.deepcopy(ik.initial_pos_high)
                else:
                    ik.current_pos = copy.deepcopy(ik.initial_pos)
            else:
                ik.current_pos = copy.deepcopy(ik.initial_pos_quadruped)
            pos = ik.current_pos

        if mode == 2:
            for i in range(6):
                pos[i][2] = -float(height)
        if th3 is None:
            th3 = threading.Thread(target=ik.stand, args=(pos, t))
            th3.start()
        else:
            if not th3.is_alive():
                th3 = threading.Thread(target=ik.stand, args=(pos, t))
                th3.start()
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'Stand')
    return ret

have_run = True
@dispatcher.add_method
def RunAction(*args):
    global th2
    global have_run

    ret = (True, (), 'RunAction')

    actName = '0'
    times = 1

    if th3 is not None:
        if th3.is_alive():
            return ret
    if th1 is not None:
        if th1.is_alive():
            return ret
    
    if control_lock:
        return ret

    if len(args) != 2:
        return (False, __RPC_E01, 'RunAction')
    try:
        if args[0] == '0':
            if have_run:
                AGC.stopAction()
                have_run = False
        else:
            if th2 is not None:
                if not th2.is_alive():
                    if args[0] in action_group_dict:
                        actName = action_group_dict[args[0]]
                    else:
                        actName = args[0]
                    times = int(args[1])
                    th2 = threading.Thread(target=AGC.run_action, args=(actName,))
                    th2.start()
                    have_run = True
            else:
                if args[0] in action_group_dict:
                    actName = action_group_dict[args[0]]
                else:
                    actName = args[0]
                times = int(args[1])
                th2 = threading.Thread(target=AGC.run_action, args=(actName, ))
                th2.start()
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'RunAction')
    return ret

unload = True
stand_up_on = False
def stand_up():
    board = Board()
    board.enable_reception()
    global control_lock

    count1 = 0
    count2 = 0
    while True:
        if not unload and stand_up_on:
            try:
                data = board.get_imu() #获取传感器值(obtain sensor value)
                #print(data)
                angle_x = int(math.degrees(math.atan2(data[0], data[2]))) #转化为角度值(convert to angle value)
                if abs(angle_x) > 130:
                    count1 += 1
                else:
                    count1 = 0
                if abs(angle_x) < 50:
                    count2 += 1
                else:
                    count2 = 0
                time.sleep(0.1)
                if count1 >= 3:  #往后倒了一定时间后起来(Fall backwards for a certain amount of time, then stand up)
                    count1 = 0
                    if not control_lock:
                        control_lock = True
                        AGC.run_action('stand_flip')
                elif count2 >= 3:  #后前倒了一定时间后起来(Fall backwards for a certain amount of time, then stand up)
                    count2 = 0
                    if control_lock:
                        control_lock = False
                        ik.stand(ik.current_pos, t=500)
            except BaseException as e:
                print(e)
        else:
            control_lock = False
            count1 = 0
            count2 = 0
            time.sleep(0.01)

threading.Thread(target=stand_up).start()

@dispatcher.add_method
def PostureDetect(args):
    global stand_up_on
    
    ret = (True, (), 'PostureDetect')
    board.set_buzzer(1900, 0.1, 0.9, 1)
    stand_up_on = int(args)
    
    return ret

def runbymainth(req, pas):
    if callable(req):
        event = threading.Event()
        ret = [event, pas, None]
        QUEUE.put((req, ret))
        count = 0
        #ret[2] =  req(pas)
        #print('ret', ret)
        while ret[2] is None:
            time.sleep(0.01)
            count += 1
            if count > 200:
                break
        if ret[2] is not None:
            if ret[2][0]:
                return ret[2]
            else:
                return (False, __RPC_E03 + " " + ret[2][1])
        else:
            return (False, __RPC_E04)
    else:
        return (False, __RPC_E05)

@dispatcher.add_method
def LoadFunc(new_func=0):
    global unload
    global stand_up_on
    global control_lock

    control_lock = False
    if new_func == 1:
        stand_up_on = False
        unload = False
    return runbymainth(Running.loadFunc, (new_func, ))

@dispatcher.add_method
def UnloadFunc():
    global unload
    global control_lock
    
    control_lock = False
    unload = True
    return runbymainth(Running.unloadFunc, ())

@dispatcher.add_method
def StartFunc():
    return runbymainth(Running.startFunc, ())

@dispatcher.add_method
def StopFunc():
    return runbymainth(Running.stopFunc, ())

@dispatcher.add_method
def FinishFunc():
    return runbymainth(Running.finishFunc, ())

@dispatcher.add_method
def Heartbeat():
    return runbymainth(Running.doHeartbeat, ())

@dispatcher.add_method
def GetRunningFunc():
    return (True, (0,))

@dispatcher.add_method
def SetTargetTrackingColor(*target_color):
    return runbymainth(ColorTrack.setTargetColor, target_color)

@dispatcher.add_method
def SetVisualPatrolColor(*target_color):
    return runbymainth(VisualPatrol.setLineTargetColor, target_color)

@dispatcher.add_method
def SetSonarDistanceThreshold(new_threshold=40):
    return runbymainth(Avoidance.setThreshold, (new_threshold,))

@dispatcher.add_method
def SetSonarRGB(index, r, g, b):
    global ultrasonic

    if index == 0:
        ultrasonic.setRGBMode(0)
        ultrasonic.setRGB(1, (r, g, b))
        ultrasonic.setRGB(2, (r, g, b))
    else:
        ultrasonic.setRGB(index, (r, g, b))
    return (True, (r, g, b))

# 设置颜色阈值(set color threshold)
# 参数：颜色lab(parameter: color lab)
# 例如(for example)：[{'red': ((0, 0, 0), (255, 255, 255))}]
@dispatcher.add_method
def SetLABValue(*lab_value):
    #print(lab_value)
    return runbymainth(lab_adjust.setLABValue, lab_value)

# 保存颜色阈值(save color threshold)
@dispatcher.add_method
def GetLABValue():
    return (True, lab_adjust.getLABValue()[1], 'GetLABValue')

# 保存颜色阈值(save color threshold)
@dispatcher.add_method
def SaveLABValue(color=''):
    return runbymainth(lab_adjust.saveLABValue, (color, ))

@dispatcher.add_method
def HaveLABAdjust():
    return (True, True, 'HaveLABAdjust')

@Request.application
def application(request):
    dispatcher["echo"] = lambda s: s
    dispatcher["add"] = lambda a, b: a + b
    response = JSONRPCResponseManager.handle(request.data, dispatcher)
    
    return Response(response.json, mimetype='application/json')

def startRPCServer():
    run_simple('', 9030, application)

if __name__ == '__main__':
    startRPCServer()
