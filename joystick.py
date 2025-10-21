#!/usr/bin/python3
# coding=utf8
import os
import time
import json
import pygame
import asyncio
import threading
import websockets
from common.ros_robot_controller_sdk import Board
from common import kinematics
from common.action_group_controller import ActionGroupController
import functions.robot_dance as dance

#ps2手柄控制动作, 已以system方式自启动，无需再开启(Control the actions with a PS2 joystick. The program has been set to automatically start with the system. There is no need to manually start it)

key_map = {"PSB_CROSS":0, "PSB_CIRCLE":1, "PSB_SQUARE":3, "PSB_TRIANGLE":4,
        "PSB_L1":6, "PSB_R1":7, "PSB_L2":8, "PSB_R2":9,
        "PSB_SELECT":10, "PSB_START":11}
action_map = ["CROSS", "CIRCLE", "", "SQUARE", "TRIANGLE", "L1", "R1", "L2", "R2", "SELECT", "START", "", "L3", "R3"]

board = Board()
agc = ActionGroupController(board)
ik = kinematics.IK(board)

ik.stand(ik.initial_pos) # 立正(stand at attention)
servo21,servo22,servo23,servo24,servo25 = 500,705,90,330,700 
board.bus_servo_set_position(1, [[21, servo21], [22, servo22], [23, servo23], [24, servo24], [25, servo25]])
time.sleep(1)
                    


def joystick_init():
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    pygame.display.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() > 0:
        js=pygame.joystick.Joystick(0)
        js.init()
        jsName = js.get_name()
        print("Name of the joystick:", jsName)
        jsAxes=js.get_numaxes()
        print("Number of axif:",jsAxes)
        jsButtons=js.get_numbuttons()
        print("Number of buttons:", jsButtons);
        jsBall=js.get_numballs()
        print("Numbe of balls:", jsBall)
        jsHat= js.get_numhats()
        print("Number of hats:", jsHat)
  
  
async def call_rpc(method, params=None):
    websocket = None
    try:
        websocket = await websockets.connect('ws://192.168.149.1:7788/up')
        call = dict(jsonrpc='2.0', method=method)
        if params is not None:
            call['params'] = params
        await websocket.send(json.dumps(call))
        await websocket.close()
    except Exception as e:
        if websocket is not None:
            await websocket.close()

async def run_action_set(action_set_name,repeat=1):
    await call_rpc('run_action_set', [action_set_name,repeat])

async def stop(action_set_name=None):
    await call_rpc('stop')
    if action_set_name is not None:
        await run_action_set(action_set_name,repeat=1)  


async def kinematics_control(name):
    await call_rpc('kinematics_control', [name])
     
  
mode = 'move'
stand_ti = time.time()
stand_st = False
th = None
last_status = ''
connected = False
while True:
    if os.path.exists("/dev/input/js0") is True:
        if connected is False:
            joystick_init()
            jscount =  pygame.joystick.get_count()
            if jscount > 0:
                
                try:
                    js=pygame.joystick.Joystick(0)
                    js.init()

                    connected = True
                except Exception as e:
                    print(e)
            else:
                pygame.joystick.quit()
    else:
        if connected is True:
            connected = False
            js.quit()
            pygame.joystick.quit()
    if connected is True:
        pygame.event.pump()     
        actName = None
        times = 1
        try:
            #print(pygame.joystick.get_count())
            if js.get_button(key_map["PSB_SELECT"]):
                
                time.sleep(0.08)
                if js.get_button(key_map["PSB_SELECT"]):
                    if mode == 'move':
                        board.set_buzzer(2400, 0.1, 0.4, 2)  # 以2400Hz的频率，0.1秒开始响，0.4秒停止响，重复2次(The buzzer sounds at a frequency of 2400Hz for 0.1 seconds, then stops for 0.4 seconds. Repeat this cycle twice)
                        time.sleep(0.1)
                        mode = 'arm'
                    else:
                        board.set_buzzer(2400, 0.1, 0.4, 1)  # 以2400Hz的频率，0.1秒开始响，0.4秒停止响，重复1次(The buzzer sounds at a frequency of 2400Hz for 0.1 seconds, then stops for 0.4 seconds. Repeat this cycle once)
                        mode = 'move'
                    time.sleep(1)
                
            if mode == 'move':
                rx = js.get_axis(2)
                ry = js.get_axis(3)
                if js.get_button(key_map["PSB_R1"]):
                    asyncio.run(kinematics_control('dance'))
                    dance.dance()
                if js.get_button(key_map["PSB_R2"]):
                    asyncio.run(kinematics_control('turn_right'))
                    ik.turn_right(ik.initial_pos, 2, 20, 100, 1)  # 原地右转30度(turn right in place for 30 degrees)
                    stand_st = True
                    stand_ti = time.time()
                if js.get_button(key_map["PSB_L2"]):
                    asyncio.run(kinematics_control('turn_left'))
                    ik.turn_left(ik.initial_pos, 2, 20, 100, 1)  # 原地左转30度(turn left in place for 30 degrees)
                    stand_st = True
                    stand_ti = time.time()
                if rx < -0.5 or js.get_button(key_map["PSB_SQUARE"]): #正方形(square)
                    actName = 'wave'
                if rx > 0.5 or js.get_button(key_map["PSB_CIRCLE"]): #圈(circle)
                    actName = 'kick'
                if ry < -0.5 or js.get_button(key_map["PSB_TRIANGLE"]): #三角(triangle)
                    actName = 'attack'
                if ry > 0.5 or js.get_button(key_map["PSB_CROSS"]): #叉(cross)
                    actName = 'twist'
                if js.get_button(key_map["PSB_START"]):
                    agc.stop_action_group()
                    time.sleep(0.5)
                    ik.stand(ik.initial_pos) # 立正(stand at attention)
                    
                lx = js.get_axis(0)
                ly = js.get_axis(1)
                hat = js.get_hat(0)
                if lx < -0.5 or hat==(-1,0):
                    asyncio.run(kinematics_control('left_move'))
                    ik.left_move(ik.initial_pos, 2, 50, 80, 1)  # 左移50mm(move left by 50mm)
                    stand_ti = time.time()
                    stand_st = True
                elif lx > 0.5 or hat==(1,0):
                    asyncio.run(kinematics_control('right_move'))
                    ik.right_move(ik.initial_pos, 2, 50, 80, 1)  # 右移50mm(move right by 50mm)
                    stand_ti = time.time()
                    stand_st = True
                if ly < -0.5 or hat==(0,1):
                    asyncio.run(kinematics_control('go_forward'))
                    ik.go_forward(ik.initial_pos, 2, 50, 80, 1)  # 朝前直走50mm(go forward by 50mm)
                    stand_ti = time.time()
                    stand_st = True
                elif ly > 0.5 or hat==(0,-1):
                    asyncio.run(kinematics_control('back'))
                    ik.back(ik.initial_pos, 2, 50, 80, 1)  # 朝后直走50mm(go back by 50mm)
                    stand_ti = time.time()
                    stand_st = True
                
                if stand_st and time.time()-stand_ti >= 0.1:
                    stand_st = False
                    stand_ti = time.time()
                    asyncio.run(kinematics_control('stand'))
                    ik.stand(ik.initial_pos, 500) # 立正(stand at attention)
                    
                if th is not None:
                    if actName is not None:
                        if not th.is_alive():
                            asyncio.run(run_action_set(actName,))
                            th = threading.Thread(target=agc.run_action, args=(actName,), daemon=True)
                            print("actName= ",actName)
                            th.start()
                else:
                    asyncio.run(run_action_set(actName,))
                    th = threading.Thread(target=agc.run_action, args=(actName,), daemon=True)
                    th.start()
                    
            elif mode == 'arm':
                rx = js.get_axis(2)
                ry = js.get_axis(3)
                if js.get_button(key_map["PSB_R2"]):
                    servo25 -= 5
                    servo25 = 200 if servo25 < 200 else servo25
                    board.bus_servo_set_position(0.03, [[25, servo25]])

                    time.sleep(0.03)
                if js.get_button(key_map["PSB_L2"]):
                    servo25 += 5
                    servo25 = 700 if servo25 > 700 else servo25
                    board.bus_servo_set_position(0.03, [[25, servo25]])

                    time.sleep(0.03)
                if rx < -0.5 or js.get_button(key_map["PSB_SQUARE"]): #正方形(square)
                    servo23 -= 5
                    board.bus_servo_set_position(0.09, [[23, servo23]])
                    time.sleep(0.09)
                if ry < -0.5 or js.get_button(key_map["PSB_TRIANGLE"]): #三角(triangle)
                    servo23 += 5
                    board.bus_servo_set_position(0.09, [[23, servo23]])
                    time.sleep(0.09)
                if rx > 0.5 or js.get_button(key_map["PSB_CIRCLE"]): #圈(circle)
                    servo24 += 5
                    board.bus_servo_set_position(0.09, [[24, servo24]])
                    time.sleep(0.09)
                if ry > 0.5 or js.get_button(key_map["PSB_CROSS"]): #叉(cross)
                    servo24 -= 5
                    board.bus_servo_set_position(0.09, [[24, servo24]])
                    time.sleep(0.09)
                if js.get_button(key_map["PSB_START"]):
                    servo21,servo22,servo23,servo24,servo25 = 500,705,90,330,700
                    board.bus_servo_set_position(1, [[21, servo21], [22, servo22], [23, servo23], [24, servo24], [25, servo25]])
                    time.sleep(1)
                    
                lx = js.get_axis(0)
                ly = js.get_axis(1)
                hat = js.get_hat(0)
                if lx < -0.5 or hat ==(-1,0):
                    servo21 += 5
                    board.bus_servo_set_position(0.03, [[21, servo21]])
                    time.sleep(0.03)
                elif lx > 0.5 or hat ==(1,0):             
                    servo21 -= 5
                    board.bus_servo_set_position(0.03, [[21, servo21]])
                    time.sleep(0.03)
                if ly < -0.5 or hat ==(0,1):
                    servo22 -= 5
                    board.bus_servo_set_position(0.03, [[22, servo22]])
                    time.sleep(0.03)
                elif ly > 0.5 or hat ==(0,-1):
                    servo22 += 5
                    board.bus_servo_set_position(0.03, [[22, servo22]])
                    time.sleep(0.03)
                    
        except Exception as e:
            print(e)
            connected = False          
    time.sleep(0.01)
