#!/usr/bin/env python3

# websocket中继广播下行客户端，连接至服务down通道，接受下行数据并做处理(A WebSocket relay broadcast downstream client that connects to the 'down' channel of the server, receives downstream data, and processes it)

import os
import time
import asyncio
import logging
import websockets
from functools import partial
from concurrent.futures import ThreadPoolExecutor
from jsonrpc import JSONRPCResponseManager, Dispatcher
from common.ros_robot_controller_sdk import Board
from common import kinematics
from common.action_group_controller import ActionGroupController
import robot_dance as dance

board = Board()
ik = kinematics.IK(board)  # 实例化逆运动学库(instantiate inverse kinematics library)
agc = ActionGroupController(board)

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("multi_control_client")
logger.setLevel(level=logging.WARNING)

executor = ThreadPoolExecutor()
dispatcher = Dispatcher()


@dispatcher.add_method
def run_action_set(action_name, repeat):
    if os.path.exists("/dev/input/js0") is True:
        time.sleep(0.01)
    else:
        agc.run_action_group(action_name, repeat)

@dispatcher.add_method
def stop():
    agc.stopActionGroup()
    
@dispatcher.add_method
def kinematics_control(name):
    if os.path.exists("/dev/input/js0") is False:
       
        if name == 'go_forward':
            print('go_forward')
            ik.go_forward(ik.initial_pos, 2, 50, 80, 1)
        elif name == 'back':
            print('back')
            ik.back(ik.initial_pos, 2, 50, 80, 1)
        elif name == 'right_move':
            print('right_move')
            ik.right_move(ik.initial_pos, 2, 50, 80, 1)
        elif name == 'left_move':
            print('left_move')
            ik.left_move(ik.initial_pos, 2, 50, 80, 1)
        elif name == 'turn_right':
            print('turn_right')
            ik.turn_right(ik.initial_pos, 2, 20, 100, 1) 
        elif name == 'turn_left':
            print('turn_left')
            ik.turn_left(ik.initial_pos, 2, 20, 100, 1)
        elif name == 'stand':
            print('stand')
            ik.stand(ik.initial_pos, 500)
        elif name == 'dance':
            print('dance')
            dance.dance()

async def listener():
    while True:
        try:
            websocket = await websockets.connect('ws://192.168.149.1:7788/down')
            async for msg in websocket:
                logger.debug(msg)
                asyncio.ensure_future(
                    loop.run_in_executor(executor, partial(JSONRPCResponseManager.handle, dispatcher=dispatcher), msg))
        except Exception as e:
            logger.error(e)
        await asyncio.sleep(2)


loop = asyncio.get_event_loop()
asyncio.run_coroutine_threadsafe(listener(), loop)
loop.run_forever()
