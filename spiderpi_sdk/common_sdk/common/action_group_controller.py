#!/usr/bin/env python3
# encoding: utf-8
import os
import time
import sqlite3 as sql

class ActionGroupController:
    runningAction = False
    stopRunning = False
    servos_id = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 22, 23, 24,25]
    def __init__(self, board, action_path='/home/pi/spiderpi/'):
        self.action_path = action_path
        self.board = board

    def stop_servo(self):
        self.board.bus_servo_stop(self.servos_id) 
            
    def run_action_group(self, actNum, times=1,lock_servos=''):
        global stop_action
        global stop_action_group

        stop_action = False

        temp = times
        while True:
            if temp != 0:
                times -= 1
                if times < 0 or self.stop_action_group: # 跳出循环(jump out the loop)
                    self.stop_action_group = False
                    break
                self.run_action(actNum,lock_servos)
            elif temp == 0:
                if self.stop_action_group: # 跳出循环(jump out the loop)
                    self.top_action_group = False
                    break
                self.run_action(actNum,lock_servos)
            
    def stop_action_group(self):
        self.stopRunning = True


    def run_action(self, actNum,lock_servos=''):        
        if actNum is None:
            return
        actNum = os.path.join(self.action_path, 'action_groups', actNum + ".d6a")
        self.stopRunning = False
        if os.path.exists(actNum) is True:
            if self.runningAction is False:
                self.runningAction = True
                ag = sql.connect(actNum)
                cu = ag.cursor()
                cu.execute("select * from ActionGroup")
                
                while True:
                    act = cu.fetchone()
                    if self.stopRunning is True:
                        self.stopRunning = False                   
                        break

                    if act is not None:
                        data = []
                        for i in range(0, len(act) - 2, 1):
                            if str(self.servos_id[i])  in lock_servos:
                                data.extend([[self.servos_id[i], lock_servos[str(self.servos_id[i])]]])  
                            else:
                                data.extend([[self.servos_id[i], act[2 + i]]])
                        self.board.bus_servo_set_position(float(act[1])/1000.0, data)
                        
                        for j in range(int(act[1]/50)):
                            if self.stopRunning:
                                self.stop_action_group = True
                                break
                            time.sleep(0.05)
                        time.sleep(0.001 + act[1]/1000.0 - 0.05*int(act[1]/50))

                    else:
                        break
                self.runningAction = False
                
                cu.close()
                ag.close()
        else:
            self.runningAction = False
            print( self.action_path)





    
