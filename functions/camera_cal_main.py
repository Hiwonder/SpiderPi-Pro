#!/usr/bin/python3
# coding=utf8
import os
import cv2
import yaml
import time
import threading
import numpy as np
from common import apriltag
from calibration.camera import Camera
from calibration.CalibrationConfig import *
from common import kinematics
from common.ros_robot_controller_sdk import Board
import arm_ik.arm_move_ik as AMK
    
#加载参数(load parameters)
param_data = np.load(calibration_param_path + '.npz')
#获取参数(obtain parameters)
mtx = param_data['mtx_array']
dist = param_data['dist_array']
newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))


board = Board()
ik = kinematics.IK(board)
ak = AMK.ArmIK()


at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))
marker_corners = np.asarray([[0, 0, 40],
                             [16.65, -16.65, 40],  # TAG_SIZE = 33.30mm
                             [-16.65, -16.65, 40],
                             [-16.65, 16.65, 40],
                             [16.65, 16.65, 40]],
                             dtype=np.float64)
marker_corners_block = np.copy(marker_corners)
marker_corners_block[:, 2] = marker_corners_block[:, 2] - 40

class State:
    def __init__(self):
        self.lock = threading.RLock()
        self.D = dist
        self.K = newcameramtx
        self.R = None
        self.T = None
        self.R_40 = None
        self.T_40 = None

    def reset(self):
        self.R = None
        self.T = None
        self.R_40 = None
        self.T_40 = None

# 初始位置(initial position)
def initMove():
    ik.stand(ik.initial_pos)
    board.bus_servo_set_position(1, [[25, 650]])
    ak.setPitchRangeMoving((0, 15, 5), -90, -90, 100, 2)


def camera_to_world(cam_mtx, r, t, img_points):
    inv_k = np.asmatrix(cam_mtx).I
    r_mat = np.zeros((3, 3), dtype=np.float64)
    cv2.Rodrigues(r, r_mat)
    # invR * T
    inv_r = np.asmatrix(r_mat).I  # 3*3
    transPlaneToCam = np.dot(inv_r, np.asmatrix(t))  # 3*3 dot 3*1 = 3*1
    world_pt = []
    coords = np.zeros((3, 1), dtype=np.float64)
    for img_pt in img_points:
        coords[0][0] = img_pt[0][0]
        coords[1][0] = img_pt[0][1]
        coords[2][0] = 1.0
        worldPtCam = np.dot(inv_k, coords)  # 3*3 dot 3*1 = 3*1
        # [x,y,1] * invR
        worldPtPlane = np.dot(inv_r, worldPtCam)  # 3*3 dot 3*1 = 3*1
        # zc
        scale = transPlaneToCam[2][0] / worldPtPlane[2][0]
        # zc * [x,y,1] * invR
        scale_worldPtPlane = np.multiply(scale, worldPtPlane)
        # [X,Y,Z]=zc*[x,y,1]*invR - invR*T
        worldPtPlaneReproject = np.asmatrix(scale_worldPtPlane) - np.asmatrix(transPlaneToCam)  # 3*1 dot 1*3 = 3*3
        pt = np.zeros((3, 1), dtype=np.float64)
        pt[0][0] = worldPtPlaneReproject[0][0]
        pt[1][0] = worldPtPlaneReproject[1][0]
        pt[2][0] = 0
        world_pt.append(pt.T.tolist())
    return world_pt


def run(img):
    frame_gray = cv2.cvtColor(np.copy(img), cv2.COLOR_RGB2GRAY)
    tags = at_detector.detect(frame_gray)
    for tag in tags:
        center = np.array(tag.center).astype(int)
        if tag.tag_id == 1:
            corners = tag.corners.reshape(1, -1, 2).astype(np.float32)
            pts = np.insert(corners[0], 0, values=tag.center, axis=0)
            with state.lock:
                rtl, new_r, new_t = cv2.solvePnP(marker_corners, pts, state.K, None)
                if rtl:
                    state.R = new_r if state.R is None else state.R * 0.95 + new_r * 0.05
                    state.T = new_t if state.T is None else state.T * 0.95 + new_t * 0.05
                rtl, new_r, new_t = cv2.solvePnP(marker_corners_block, pts, state.K, None)
                if rtl:
                    state.R_40 = new_r if state.R_40 is None else state.R_40 * 0.95 + new_r * 0.05
                    state.T_40 = new_t if state.T_40 is None else state.T_40 * 0.95 + new_t * 0.05
                if state.R is not None:
                    img_pts, jac = cv2.projectPoints(marker_corners, state.R, state.T, state.K, None)
                else:
                    img_pts = []
            corners = corners.astype(int)
            
            w = camera_to_world(state.K, state.R_40, state.T_40,
                                        center.reshape((1, 1, 2)))[0][0]
            print(center.reshape((1, 1, 2)))
            cv2.circle(img, tuple(corners[0][0]), 5, (255, 0, 0), 12)
            cv2.circle(img, tuple(corners[0][1]), 5, (0, 255, 0), 12)
            cv2.circle(img, tuple(corners[0][2]), 5, (0, 0, 255), 12)
            cv2.circle(img, tuple(corners[0][3]), 5, (0, 255, 255), 12)
            cv2.circle(img, tuple(center), 5, (255, 255, 0), 12)
            cv2.putText(img, "id:%d" % tag.tag_id,
                        (center[0], center[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            for i, c in enumerate(img_pts):
                cv2.circle(img, tuple(c.astype(int)[0]), 3, (0, 0, 0), 4)
        else:
            with state.lock:
                if state.R_40 is not None:
                    
                    w = camera_to_world(state.K, state.R_40, state.T_40,
                                        center.reshape((1, 1, 2)))[0][0]
                else:
                    w = 0, 0, 0
            cv2.putText(img, "id:{} x:{:.2f} y:{:.2f}".format(tag.tag_id, w[0], w[1]),
                        (center[0] - 20, center[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
    return img


def save_cb():
    with state.lock:
        with state.lock:
            card_params = {
                "K": state.K.tolist(),
                "R": state.R.tolist(),
                "T": state.T.tolist()
            }
            block_params = {
                "K": state.K.tolist(),
                "R": state.R_40.tolist(),
                "T": state.T_40.tolist()
            }
            s = yaml.dump({
                'card_params': card_params,
                'block_params': block_params,
            }, default_flow_style=True)
        with open('/home/pi/spiderpi/config/camera_cal.yaml', 'w') as f:
            f.write(s)
          
    return [True, '']

def up_cb():
    ak.setPitchRangeMoving((0, 15, 5), -90, -90, 100, 2)
    return [True, '']

def down_cb():
    ak.setPitchRangeMoving((0, 15, -2), -90, -90, 100, 2)
    return [True, '']




def move():
    time.sleep(3)
    board.set_buzzer(2400, 0.1, 0.4, 1)
    down_cb()
    time.sleep(10)
    board.set_buzzer(2400, 0.1, 0.4, 2)
    up_cb()
    time.sleep(15)
    board.set_buzzer(2400, 0.1, 0.4, 3)
    save_cb()

# 运行子线程(run sub-thread)
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

if __name__ == '__main__':
    
    #mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)
    
    debug = False
    if debug:
        print('Debug Mode')
        
    initMove()
    state = State()
    camera = Camera()
    camera.camera_open()
    while True:
        img = camera.frame
        if img is not None:
            frame = img.copy()
            #frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)  # 畸变矫正(distortion correction)
            Frame = run(frame)           
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    camera.camera_close()
    cv2.destroyAllWindows()   
    
    