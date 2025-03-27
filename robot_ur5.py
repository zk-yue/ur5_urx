"""
机器人控制程序
"""
import time
import numpy as np
from math import pi
import matplotlib.pyplot as plt
import urx
import os
import scipy
import math3d as m3d
from scipy.io import loadmat
import copy
import time
from gripper import inspire_gripper
import hand_detection
from pynput import keyboard
import threading
from scipy.spatial.transform import Rotation as R
from matplotlib.pyplot import MultipleLocator
import cv2
v = 0.1
a = v*5
font = {
            'weight' : 'normal',
            'size'   : 13,
            }

class URRobot():
    lock = threading.Lock() #多线程锁
    # 任务阶段控制
    WAITFORBEGIN=0
    SEARCHGRASPPOINT=1
    GRASP=2
    COLLABWORK=3
    QUIT=4

    def __init__(self, velocity=0.1, acceleration=0.5 , need_bigdepth=False, need_color_depth_map=False):
        # 机器人设置
        ## 机器人home位置， 根据情况修改
        self.homePoseMatrix = np.array([
            [-0.9986, 0.0145, 0.0494, 0.2051],
            [-0.0494,-0.0017,-0.9987, 0.1059],
            [-0.0144,-0.9998, 0.0024, 1.0217]
        ])
        self.pose = [0.2051, 0.1059, 1.0217, -0.0382, 2.1869, -2.1915] # 与self.homePose.pose_vector相同
        self.homePosition = np.array([0.2051, 0.1059, 1.0217])
        self.homePose = m3d.Transform(self.homePoseMatrix)
        
        self.vel = velocity
        self.acc = acceleration
        ## 连接机器人

        self.ur5 = urx.Robot("192.168.56.2")
        ## 设置机器人世界坐标系
        vector = m3d.Vector(0, 1.089, 0.743)
        orientation = m3d.Orientation(m3d.Versor(0.383, 0.924, 0.0, 0.0)) # 绕 x-y 平面方向的旋转轴旋转 135 度 的方向。
        # orientation=array([[ 1.        ,  0.        ,  0.        ],
        #     [ 0.        , -0.70675836, -0.70745503],
        #     [ 0.        ,  0.70745503, -0.70675836]])>
        base2world = m3d.Transform(orientation, vector)
        self.ur5.set_csys(base2world)  #  Set reference coordinate system to use

        # 夹爪设置
        ## 连接手爪
        # self.gripper = inspire_gripper(com_port='/dev/ttyUSB0',baudrate=115200)
        ## 设置tcp位置
        vector = m3d.Vector(0, 0, 0.178) #tcp: 0.178, ftsensor: 0.038
        orientation = m3d.Orientation(m3d.Versor(1, 0, 0, 0))
        wrist2tcp = m3d.Transform(orientation, vector)
        self.ur5.set_tcp(wrist2tcp)

        self.robotState = self.WAITFORSTART # 轨迹记录过程中按键控制
        self.taskStage = self.WAITFORBEGIN # 任务执行阶段控制
        self.busy=0 # 机械臂工作中

    def close(self):
        self.ur5.close()
        # self.gripper.close()
    def robotDemonOnPress(self, key):
        if key.char == 'b' and self.robotState == self.WAITFORSTART:
            self.lock.acquire()
            self.robotState = self.CACHEPOS
            self.lock.release()
        elif key.char == 's' and self.robotState == self.CACHEPOS:
            self.lock.acquire()
            self.robotState = self.STOREPOS
            self.lock.release()
            return False
        elif key.char == 'q': 
            self.lock.acquire()
            self.robotState = self.QUIT
            self.lock.release()
            return False

    def taskStageChange(self, key):
        if key.char == 'u' : # and self.taskStage == self.WAITFORBEGIN:
            self.lock.acquire()
            self.taskStage = self.SEARCHGRASPPOINT
            self.busy=0
            self.lock.release()
            
        elif key.char == 'v' : #  and self.taskStage == self.SEARCHGRASPPOINT:
            self.lock.acquire()
            self.taskStage = self.GRASP
            self.busy=0
            self.lock.release()
            
        elif key.char == 'w' : #  and self.taskStage == self.GRASP: 
            self.lock.acquire()
            self.taskStage = self.COLLABWORK
            self.busy=0
            self.lock.release()
            
        elif key.char == 'x': 
            self.lock.acquire()
            self.taskStage = self.QUIT
            self.busy=0
            self.lock.release()
            

    def demoRobotPositionTraj(self, path="robotTrajectory.txt"):
        # 监听按键 'b' 开始记录， 's' 保存，'q' 放弃
        print("demostrate robot trajectory waiting for command")
        listener = keyboard.Listener(on_press=self.robotDemonOnPress) #构造类 on_press=按下按键时触发的函数
        listener.start()
        beginTime = -1.0
        robotTraj = np.zeros([0, 4])
        while True:
            if self.robotState == self.CACHEPOS:
                curTime = time.time()
                if beginTime < 0:
                    beginTime = curTime
                pos = self.ur5.get_pos()
                print("pos.x: %f, pos.y: %f, pos.z: %f, time: %d\n" % (pos.x, pos.y, pos.z, curTime-beginTime)) 
                robotTraj = np.append(robotTraj, np.array([pos.x, pos.y, pos.z, curTime-beginTime]).reshape([1, 4]), 0)
            elif self.robotState == self.STOREPOS:
                np.savetxt(path, robotTraj)
                print("robot trajectory saved at: %s \n" % path)
                break
            elif self.robotState == self.QUIT:
                print("QUIT\n")
                break
        listener.join()

    def demoRobotPoseTraj_6Dof(self, path="robotTrajectory.txt"):
        # 监听按键 'b' 开始记录， 's' 保存，'q' 放弃
        print("demostrate robot trajectory waiting for command")
        listener = keyboard.Listener(on_press=self.robotDemonOnPress)
        listener.start()
        beginTime = -1.0
        robotTraj = np.zeros([0, 8])
        while True:
            if self.robotState == self.CACHEPOS:
                curTime = time.time()
                if beginTime < 0:
                    beginTime = curTime
                pose = self.ur5.get_pose()
                print("pos.x: %f, pos.y: %f, pos.z: %f, time: %f\n" % (pose.pos.x, pose.pos.y, pose. pos.z, curTime-beginTime)) 
                quat = pose.orient.quaternion.get_list()
                print("orient.w: %f, orient.x: %f, orient.y: %f, orient.z: %f\n" % (quat[0], quat[1], quat[2], quat[3]))
                robotTraj = np.append(robotTraj, np.array([pose.pos.x, pose.pos.y, pose.pos.z, quat[0], quat[1], quat[2], quat[3], curTime-beginTime]).reshape([1, 8]), 0)
            elif self.robotState == self.STOREPOS:
                np.savetxt(path, robotTraj)
                print("robot trajectory saved at: %s \n" % path)
                break
            elif self.robotState == self.QUIT:
                print("QUIT\n")
                break
        listener.join()
    
    # 移动机械臂至目标位置
    def moveToPos(self, pos):
        self.ur5.set_pos(pos, self.acc, self.vel, wait=False) # set tool to given pos, keeping constant orientation 只变化位置
    def moveToPose(self,pose):
        self.ur5.set_pose(pose, self.acc, self.vel, wait=False)
    def moveTraj(self, traj):
        self.ur5.movels(traj, acc=self.acc, vel=self.vel, wait=False) # 包含姿态信息
    def moveToHome(self):
        self.ur5.set_pose(self.homePose, self.acc, self.vel, wait=False) #move tcp to point and orientation defined by a transformation
    def moveToHome_pugai(self): # 铺盖任务home
        vector = m3d.Vector( 0.209025, 0.347205, 0.805106)
        orientation = m3d.Orientation(m3d.Versor(0.009121, 0.674213, 0.738367, -0.012966))
        home_pugai = m3d.Transform(orientation, vector)
        self.moveToPose(home_pugai)
    def moveToBegin_pugai(self): # 铺盖任务home
        vector = m3d.Vector(0.322717, 0.305850, 0.728400) # 0.73760 上方1cm 0.72760 最低
        orientation = m3d.Orientation(m3d.Versor(0.004789, -0.034733, 0.999354, -0.007836))
        home_pugai = m3d.Transform(orientation, vector)
        self.moveToPose(home_pugai)

    def stayStill(self):
        pos = self.ur5.get_pos() # get tool tip pos(x, y, z) in base coordinate system
        self.moveToPos(pos)
    def waitForInPosition(self, pos, thresh=0.03):
        while True:
            curPos = self.ur5.get_pos()
            curPosVec = np.array([curPos.x, curPos.y, curPos.z])
            posVec = np.array([pos[0], pos[1], pos[2]])
            dis = np.linalg.norm(posVec - curPosVec) #求二范数
            if dis <= thresh:
                print("robot is in position \n")
                break
    
if __name__ == '__main__':

    ur5 = URRobot()
    # ur5.openGripper()
    vector = m3d.Vector( 0.261187, 0.427371, 0.828347)
    orientation = m3d.Orientation(m3d.Versor(0.013902, 0.749052, -0.662069, 0.019809))
    
    home_pugai = m3d.Transform(orientation, vector)
    ur5.moveToPose(home_pugai)
    # ur5.closeGripperAndHold()
    # ur5.findAndStoreHandTraj(path="/home/q407/HRCollab_ws/src/data_preprocess/datasets/V2/raw_data/hand/hand-6.txt")
    # ur5.demoRobotPoseTraj(path="/home/q407/HRCollab_ws/src/data_preprocess/datasets/V5/robot-15.txt")
    # ur5.demoCollabTraj(handPath='/home/q407/HRCollab_ws/src/data_preprocess/datasets/V7/raw_data/hand/hand-12.txt',
    #                     robotPath='/home/q407/HRCollab_ws/src/data_preprocess/datasets/V7/raw_data/robot/robot-12.txt')
    
    # pos = ur5.ur5.get_pos()
    # ur5.moveGripperTgt(500)
    # ur5.moveToHome_handover()
    # time.sleep(4)
    # ur5.moveToGraspPoint_temp()
    # time.sleep(4)
    # ur5.moveToGraspPoint()
    # ur5.moveToHome_pugai()
    # ur5.moveToBegin_banyun()-
    # ur5.moveToHome_banyun()
    # ur5.moveToHome_handover()

    # vector = m3d.Vector(0.744, -0.213, 0.9541) # [0,0,0] [0,1,0]
    # ur5.moveToPos(vector)
    # while(1):
    #     curPose = ur5.ur5.get_pose() 
    #     print("x",curPose.pos.x,"y",curPose.pos.y,"z",curPose.pos.z )
    #     quat = curPose.orient.quaternion.get_list()
    #     print("orient.w: %f, orient.x: %f, orient.y: %f, orient.z: %f\n" % (quat[0], quat[1], quat[2], quat[3]))
    """
    oriTraj = np.loadtxt("/home/q407/HRCollab_ws/src/data_preprocess/datasets/V6/processed_data_6d/robot-1.txt")
    traj = np.zeros([oriTraj.shape[0], 6])
    for idx in range(oriTraj.shape[0]):
        traj[idx, 0:3] = oriTraj[idx, 0:3]
        r = R.from_quat(np.array([oriTraj[idx, 4], oriTraj[idx, 5], oriTraj[idx, 6], oriTraj[idx, 3]]))
        vec = r.as_rotvec()
        traj[idx, 3:6] = vec
    
    ur5.moveTraj(traj)
    """

    # vector = m3d.Vector(0.681867, 0.427371, 0.728347) # 0.73760 上方1cm 0.72760 最低
    # orientation = m3d.Orientation(m3d.Versor(0.000374, 0.050861, -0.998672, -0.008238))
    # Pose = m3d.Transform(orientation, vector)
    # ur5.moveToPose(Pose)
    # while True:
    #     joints=ur5.ur5.getj()
    #     print("%f,%f,%f,%f,%f,%f\n" % (joints[0], joints[1], joints[2], joints[3],joints[4],joints[5]))
    # 3.131853,-0.986128,0.467987,5.103150,-0.751604,1.690937
    # joints=[3.131853,-0.986128,0.467987,5.103150,-0.751604,1.690937]
    # joints_now = ur5.ur5.movej(joints)

    # joints=[3.123257,-1.051154,0.555166,5.089524,-0.796284,1.626101]
    # joints_now = ur5.ur5.movej(joints,vel=0.10)

    # joints=[3.093622,-1.111022,0.659165,5.129060,-0.829567,1.583161]
    # joints_now = ur5.ur5.movej(joints,vel=0.10)

    # #pose = ur5.ur5.get_pose()
    # #poseVec = ur5.ur5.getl()
    # ur5.moveToHome()   # home <Vector: (0.45787, 0.02572, 1.10201)>
    # ur5.bsdemoRobotPoseTraj(bspath="/home/q40bs7/HRCollab_ws/src/data_preprocess/datasets/V5/robot-tesbst.txt") # 轨迹存储
    # oriTraj = np.loadtxt("/home/q407/HRCollab_ws/src/data_preprocess/datasets/V5/robot-test.txt")
    # oriTraj = oriTraj[3000::200]
    # traj = np.zeros([oriTraj.shape[0], 6])
    # for idx in range(oriTraj.shape[0]):
    #     traj[idx, 0:3] = oriTraj[idx, 0:3]
    #     r = R.from_quat(np.array([oriTraj[idx, 4], oriTraj[idx, 5], oriTraj[idx, 6], oriTraj[idx, 3]]))
    #     vec = r.as_rotvec()
    #     traj[idx, 3:6] = vec
    # ur5.moveTraj(traj)
    # pass
    # while True:
    #     curPose = ur5.ur5.get_pose() 
    #     print("x",curPose.pos.x,"y",curPose.pos.y,"z",curPose.pos.z )
    # y轴沿左右，远端为负 
    # x轴沿前后，近段为小
    # z轴沿竖直，上为大

    # curPose = ur5.ur5.get_pose() 

    # # 位置
    # position = np.zeros([3])
    # position[0] = curPose.pos.x
    # position[1] = curPose.pos.y + 0.1
    # position[2] = curPose.pos.z

    # # 姿态
    # quat = curPose.orient.quaternion.get_list()
    # orientation = np.zeros([4])
    # orientation[0] = quat[0]
    # orientation[1] = quat[1]
    # orientation[2] = quat[2]
    # orientation[3] = quat[3]
    # ur5.moveToPos(position)
    # ur5.moveToHome() # (0.45787, 0.02572, 1.10201)

    # ur5.moveToPos()
    # ur5.moveTraj()
    # ur5.showHand()