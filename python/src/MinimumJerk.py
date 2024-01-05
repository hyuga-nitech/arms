import threading
import time
import csv
from itertools import cycle as iter_cycle

import numpy as np
import lib.self.CustomFunction as cf
from src.DataManager import DataPlotManager
from src.SensorManager import FootSwitchManager    

class MinimumJerk:
    def __init__(self, Target: list, xArmConfig: dict, Threshold = 300) -> None:
        self.initPos = xArmConfig['InitRot']
        initRot = cf.Convert2InverseMatrix(cf.euler_to_quaternion(xArmConfig['InitRot']))
        self.predictedPosition = []
        self.predictedRotation = []
        self.predictedGripper = []
        self.Threshold = Threshold
        self.target = Target
        self.q_init = []
        for target in self.target:
            target['position'] -= np.array(self.initPos)
            target['rotation'] = np.dot(cf.convert_to_matrix(cf.euler_to_quaternion(target['rotation'])), np.dot(initRot, [0, 0, 0, 1]))
            if target['rotation'][3] < 0:
                target['rotation'] = -target['rotation']
        self.flag = False
        self.initThreshold = 100
        self.wayPoint = []
        self.freq = 240
        self.acc_flag = True
        self.before_acc = 0
        self.before_vel = 0
        self.time_list = []
        self.t0 = 0
        self.tf = 0
        self.x0 = [0, 0, 0]
        self.target_index = 0
        self.pos_list = []
        self.tn = 0
        self.a = 0
        self.elaspedTime = 0
        self.init_time = time.perf_counter()
        self.init_rot = 0

        self.switchManager = FootSwitchManager()
        switchThread = threading.Thread(target=self.switchManager.detect_sensor)
        switchThread.setDaemon(True)
        switchThread.start()

    def GetPosition(self, elaspedTime):
        self.elaspedTime = time.perf_counter() - self.init_time
        position, isMoving, weight, velocity = self.CaluculateMotion(self.elaspedTime, self.target[self.target_index]['position'])
        self.posRetained = position

        if isMoving == False:
            position, isMoving = self.posRetained, False

        return position, isMoving, weight, velocity

    def GetRotation(self, elaspedTime):
        self.elaspedTime = time.perf_counter() - self.init_time
        rotation, isMoving, weight = self.CaluculateSlerpMotion(self.elaspedTime, self.target[self.target_index]['rotation'])
        rotation = np.dot(cf.convert_to_matrix(self.q_init), rotation)
        self.rotRetained = rotation

        if isMoving == False:
            rotation, isMoving = self.rotRetained, False

        return rotation, isMoving, weight

    def GetGripperValue(self):
        try:
            gripper = self.gripRetained = next(self.predictedGripper)
            isMoving = True
        except StopIteration:
            gripper, isMoving = self.gripRetained, False

        return gripper, isMoving

    def MonitoringMotion(self, position, rotation, gripper, velocity, accelaration):
        isMoving = False

        if self.switchManager.flag == True:
            self.init_time = time.perf_counter()
            self.x0 = position
            self.flag = True
            self.switchManager.flag = False

        if self.flag == True:
            self.elaspedTime = time.perf_counter() - self.init_time
            diff_init = np.linalg.norm(np.array(position) - np.array(self.x0))
            self.time_list.append(self.elaspedTime)
            self.pos_list.append(position)

            if diff_init >= self.initThreshold:
                self.rot_n = rotation[0]
                self.target_index = self.DetermineTarget(self.target, position, self.pos_list[-1]-self.pos_list[-2])
                
                self.tf = self.CalculateReachingTime_minimumjerk(self.time_list[-25], velocity, self.target[self.target_index]['position'])
                
                self.CreateMotionData(rotation, gripper, self.target[self.target_index]['position'], self.target[self.target_index]['rotation'], self.target[self.target_index]['gripper'], self.elaspedTime)
                isMoving = True
                self.flag = False

        return isMoving

    def CreateMotionData(self, rot_n, grip_n, pos_f, rot_f, grip_f, tn):
        self.tn = tn
        DataPlotManager.thres = tn
        frameLength = int((self.tf-(tn - self.t0))* self.freq)
        self.CreateGripMotion(grip_n, grip_f, frameLength, gripFrame = 150)

    def DetermineTarget(self, target_list, position, vector):
        D_list = []
        x = position
        a = vector
        alfa = beta = gamma = 0
        for target in target_list:
            y = target['position']
            alfa = np.dot(a, x-y)
            beta = np.dot(a, a)
            k = -alfa/beta
            z = x + k*a
            d = np.linalg.norm(z - y)
            D_list.append(d)

            print('current: {}, target: {}, distance: {}'.format(x, y, d))
        print(D_list)

        return D_list.index(min(D_list))

    def CalculateReachingTime_minimumjerk(self, t, v, xf):
        ans = 0
        a = self.a =  np.sqrt((xf[0] - self.x0[0])**2 + (xf[1] - self.x0[1])**2 + (xf[2] - self.x0[2])**2)
        c = cf.solve_nploy(np.array([-(30*a*((t - self.t0)**4))/v, (60*a*((t - self.t0)**3))/v, -(30*a*((t - self.t0)**2))/v, 0, 0]))
        for cn in c:
            normalize = (t - self.t0)/cn
            if 0 < normalize and normalize < 0.5:
                ans = cn

        return ans
    
    def CreateGripMotion(self, grip_n, grip_f, frameLength, gripFrame):
        def CreateMotion_Liner(target, data, split):
            motionlist = np.linspace(data, target, split)
            return motionlist

        diffGrip = [850] * frameLength
        diffGrip = np.concatenate([diffGrip, CreateMotion_Liner(grip_f, grip_n, gripFrame)], 0)

        self.predictedGripper = iter(diffGrip)

    def CaluculateSlerpMotion(self, elaspedTime, xf):
        isMoving = True
        t = (self.elaspedTime - self.t0)/self.tf
        if t > 1:
            t = 1
            isMoving = False
        weight = (t - (self.tn - self.t0)/self.tf)/(1-(self.tn - self.t0)/self.tf)
        # weight = fc.liner(weight)

        # print(weight)

        return cf.Slerp_Quaternion(xf, self.rot_n, weight), isMoving, weight

    def func_minimumjerk(self,t):
        return 6* (t** 5)- 15* (t** 4)+ 10* (t** 3)
    
    def CaluculateMotion(self, elaspedTime, xf): # デフォルト
        isMoving = True
        t = (self.elaspedTime - self.t0)/self.tf
        if t > 1:
            t = 1
            isMoving = False
        weight = (t - (self.tn - self.t0)/self.tf)/(1-(self.tn - self.t0)/self.tf)
        # print(weight)

        return self.x0 + (xf- self.x0)* self.func_minimumjerk(t), isMoving, weight, 30 * self.a * (t**4 - 2*(t**3) + t**2)
        
class Fitting:
    def __init__(self, path) -> None:
        data = self.load(path)
        time = data[:, 0]
        time = time - time[0]
        time = time/time[-1]
        pos = data[:, 1:4]
        pos = pos - pos[0]
        norm = np.sqrt(pos[:, 0]**2 + pos[:, 1]**2 + pos[:, 2]**2)
        norm = norm/norm[-1]

        self.coe = self.custom_fit(time, norm)
        print(self.coe)

        # y_fit = self.coe[0] * time ** 5 + self.coe[1] * time ** 4 + self.coe[2] * time ** 3 + self.coe[3] * time ** 2 + (1 - (self.coe[0] + self.coe[1] + self.coe[2] + self.coe[3])) * time
        # plt.plot(time, norm)
        # plt.plot(time, y_fit)
        # plt.show()

    def load(self, path):
        with open(path) as file:
            reader = csv.reader(file)
            data = [row for row in reader][1:]
            data = [[float(v) for v in row] for row in data]
            data = np.array(data)

        return data

    def custom_fit(self, x, y):
        a11 = sum((x**5 - x)**2)
        a12 = sum((x**5 - x)*(x**4 - x))
        a13 = sum((x**5 - x)*(x**3 - x))
        a14 = sum((x**5 - x)*(x**2 - x))
        a21 = sum((x**4 - x)*(x**5 - x))
        a22 = sum((x**4 - x)**2)
        a23 = sum((x**4 - x)*(x**3 - x))
        a24 = sum((x**4 - x)*(x**2 - x))
        a31 = sum((x**3 - x)*(x**5 - x))
        a32 = sum((x**3 - x)*(x**4 - x))
        a33 = sum((x**3 - x)**2)
        a34 = sum((x**3 - x)*(x**2 - x))
        a41 = sum((x**2 - x)*(x**5 - x))
        a42 = sum((x**2 - x)*(x**4 - x))
        a43 = sum((x**2 - x)*(x**3 - x))
        a44 = sum((x**2 - x)**2)

        A = np.array([
            [a11, a12, a13, a14],
            [a21, a22, a23, a24],
            [a31, a32, a33, a34],
            [a41, a42, a43, a44]
        ])
        A_inv = np.linalg.inv(A)

        b1 = sum((y - x)*(x**5 - x))
        b2 = sum((y - x)*(x**4 - x))
        b3 = sum((y - x)*(x**3 - x))
        b4 = sum((y - x)*(x**2 - x))

        B = np.array([
            b1,
            b2,
            b3,
            b4
        ])

        return np.dot(A_inv, B)

if __name__ == '__main__':
    def CalculateReachingTime(t, v, xf, x0):
        a = np.sqrt((xf[0] - x0[0])**2 + (xf[1] - x0[1])**2 + (xf[2] - x0[2])**2)
        b = v/(30*a)

        return 0.5*(1 - np.sqrt(1 - 4*np.sqrt(b)))

    T = CalculateReachingTime(3, 20, [80, 60, 0], [0, 0, 0])
    print(T)
