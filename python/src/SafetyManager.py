from math import pi

import numpy as np

import lib.self.CustomFunction as cf


class SafetyManager:
    def __init__(self, xArmConfigs):
        self.Init_X, self.Init_Y, self.Init_Z, self.Init_Roll, self.Init_Pitch, self.Init_Yow = self.set_initial(xArmConfigs['InitPos'], xArmConfigs['InitRot'])
        self.Max_X, self.Max_Y, self.Max_Z, self.Max_Roll, self.Max_Pitch, self.Max_Yow = self.set_maximum(xArmConfigs['MaxPos'], xArmConfigs['MaxRot'])
        self.Min_X, self.Min_Y, self.Min_Z, self.Min_Roll, self.Min_Pitch, self.Min_Yow = self.set_minimum(xArmConfigs['MinPos'], xArmConfigs['MinRot'])
        self.initRot = cf.euler_to_quaternion(xArmConfigs['InitRot'])

    def set_initial(self, InitPos, InitRot):
        return InitPos[0], InitPos[1], InitPos[2], InitRot[0], InitRot[1], InitRot[2]

    def set_maximum(self, MaxPos, MaxRot):
        return MaxPos[0], MaxPos[1], MaxPos[2], MaxRot[0], MaxRot[1], MaxRot[2]

    def set_minimum(self, MinPos, MinRot):
        return MinPos[0], MinPos[1], MinPos[2], MinRot[0], MinRot[1], MinRot[2]

    def check_limit(self, position, rotation):
        rotation = cf.quaternion_to_euler(np.dot(cf.convert_to_matrix(rotation), self.initRot))

        x, y, z = position[0] + self.Init_X, position[1] + self.Init_Y, position[2] + self.Init_Z
        roll, pitch, yow = rotation[0], rotation[1], rotation[2]

        # pos X
        if(x > self.Max_X):
            x = self.Max_X
        elif(x < self.Min_X):
            x = self.Min_X

        # pos y
        if(y> self.Max_Y):
            y = self.Max_Y
        elif(y < self.Min_Y):
            y = self.Min_Y

        # pos z
        if(z > self.Max_Z):
            z = self.Max_Z
        elif(z < self.Min_Z):
            z = self.Min_Z

        # # roll
        # if(0 < rotation[0] < self.MaxRot[0]):
        #     rotation[0] = self.MaxRot[0]
        # elif(self.MinRot[0] < rotation[0] < 0):
        #     rotation[0] = self.MinRot[0]

        # # pitch
        # if(rotation[1] > self.MaxRot[1]):
        #     rotation[1] = self.MaxRot[1]
        # elif(rotation[1] < self.MinRot[1]):
        #     rotation[1] = self.MinRot[1]

        # # yaw
        # if(rotation[2] > self.MaxRot[2]):
        #     rotation[2] = self.MaxRot[2]
        # elif(rotation[2] < self.MinRot[2]):
        #     rotation[2] = self.MinRot[2]

        return [x, y, z, roll, pitch, yow]

