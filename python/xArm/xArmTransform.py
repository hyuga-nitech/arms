# -----------------------------------------------------------------
# Author:   Takayoshi Hagiwara (KMD)
# Created:  2021/8/12
# Summary:  xArmの初期値、限界値の設定
# -----------------------------------------------------------------

from math import pi
import numpy as np

class xArmTransform:
    """
    xArmの座標と回転を保持するクラス
    """

    x, y, z             = 0, 0, 0
    roll, pitch, yaw    = 0, 0, 0

    # ----- Minimum limitation ----- #
    __initX, __initY, __initZ           = 310, 0, 600
    __initRoll, __initPitch, __initYaw  = 179.9, 1.6, 0.3

    # ----- Minimum limitation ----- #
    __minX, __minY, __minZ          = 310, -300, 225
    __minRoll, __minPitch, __minYaw = -90, -65, -90

    # ----- Maximum limitation ----- #
    __maxX, __maxY, __maxZ          = 650, 300, 650
    __maxRoll, __maxPitch, __maxYaw = 90, 70, 90

    def __init__(self):
        pass
    
    def SetInitialTransform(self, initX, initY, initZ, initRoll, initPitch, initYaw):
        """
        Set the initial position and rotation.
        If this function is not called after the class is instantiated, the initial values of the member variables of this class will be used.

        Parameters
        ----------
        initX, initY, initZ: float
            Initial position.
        initRoll, initPitch, initYaw: float
            Initial rotation.
        """

        self.__initX = initX
        self.__initY = initY
        self.__initZ = initZ

        self.__initRoll     = initRoll
        self.__initPitch    = initPitch
        self.__initYaw      = initYaw

    def GetInitialTransform(self):
        """
        Get the initial position and rotation.
        """

        return self.__initX, self.__initY, self.__initZ, self.__initRoll, self.__initPitch, self.__initYaw
    
    def SetMinimumLimitation(self, minX, minY, minZ, minRoll, minPitch, minYaw):
        """
        Set the lower limit of the position and rotation.
        If this function is not called after the class is instantiated, the initial values of the member variables of this class will be used.

        Parameters
        ----------
        minX, minY, minZ: float
            Lower limit of the position.
        minRoll, minPitch, minYaw: float
            Lower limit of the rotation.
        """

        self.__minX = minX
        self.__minY = minY
        self.__minZ = minZ

        self.__minRoll  = minRoll
        self.__minPitch = minPitch
        self.__minYaw   = minYaw
    
    def SetMaximumLimitation(self, maxX, maxY, maxZ, maxRoll, maxPitch, maxYaw):
        """
        Set the upper limit of the position and rotation.
        If this function is not called after the class is instantiated, the initial values of the member variables of this class will be used.

        Parameters
        ----------
        maxX, maxY, maxZ: float
            Upper limit of the position.
        maxRoll, maxPitch, maxYaw: float
            Upper limit of the rotation.
        """

        self.__maxX = maxX
        self.__maxY = maxY
        self.__maxZ = maxZ

        self.__maxRoll  = maxRoll
        self.__maxPitch = maxPitch
        self.__maxYaw   = maxYaw
    

    def Transform(self, posMagnification = 1, rotMagnification = 1, isLimit = True, isOnlyPosition = True):
        """
        Calculate the position and rotation to be sent to xArm.
        
        Parameters
        ----------
        posMagnification: int (Default = 1)
            Magnification of the position. Used when you want to move the position less or more.
        rotMagnification: int (Default = 1)
            Magnification of the rotation. Used when you want to move the position less or more.
        isLimit: bool (Default = True)
            Limit the position and rotation.
            Note that if it is False, it may result in dangerous behavior.
        isOnlyPosition: bool (Default = True)
            Reflect only the position.
            If True, the rotations are __initRoll, __initPitch, and __initYaw.
            If False, the rotation is also reflected.
        """
        
        x, y, z             = self.x * posMagnification + self.__initX, self.y * posMagnification + self.__initY, self.z * posMagnification + self.__initZ
        roll, pitch, yaw    = self.roll * rotMagnification + self.__initRoll, self.pitch * rotMagnification + self.__initPitch, self.yaw * rotMagnification + self.__initYaw

        if isOnlyPosition:
            roll, pitch, yaw    = self.__initRoll, self.__initPitch, self.__initYaw

        if isLimit:
            # pos X
            if(x > self.__maxX):
                x = self.__maxX
            elif(x < self.__minX):
                x = self.__minX
            
            # pos Y
            if(y> self.__maxY):
                y = self.__maxY
            elif(y < self.__minY):
                y = self.__minY
            
            # pos Z
            if(z > self.__maxZ):
                z = self.__maxZ
            elif(z < self.__minZ):
                z = self.__minZ

            # Roll
            if(0 < roll < self.__maxRoll):
                roll = self.__maxRoll
            elif(self.__minRoll < roll < 0):
                roll = self.__minRoll
            
            # Pitch
            if(pitch > self.__maxPitch):
                pitch = self.__maxPitch
            elif(pitch < self.__minPitch):
                pitch = self.__minPitch
            
            # Yaw
            if(yaw > self.__maxYaw):
                yaw = self.__maxYaw
            elif(yaw < self.__minYaw):
                yaw = self.__minYaw

        return np.array([x, y, z, roll, pitch, yaw])