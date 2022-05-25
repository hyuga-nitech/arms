# -----------------------------------------------------------------
# Author:          Hyuga Suzuki
# Original Author: Takayoshi Hagiwara (KMD)
# Created:         2022/5/23
# Summary:         mikataArmの初期値、限界値の設定、指令値の変換
# -----------------------------------------------------------------

import math as math
import numpy as np

class mikataTransform:
    """
    mikataArmの座標と回転を保持するクラス
    Dynamixelの角度指令値への変換も行う

    肩に取り付けた剛体との差を使用する予定のため、必ずリミッターを用いる
    """

    #入力位置変数
    x,y,z = 0

    #極座標変数
    r     = 0
    phi   = 0
    theta = 0

    #モータ角度変数
    d1,d2,d3,d4 = 0,0,0,0

    #モータ電流変数
    c1,c2,c3,c4 = 0,0,0,0

    # ----- Initial limitation ----- #
    __initd1, __initd2, __initd3, __initd4, __initd5 = 270, 260, 100, 180, 240

    # ----- Minimum limitation ----- #
    __minR, __minPhi, __minTheta     = 60, -150, -30

    # ----- Maximum limitation ----- #
    __maxR, __maxPhi, __maxTheta     = 300, 150, 90

    def __init__(self):
        pass
    
    def Transform(self):
        self.TransformArray = mikataTransform.Cartesian2Polar(self.x,self.y,self.z)

        r     = math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
        phi   = math.degrees(math.atan2(self.y,self.x))
        theta = math.degrees(math.atan2(math.sqrt(self.x * self.x + self.y * self.y),-1*(self.z)))

        # R limit
        if(r > self.__maxR):
            r = self.__maxR
        elif(r < self.__minR):
            r = self.__minR

        # Phi limit
        if(phi > self.__maxPhi):
            phi = self.__maxPhi
        elif(phi < self.__minPhi):
            phi = self.__minPhi

        # Theta limit
        if(theta > self.__maxTheta):
            theta = self.__maxTheta
        elif(theta < self.__minTheta):
            theta = self.__minTheta

        # Motor's Kinematics
        d1 = 270 + phi
        d2 = 260 - theta - math.degrees(math.acos(r/2/150))
        d3 = 100 - (180 - 2 * math.degrees(math.asin(r/2/150)))
        d4 = 180 + theta + math.degrees(math.acos(r/2/150)) - (180 - 2 * math.degrees(math.asin(r/2/150)))

        c1 = mikataTransform.Degree2Current(d1)
        c2 = mikataTransform.Degree2Current(d2)
        c3 = mikataTransform.Degree2Current(d3)
        c4 = mikataTransform.Degree2Current(d4)

        return c1,c2,c3,c4

    def Degree2Current(self,degree):
        self.c = round(degree / 360 * 4095)
        if(c > 4095):
            c = 4095
        elif(c < 0):
            c = 0
        
        return c

    def GetmikataInitialTransform(self):
        __initc1 = mikataTransform.Degree2Current(self.__initd1)
        __initc2 = mikataTransform.Degree2Current(self.__initd2)
        __initc3 = mikataTransform.Degree2Current(self.__initd3)
        __initc4 = mikataTransform.Degree2Current(self.__initd4)
        __initc5 = mikataTransform.Degree2Current(self.__initd5)
        return [__initc1, __initc2, __initc3, __initc4, __initc5]