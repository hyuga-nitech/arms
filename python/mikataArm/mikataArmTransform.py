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
    追記：初期姿勢と開始時の指令値に差が生じやすいため、リミッターを小さい値にしているとエラーになりやすい
    """

    #入力位置変数
    x,y,z = 0,0,0

    #極座標変数
    r     = 0
    phi   = 0
    theta = 0

    #モータ角度変数
    d1,d2,d3,d4 = 0,0,0,0

    #モータ電流変数
    c1,c2,c3,c4 = 0,0,0,0

    # ----- Initial Position ----- #
    __initX, __initY, __initZ = 300,0,0

    # ----- Initial limitation ----- #
    # __initd1, __initd2, __initd3, __initd4 = 270, 260, 100, 180
    __initd5 = 240

    # ----- Minimum limitation ----- #
    __minR, __minPhi, __minTheta     = 60, -150, -30

    # ----- Maximum limitation ----- #
    __maxR, __maxPhi, __maxTheta     = 300, 150, 90

    def __init__(self):
        
        pass
    
    def Transform(self):
        self.posX = self.x + self.__initX
        self.posY = self.y + self.__initY
        self.posZ = self.z + self.__initZ

        r     = math.sqrt(self.posX * self.posX + self.posY * self.posY + self.posZ * self.posZ )
        phi   = math.degrees(math.atan2(self.posY ,self.posX ))
        theta = -1 * math.degrees(math.atan2(self.posZ , math.sqrt(self.posX * self.posX + self.posY * self.posY )))

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
        d1 = 270 - phi
        d2 = 260 - theta - math.degrees(math.acos(r/2/150))
        d3 = 100 + (180 - 2 * math.degrees(math.asin(r/2/150)))
        d4 = 180 + theta + math.degrees(math.acos(r/2/150)) - (180 - 2 * math.degrees(math.asin(r/2/150)))

        D2C = mikataTransform()

        c1 = D2C.Degree2Current(d1)
        c2 = D2C.Degree2Current(d2)
        c3 = D2C.Degree2Current(d3)
        c4 = D2C.Degree2Current(d4)

        print('r,phi,theta = ',r,' ',phi,' ',theta,' d1,d2,d3,d4 = ',d1,' ',d2,' ',d3,' ',d4)

        return c1,c2,c3,c4

    def Degree2Current(self,degree):
        self.c = round(degree / 360 * 4095)
        if(self.c > 4095):
            self.c = 4095
        elif(self.c < 0):
            self.c = 0
        
        return self.c

    def GetmikataInitialTransform(self):
        __initr     = math.sqrt(self.__initX * self.__initX + self.__initY * self.__initY + self.__initZ * self.__initZ )
        __initphi   = math.degrees(math.atan2(self.__initY ,self.__initX ))
        __inittheta = -1 * math.degrees(math.atan2(self.__initZ , math.sqrt(self.__initX * self.__initX + self.__initY * self.__initY )))

        # R limit
        if(__initr > self.__maxR):
            __initr = self.__maxR
        elif(__initr < self.__minR):
            __initr = self.__minR

        # Phi limit
        if(__initphi > self.__maxPhi):
            __initphi = self.__maxPhi
        elif(__initphi < self.__minPhi):
            __initphi = self.__minPhi

        # Theta limit
        if(__inittheta > self.__maxTheta):
            __inittheta = self.__maxTheta
        elif(__inittheta < self.__minTheta):
            __inittheta = self.__minTheta

        # Motor's Kinematics
        self.__initd1 = 270 - __initphi
        self.__initd2 = 260 - __inittheta - math.degrees(math.acos(__initr/2/150))
        self.__initd3 = 100 + (180 - 2 * math.degrees(math.asin(__initr/2/150)))
        self.__initd4 = 180 + __inittheta + math.degrees(math.acos(__initr/2/150)) - (180 - 2 * math.degrees(math.asin(__initr/2/150)))

        __initc1 = round(self.__initd1 * 11.375)
        __initc2 = round(self.__initd2 * 11.375)
        __initc3 = round(self.__initd3 * 11.375)
        __initc4 = round(self.__initd4 * 11.375)
        __initc5 = round(self.__initd5 * 11.375)
        return [__initc1, __initc2, __initc3, __initc4, __initc5]