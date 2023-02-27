# -----------------------------------------------------------------
# Author:          Hyuga Suzuki
# Created:         2022/9/23
# Summary:         ratioの値を取得（Bluetooth経由シリアル通信）
# -----------------------------------------------------------------

import serial

class SliderManager:
    def __init__(self, port, operatorNum):
        try:
            self.slider_serial = serial.Serial(port, 9600, timeout=0.1)
            self.operatorNum = operatorNum

        except:
            print('Failed to connect to the slider')
            pass

    def receive(self):
        while True:
            try:
                line = self.slider_serial.readline()
                data = float(line.decode('utf-8').rstrip('\n'))

                if self.operatorNum == 1:
                    self.slider_xratio = [data,data]    #[RigidBody1-to-xArmPos, RigidBody1-to-xArmRot]
                    self.slider_mikataratio = [1-data]  #[RigidBody1-to-mikataArmPos]

                if self.operatorNum == 2:
                    self.slider_xratio = [data,data,1-data,1-data]  #[RigidBody1-to-xArmPos, RigidBody1-to-xArmRot, RigidBody2-to-xArmPos, RigidBody2-to-xArmRot]
                    self.slider_mikataratio = [1-data,data]         #[RigidBody1-to-mikataArmPos,RigidBody2-to-mikataArm]

            except:
                print('Failed to read slider value')
                self.slider_xratio = [0.75,0.75,0.25,0.25]
                self.slider_mikataratio = [0.25,0.75]