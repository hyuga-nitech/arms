# -----------------------------------------------------------------
# Author:          Hyuga Suzuki
# Created:         2022/9/23
# Summary:         ratioの値を取得（Bluetooth経由シリアル通信）
# -----------------------------------------------------------------

import serial

class SliderManager:
    def __init__(self, port):
        try:
            self.slider_serial = serial.Serial(port, 115200, timeout=0.1)
            print('Successfully connected to the slider')

        except:
            print('Failed to connect to the slider')
            pass

        self.receive()

    def receive(self):
        global slider_xratio
        global slider_mikataratio

        while True:
            try:
                line = self.slider_serial.readline()
                data = line.decode('utf-8').rstrip('\n')
                data = int(data)
                slider_xratio = [0.5+(data/4095)/2,0.5+(data/4095)/2,0.5-(data/4095)/2,0.5-(data/4095)/2] #[RigidBody1-to-xArmPos, RigidBody1-to-xArmRot, RigidBody2-to-xArmPos, RigidBody2-to-xArmRot]
                slider_mikataratio = [0.5-(data/4095)/2,0.5+(data/4095)/2] #[RigidBody1-to-mikataArmPos,RigidBody2-to-mikataArm]

            except:
                slider_xratio = [0.75,0.75,0.25,0.25]
                slider_mikataratio = [0.25,0.75]