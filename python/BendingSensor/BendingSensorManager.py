# -----------------------------------------------------------------
# Author:          Suzuki Hyuga
# Original Author: Takayoshi Hagiwara (KMD)
# Created:         2022/5/20
# Summary:         BendingSensorの値を取得（シリアル通信）
# -----------------------------------------------------------------

import numpy as np
import socket
import serial

class BendingSensorManager:

    bendingValue = 0
    
    def __init__(self, ip, port) -> None:
        self.ip             = ip
        self.port           = port
        self.bufsize        = 4096
        self.bendingValue   = 425

        self.serialObject = serial.Serial(ip,port)
        nonUsed = self.serialObject.readline()

        """
        self.ser1 = serial.Serial("COM3",9600)
        self.not_used1 = self.ser1.readline()
        self.ser2 = serial.Serial("COM2",9600)
        self.not_used2 = self.ser2.readline()
        """
    
    def StartReceiving(self, fromUdp: bool = False):
        """
        Receiving data from bending sensor and update self.bendingValue
        """
        
        try:
            while True:
                data = self.serialObject.readline()
                self.bendingValue = float(data.strip().decode('utf-8'))

        except KeyboardInterrupt:
            print('KeyboardInterrupt >> Stop: BendingSensorManager.py')