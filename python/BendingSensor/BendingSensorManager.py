# -----------------------------------------------------------------
# Author:          Hyuga Suzuki
# Original Author: Takumi Katagiri (Nagoya Institute of Technology), Takayoshi Hagiwara (KMD)
# Created:         2022/5/20
# Summary:         BendingSensorの値を取得（シリアル通信）
# -----------------------------------------------------------------

import serial

class BendingSensorManager:

    bendingValue = 0
    
    def __init__(self, port, baudrate) -> None:
        
        self.serialObject = serial.Serial(port,baudrate)
    
    def StartReceiving(self):
        """
        Receiving data from bending sensor and update self.bendingValue
        """
        
        try:
            while True:
                data = self.serialObject.readline()
                self.bendingValue = float(data.strip().decode('utf-8'))

        except KeyboardInterrupt:
            print('KeyboardInterrupt >> Stop: BendingSensorManager.py')