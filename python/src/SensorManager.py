import time
import serial

class GripperSensorManager:
    def __init__(self, ComPort, BandRate) -> None:
        self.sensorValue   = 1
        self.ComPort = ComPort
        print(self.ComPort)

        if ComPort != 'None':
            self.serial_object = serial.Serial(ComPort, BandRate)

        else:
            pass

    def start_receiving(self):
        try:
            while True:
                if self.ComPort != 'None':
                    data = self.serial_object.readline()
                    self.sensorValue = float(data.strip().decode('utf-8'))

                else:
                    self.sensorValue = 1

                time.sleep(0.005)

        except KeyboardInterrupt:
            print('KeyboardInterrupt >> Stop: BendingSensorManager.py')

class FootSwitchManager:
    def __init__(self) -> None:
        self.flag = False

    def detect_sensor(self):
        time.sleep(3)

        try:
            while True:
                key = input('press to start predicition')
                if key == 'f':
                    self.flag = True
                    print('----- foot switch pressed -----')

                time.sleep(0.005)

        except:
            print('error occured')