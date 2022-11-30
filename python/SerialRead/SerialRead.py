import serial

class SerialRead:

    def __init__(self,port,baudrate) -> None:

        try:
            self.SerialObject = serial.Serial(port,baudrate)
        except:
            print('!!!!!Failed to connect %s!!!!!',port)
            pass

    def ReadLine(self):
        while True:
            try:
                self.line = self.SerialObject.readline()
                self.SerialVal = float(self.line.strip().decode('utf-8'))

            except:
                pass