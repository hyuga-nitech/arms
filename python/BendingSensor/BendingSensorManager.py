import serial
import logging

class BendingSensorManager:
    def __init__(self, port, baudrate) -> None:
        self.bending_value = 0
        self.serial_object = serial.Serial(port,baudrate)
        logging.info("Start serial: %c ", port)
    
    def start_receiving(self):
        """
        Receiving data from bending sensor and update self.bendingValue
        """

        try:
            while True:
                data = self.serial_object.readline()
                self.bending_value = float(data.strip().decode('utf-8'))

        except KeyboardInterrupt:
            self.serial_object.close()
            logging.info('KeyboardInterrupt >> Stop: BendingSensorManager.py')