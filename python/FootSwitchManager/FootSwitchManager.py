import time
import logging

class FootSwitchManager:
    def __init__(self) -> None:
        self.flag = False

    def detect_sensor(self):
        time.sleep(3)

        try:
            while True:
                key = input('press to change mode')
                if key == 'f':
                    if self.flag == False:
                        self.flag = True
                        logging.info("On")

                    elif self.flag == True:
                        self.flag = False
                        logging.info("Off")
                
                time.sleep(0.1)

        except:
            logging.error('FootSwitch error occured')