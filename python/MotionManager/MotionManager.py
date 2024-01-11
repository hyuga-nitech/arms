import threading
import numpy as np
import json as js

# ----- Custom class ----- #
from Optitrack.OptiTrackStreamingManager import OptiTrackStreamingManager
from BendingSensor.BendingSensorManager import BendingSensorManager

# ----- Numeric range remapping ----- #
TARGET_MIN = 0
TARGET_MAX = 850
SENSOR_CLOSE = 0
SENSOR_OPEN = 1

class MotionManager:
    def __init__(self) ->None:
        self.optiTrackStreamingManager = OptiTrackStreamingManager()
        streamingThread = threading.Thread(target=self.optiTrackStreamingManager.stream_run)
        streamingThread.setDaemon(True)
        streamingThread.start()

        bending_f = open("bending_sensor_setting.json","r")
        self.bending_sensor_js = js.load(bending_f)

        self.bending_sensor_object_dict = {}

        for sensor in self.bending_sensor_js["BendingSensorConfig"]:
            sensor_setting = self.bending_sensor_js["BendingSensorConfig"][sensor]
            bendingSensorManager = BendingSensorManager(port=sensor_setting["Port"], baudrate=sensor_setting["Baudrate"])
            self.bending_sensor_object_dict[sensor] = bendingSensorManager

            # ----- Start receiving bending sensor value from UDP socket ----- #
            bendingSensorThread = threading.Thread(target=bendingSensorManager.start_receiving)
            bendingSensorThread.setDaemon(True)
            bendingSensorThread.start()

    def get_gripper_value_dict(self,loopCount: int = 0):
        dict_gripper_value = {}
        for sensor in range(self.bending_sensor_js):
            bending_norm = self.bending_sensor_object_dict[sensor].bending_value
            if SENSOR_CLOSE < SENSOR_OPEN:
                bending_norm = (bending_norm - SENSOR_CLOSE) / (SENSOR_OPEN - SENSOR_CLOSE) * (TARGET_MAX - TARGET_MIN) + TARGET_MIN
            elif SENSOR_CLOSE > SENSOR_OPEN:
                bending_norm = (SENSOR_CLOSE - bending_norm) / (SENSOR_CLOSE - SENSOR_OPEN) * (TARGET_MAX - TARGET_MIN) + TARGET_MIN

            if bending_norm > TARGET_MAX:
                bending_norm = TARGET_MAX

            dict_gripper_value[sensor] = bending_norm

        return dict_gripper_value

    def LocalPosition(self, loopCount: int = 0):
        dictPos = {}
        dictPos = self.optiTrackStreamingManager.position
        return dictPos

    def LocalRotation(self, loopCount: int = 0):
        dictRot = {}
        dictRot = self.optiTrackStreamingManager.rotation
        return dictRot
