import numpy as np
import json as js
import logging

class xArmTransform:
    def __init__(self,arm_key):
        xArm_setting_f = open("xArm_setting.json","r")
        self.xArm_js = js.load(xArm_setting_f)

        self.mount = self.xArm_js[arm_key]["Mount"]

        self.__initX, self.__initY, self.__initZ = self.xArm_js[arm_key]["InitialPos"]
        self.__initRoll, self.__initPitch, self.__initYaw = self.xArm_js[arm_key]["InitialRot"]
        self.__maxX, self.__maxY, self.__maxZ = self.xArm_js[arm_key]["MaxPos"]
        self.__maxRoll, self.__maxPitch, self.__maxYaw = self.xArm_js[arm_key]["MaxRot"]
        self.__minX, self.__minY, self.__minZ = self.xArm_js[arm_key]["MinPos"]
        self.__minRoll, self.__minPitch, self.__minYaw = self.xArm_js[arm_key]["MinRot"]

        logging.info("Initial setting : %c", arm_key)

    def get_initial_transform(self):
        """
        Get the initial position and rotation.
        """

        return self.__initX, self.__initY, self.__initZ, self.__initRoll, self.__initPitch, self.__initYaw
    
    def transform(self, pos_list, rot_list, isLimit = True, isOnlyPosition = False):
        
        pos_list = pos_list * 1000

        if  self.mount == "flat":
            x, y, z = pos_list[2] + self.__initX, pos_list[0] + self.__initY, pos_list[1] + self.__initZ
            roll, pitch, yaw = rot_list[2] + self.__initRoll, rot_list[0] + self.__initPitch, rot_list[1] + self.__initYaw

        elif self.mount == "right":
            x, y, z = pos_list[2] + self.__initX, pos_list[0] + self.__initY, pos_list[1] + self.__initZ
            roll, pitch, yaw = rot_list[2] + self.__initRoll, rot_list[0] + self.__initPitch, rot_list[1] + self.__initYaw

        else:
            print("Hey!!! You've not program this.")
    
        if isOnlyPosition:
            roll, pitch, yaw = self.__initRoll, self.__initPitch, self.__initYaw

        if isLimit:
            # pos X
            if(x > self.__maxX):
                x = self.__maxX
            elif(x < self.__minX):
                x = self.__minX
            
            # pos Y
            if(y> self.__maxY):
                y = self.__maxY
            elif(y < self.__minY):
                y = self.__minY
            
            # pos Z
            if(z > self.__maxZ):
                z = self.__maxZ
            elif(z < self.__minZ):
                z = self.__minZ

            # Roll
            if(0 < roll < self.__maxRoll):
                roll = self.__maxRoll
            elif(self.__minRoll < roll < 0):
                roll = self.__minRoll
            
            # Pitch
            if(pitch > self.__maxPitch):
                pitch = self.__maxPitch
            elif(pitch < self.__minPitch):
                pitch = self.__minPitch
            
            # Yaw
            if(yaw > self.__maxYaw):
                yaw = self.__maxYaw
            elif(yaw < self.__minYaw):
                yaw = self.__minYaw

        return np.array([x, y, z, roll, pitch, yaw])
    
    def ConvertToModbusData(self, value: int):
        """
        Converts the data to modbus type.

        Parameters
        ----------
        value: int
            The data to be converted.
            Range: 0 ~ 800
        """

        if int(value) <= 255 and int(value) >= 0:
            dataHexThirdOrder = 0x00
            dataHexAdjustedValue = int(value)

        elif int(value) > 255 and int(value) <= 511:
            dataHexThirdOrder = 0x01
            dataHexAdjustedValue = int(value)-256

        elif int(value) > 511 and int(value) <= 767:
            dataHexThirdOrder = 0x02
            dataHexAdjustedValue = int(value)-512

        elif int(value) > 767 and int(value) <= 1123:
            dataHexThirdOrder = 0x03
            dataHexAdjustedValue = int(value)-768

        modbus_data = [0x08, 0x10, 0x07, 0x00, 0x00, 0x02, 0x04, 0x00, 0x00]
        modbus_data.append(dataHexThirdOrder)
        modbus_data.append(dataHexAdjustedValue)

        return modbus_data