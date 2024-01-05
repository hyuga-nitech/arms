import numpy as np
from src.SafetyManager import SafetyManager
from lib.xarm.wrapper import XArmAPI
import lib.self.CustomFunction as cf


class RobotControlManager:
    def __init__(self, xArmConfigs) -> None:
        self.xArmManagers = {}
        for xArm in xArmConfigs:
            self.xArmManagers[xArmConfigs[xArm]['Mount']] = xArmManager(xArmConfigs[xArm])

    def SendDataToRobot(self, sharedMotions):
        for mount in self.xArmManagers.keys():
            self.xArmManagers[mount].send_data_to_robot(sharedMotions[mount])

    def DisConnect(self):
        for mount in self.xArmManagers.keys():
            self.xArmManagers[mount].disconnect()

class xArmManager:
    def __init__(self, xArmConfig: dict) -> None:
        self.xArmConfig = xArmConfig
        self.arm = XArmAPI(xArmConfig['IP'])
        self.safetyManager = SafetyManager(xArmConfig)
        self.initialize_all(xArmConfig['InitPos'], xArmConfig['InitRot'])
        self.initPosition, self.initQuaternion = xArmConfig['InitPos'], cf.euler_to_quaternion(xArmConfig['InitRot'])

    def disconnect(self):
        self.arm.disconnect()
        print('Disconnect >> xArm: {}', format(self.xArmConfig['IP']))

    def check_error(self):
        if self.arm.has_err_warn:
            print('[ERROR] >> xArm: {} Error has occured.', format(self.xArmConfig['IP']))

    def send_data_to_robot(self, transform):
        self.arm.set_servo_cartesian(self.safetyManager.check_limit(transform['position'], transform['rotation']))
        self.arm.getset_tgpio_modbus_data(self.convert_to_modbus_data(transform['gripper']))

    def initialize_all(self, InitPos, InitRot):
        self.arm.connect()
        if self.arm.warn_code != 0:
            self.arm.clean_warn()
        if self.arm.error_code != 0:
            self.arm.clean_error()
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)            
        self.arm.set_state(state=0)    

        self.arm.set_position(x = InitPos[0], y = InitPos[1], z = InitPos[2], roll = InitRot[0], pitch = InitRot[1], yaw = InitRot[2], wait=True)
        print('Initialized >> xArm: {}'.format(self.xArmConfig['IP']))

        self.arm.set_tgpio_modbus_baudrate(2000000)
        self.arm.set_gripper_mode(0)
        self.arm.set_gripper_enable(True)
        self.arm.set_gripper_position(850, speed=5000)
        self.arm.getset_tgpio_modbus_data(self.convert_to_modbus_data(850))
        print('Initialized >> xArm gripper: {}'.format(self.xArmConfig['IP']))

        self.arm.set_mode(1)
        self.arm.set_state(state=0)

    def convert_to_modbus_data(self, value: int):
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

    def increment_init_value(self, position, rotation):
        pos = np.array(position) + self.initPosition
        rot = cf.quaternion_to_euler(np.dot(cf.convert_to_matrix(rotation), self.initQuaternion))

        return pos[0], pos[1], pos[2], rot[0], rot[1], rot[2]