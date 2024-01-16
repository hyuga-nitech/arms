import time
import threading
import logging
import json as js
from xarm.wrapper import XArmAPI

# ----- Custom class ----- #
from xArmTransform.xArmTransform import xArmTransform
from MotionCalculator.MotionCalculator import MotionCalculator
from Recorder.DataRecordManager import DataRecordManager
from MotionManager.MotionManager import MotionManager
from FootSwitchManager.FootSwitchManager import FootSwitchManager
from VibrotactileFeedback.VibrotactileFeedbackManager import VibrotactileFeedbackManager

# ----- Setting ----- #
OperatorNum             = 2
PairID                  = 1
OperatorID              = 1   #Only OperatorNum = 2

xratio = [0.5,0.5,0.5,0.5]

# ----- Core Setting ----- #
bendingSensorNum        = 2
xArmMovingLimit         = 500
executionTime           = 9999

class RobotControlManager:
    def __init__(self) ->None:
        xArm_setting_f = open("xArm_setting.json","r")
        self.xArm_js = js.load(xArm_setting_f)

    def mainloop(self,isExportData: bool = True, isEnableArm: bool = True, isSlider: bool = True):
        # ----- Process info ----- #
        self.loopCount      = 0
        self.taskTime       = []
        self.errorCount     = 0
        taskStartTime       = 0

        # ----- Instantiating custom classes ----- #
        Calculator           = MotionCalculator(OperatorNum)
        motionManager       = MotionManager()
        dataRecordManager   = DataRecordManager()
        # vibrotactileManager = VibrotactileFeedbackManager()

        if isEnableArm:
            self.arm_object_dict = {}
            self.xArm_transform_dict = {}
            for arm in self.xArm_js["xArmConfig"]:
                self.arm_object_dict[arm] = XArmAPI(self.xArm_js["xArmConfig"][arm]["IP"])
                self.xArm_transform_dict[arm] = xArmTransform(arm)
            self.initialize_arms(self.arm_object_dict, self.xArm_transform_dict)

        self.foot_switch_manager = FootSwitchManager()
        streamingThread = threading.Thread(target=self.foot_switch_manager.detect_sensor)
        streamingThread.setDaemon(True)
        streamingThread.start()

        # ----- Control flags ----- #
        isMoving = False

        try:
            while True:
                if isMoving and (time.perf_counter() - taskStartTime > executionTime):
                    logging.info("Task time finished")
                    isMoving = False

                    self.taskTime.append(time.perf_counter() - taskStartTime)
                    self.print_process_info()
                    
                    if isExportData:
                        dataRecordManager.ExportSelf(filename)

                    if isEnableArm:
                        self.disconnect_arms(self.arm_object_dict)

                    print('----- Finish Task -----')
                    break

                if isMoving:
                    # ----- Get transform data ----- #
                    local_position    = motionManager.LocalPosition(loopCount=self.loopCount)
                    local_rotation    = motionManager.LocalRotation(loopCount=self.loopCount)
                    # モーキャプからの生の値を取得する．

                    arm_position = {}

                    for arm in self.xArm_js["xArmConfig"]:
                        error, arm_position[arm] = self.arm_object_dict[arm].get_position()
                        if error != 0:
                            logging.error("Can NOT get position about %s", arm)

                    xArm_pos, xArm_rot, ratio_dict = Calculator.calculate_shared_pos_rot(self.foot_switch_manager.flag, local_position, local_rotation, arm_position)
                    # これはdictで作る．アームごとのdictのイメージ

                    if isEnableArm:
                        self.move_arms(self.arm_object_dict, self.xArm_transform_dict, xArm_pos, xArm_rot)

                    # ----- Bending sensor ----- #
                    dict_bending_value = motionManager.get_gripper_value_dict(loopCount=self.loopCount)

                    gripper_value_dict = Calculator.calculate_shared_grip()

                    if isEnableArm:
                        self.grip_arms(self.arm_object_dict, self.xArm_transform_dict, gripper_value_dict)

                    # ----- Vibrotactile Feedback ----- #
                    # if OperatorNum == 2:
                    #     vibrotactileManager.FBEachOther(localPosition, localRotation, xratio)

                    # ----- Data recording ----- #
                    Time = time.perf_counter()
                    dataRecordManager.Record(Time, local_position, local_rotation, dict_bending_value, ratio_dict)

                    self.error_check(self.arm_object_dict)

                    self.loopCount += 1

                else:
                    filename = "test"
                    print('Cullent OutputFilename = ' + filename)
                    keycode = input('Input > "q": quit, "s": start control, "r": rename Outputfile \n')

                    # ----- Quit program ----- #
                    if keycode == 'q':
                        if isEnableArm:
                            self.disconnect_arms(self.arm_object_dict)

                        self.print_process_info()
                        break

                    # ----- Rename ----- #
                    elif keycode == 'r':
                        filename = input('Filename?:')
                        continue

                    # ----- Start streaming ----- #
                    elif keycode == 's':
                        Calculator.set_origin_position(motionManager.LocalPosition())
                        Calculator.set_inversed_matrix(motionManager.LocalRotation())

                        arm_position = {}

                        for arm in self.xArm_js["xArmConfig"]:
                            error, arm_position[arm] = self.arm_object_dict[arm].get_position()
                            # オイラーで取得する
                            if error != 0:
                                logging.error("Can NOT get position about %s", arm)

                        xArm_pos, xArm_rot = Calculator.calculate_shared_pos_rot(motionManager.LocalPosition(), motionManager.LocalRotation(), arm_position)
                        
                        self.move_arms(self.arm_object_dict, self.xArm_transform_dict, xArm_pos, xArm_rot)

                        # ----- Bending sensor ----- #
                        dict_bending_value = motionManager.get_gripper_value_dict(loopCount=self.loopCount)

                        gripper_value_dict = Calculator.calculate_shared_grip()

                        if isEnableArm:
                            self.grip_arms(self.arm_object_dict, self.xArm_transform_dict, gripper_value_dict)

                        # beforeX , beforeY , beforeZ            = xArmtransform.x, xArmtransform.y, xArmtransform.z

                        isMoving = True
                        taskStartTime = time.perf_counter()

        except KeyboardInterrupt:
            print('\nKeyboardInterrupt >> Stop: RobotControlManager.SendDataToRobot()')

            self.taskTime.append(time.perf_counter() - taskStartTime)
            self.print_process_info()
            
            if isExportData:
                dataRecordManager.ExportSelf(filename)

            if isEnableArm:
                self.disconnect_arms(self.arm_object_dict)

        except:
            print('----- Exception has occurred -----')
            import traceback
            traceback.print_exc()

    def initialize_arms(self, arm_object_dict, xArm_transform_dict, isSetInitPosition = True):
        for arm in arm_object_dict.keys():
            arm_object_dict[arm].connect()
            if arm_object_dict[arm].warn_code != 0:
                arm_object_dict[arm].clean_warn()
            if arm_object_dict[arm].error_code != 0:
                arm_object_dict[arm].clean_error()
            arm_object_dict[arm].motion_enable(enable=True)
            arm_object_dict[arm].set_mode(0)             # set mode: position control mode
            arm_object_dict[arm].set_state(state=0)      # set state: sport state

            if isSetInitPosition:
                initX, initY, initZ, initRoll, initPitch, initYaw = xArm_transform_dict[arm].get_initial_transform()
                arm_object_dict[arm].set_position(x=initX, y=initY, z=initZ, roll=initRoll, pitch=initPitch, yaw=initYaw, wait=True)
            else:
                arm_object_dict[arm].reset(wait=True)
            logging.info('Initialized > %s', arm)

            arm_object_dict[arm].set_mode(1)
            arm_object_dict[arm].set_state(state=0)

    def disconnect_arms(self, arm_object_dict):
        for arm in arm_object_dict.keys():
            arm_object_dict[arm].disconnect()
            logging.info('Disconnect > %s', arm)

    def move_arms(self, arm_object_dict, xArm_transform_dict, pos_dict, rot_dict):
        for arm in arm_object_dict.keys():
            arm_object_dict[arm].set_servo_cartesian(xArm_transform_dict[arm].transform(pos_dict[arm], rot_dict[arm]))

    def grip_arms(self, arm_object_dict, xArm_transform_dict, bending_value_dict):
        for arm in arm_object_dict.keys():
            code_1, ret_1 = arm_object_dict[arm].getset_tgpio_modbus_data(xArm_transform_dict[arm].convert_to_Modbus(bending_value_dict[arm]))

    def error_check(self, arm_object_dict):
        for arm in arm_object_dict.keys():
            if arm_object_dict[arm].has_err_warn:
                isMoving    = False
                self.errorCount += 1
                logging.error("xArm Error has occured")
                print('[ERROR] >> xArm Error has occured. Please enter "q" to quit')

    def print_process_info(self):
        """
        Print process information. 
        """

        print('----- Process info -----')
        print('Total loop count > ', self.loopCount)
        for ttask in self.taskTime:
            print('Task time\t > ', ttask, '[s]')
        print('Error count\t > ', self.errorCount)
        print('------------------------')
        