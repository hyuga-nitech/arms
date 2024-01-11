import time
import threading
import logging
import json as js
from xArm.wrapper import XArmAPI

# ----- Custom class ----- #
from xArmTransform.xArmTransform import xArmTransform
from MotionBehaviour.MotionBehaviour import MotionBehaviour
from Recorder.DataRecordManager import DataRecordManager
from MotionManager.MotionManager import MotionManager
from VibrotactileFeedback.VibrotactileFeedbackManager import VibrotactileFeedbackManager
from RobotControlManager.SafetyChecker import SafetyChecker

# ----- Setting ----- #
OperatorNum             = 2
PairID                  = 1
OperatorID              = 1   #Only OperatorNum = 2

xratio = [0.5,0.5,0.5,0.5]

# ----- Core Setting ----- #
RigidBodyNum            = 2
bendingSensorNum        = 2
xArmMovingLimit         = 500
executionTime           = 9999

filename = "test"

class RobotControlManager:
    def __init__(self) ->None:
        parameter_f = open("parameter.json","r")
        self.parameter_js = js.load(parameter_f)
        xArm_setting_f = open("xArm_setting.json","r")
        self.xArm_js = js.load(xArm_setting_f)

    def SendDataToRobot(self,isExportData: bool = True, isEnableArm: bool = True, isSlider: bool = True):
        # ----- Process info ----- #
        self.loopCount      = 0
        self.taskTime       = []
        self.errorCount     = 0
        taskStartTime       = 0

        # ----- Instantiating custom classes ----- #
        Behaviour           = MotionBehaviour(OperatorNum)
        motionManager       = MotionManager(RigidBodyNum,bendingSensorNum,self.parameter_js["BendingSensorPorts"],self.parameter_js["BendingSensorBaudrates"])
        dataRecordManager   = DataRecordManager(rigidBodyNum=RigidBodyNum)
        # vibrotactileManager = VibrotactileFeedbackManager()

        if isEnableArm:
            self.arm_object_dict = {}
            self.xArm_transform_dict = {}
            for arm in self.xArm_js["xArmConfig"]:
                self.arm_object_dict[arm] = XArmAPI(arm["IP"])
                self.xArm_transform_dict[arm] = xArmTransform(arm)
            self.initialize_arms(self.arm_object_dict, self.xArm_transform_dict)

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
                    localPosition    = motionManager.LocalPosition(loopCount=self.loopCount)
                    localRotation    = motionManager.LocalRotation(loopCount=self.loopCount)
                    # モーキャプからの生の値を取得する．

                    xArm_pos,xArm_rot = Behaviour.GetSharedxArmTransform(localPosition,localRotation,xratio)
                    # これはdictで作る．アームごとのdictのイメージ

                    if isEnableArm:
                        self.move_arms(self.arm_object_dict, self.xArm_transform_dict, xArm_pos, xArm_rot)

                    # ----- Bending sensor ----- #
                    dictBendingValue = motionManager.GripperControlValue(loopCount=self.loopCount)
                    gripperValue = 0
                    for i in range(bendingSensorNum):
                        gripperValue += dictBendingValue['gripperValue'+str(i+1)] * self.parameter_js["GripperRatio"][i]
                    # 右手と左手で分けてベンディングセンサーを設計する必要あり
                    
                    gripper_value_dict = {}
                    gripper_value_dict["xArm1"] = gripperValue
                    
                    if isEnableArm:
                        self.grip_arms(self.arm_object_dict, self.xArm_transform_dict, gripper_value_dict)

                    # ----- Vibrotactile Feedback ----- #
                    # if OperatorNum == 2:
                    #     vibrotactileManager.FBEachOther(localPosition, localRotation, xratio)

                    # ----- Data recording ----- #
                    Time = time.perf_counter()
                    dataRecordManager.Record(Time, localPosition, localRotation, dictBendingValue)

                    self.error_check(self.arm_object_dict)

                    self.loopCount += 1

                else:
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
                        Behaviour.SetOriginPosition(motionManager.LocalPosition())
                        Behaviour.SetInversedMatrix(motionManager.LocalRotation())


                        xArmPosition,xArmRotation      = Behaviour.GetSharedxArmTransform(motionManager.LocalPosition(),motionManager.LocalRotation(),xratio)
                        
                        self.move_arms(self.arm_object_dict, self.xArm_transform_dict, xArm_pos, xArm_rot)

                        # ----- Bending sensor ----- #
                        dictBendingValue = motionManager.GripperControlValue(loopCount=self.loopCount)
                        gripperValue = 0
                        for i in range(bendingSensorNum):
                            gripperValue += dictBendingValue['gripperValue'+str(i+1)] * self.parameter_js["GripperRatio"][i]

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

    def initialize_arm(self, arm_object_dict, xArm_transform_dict, isSetInitPosition = True):
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
            logging.info('Initialized > %c', arm)

            arm_object_dict[arm].set_mode(1)
            arm_object_dict[arm].set_state(state=0)

    def disconnect_arms(self, arm_object_dict):
        for arm in arm_object_dict.keys():
            arm_object_dict[arm].disconnect()
            logging.info('Disconnect > %c', arm)

    def move_arms(self, arm_object_dict, xArm_transform_dict, pos_dict, rot_dict):
        for arm in arm_object_dict.keys():
            arm_object_dict[arm].set_servo_cartesian(xArm_transform_dict[arm].transform(pos_dict[arm], rot_dict[arm]))

    def grip_arms(self, arm_object_dict, xArm_transform_dict, bending_value_dict):
        for arm in arm_object_dict.keys():
            code_1, ret_1 = arm_object_dict[arm].getset_tgpio_modbus_data(xArm_transform_dict[arm].ConvertToModbusData(bending_value_dict[arm]))

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
        