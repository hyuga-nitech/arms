# -----------------------------------------------------------------
# Author:          Hyuga Suzuki
# Original Author: Takayoshi Hagiwara (KMD)
# Created:         2022/5/24
# Summary:         xArm,mikataArm,BendingSensorの制御
# -----------------------------------------------------------------

import time
import threading
import numpy as np
from xarm.wrapper import XArmAPI
from ctypes import windll

# ----- Custom class ----- #
from xArm.xArmTransform import xArmTransform
from mikataArm.mikataArmTransform import mikataTransform
from mikataArm.mikataControl import mikataControl
from MotionBehaviour.MotionBehaviour import MotionBehaviour
from Recorder.DataRecordManager import DataRecordManager
from BendingSensor.BendingSensorManager import BendingSensorManager
from MotionManager.MotionManager import MotionManager
from FileIO.FileIO import FileIO

# ----- Setting: Number ----- #
defaultRigidBodyNum     = 2
defaultBendingSensorNum = 2
xArmMovingLimit         = 100
mikataMovingLimit       = 2000
xRatio                  = [0.2,0.2,0.8,0.8]  #[RigidBody1-to-xArmPos, RigidBody1-to-xArmRot, RigidBody2-to-xArmPos, RigidBody2-to-xArmRot]
mikataRatio             = [0.8,0.2]  #[RigidBody1-to-mikataArmPos,RigidBody2-to-mikataArm]
gripperRatio            = [0.5,0.5]  #[BendingSensor1-to-mikataGripper,BendingSensor2-to-mikataGripper]

class RobotControlManager:
    def __init__(self) ->None:
        fileIO = FileIO()

        dat = fileIO.Read('settings.csv',',')
        xArmIP = [addr for addr in dat if 'xArmIP' in addr[0]][0][1]
        self.xArmIpAddress = xArmIP

    def SendDataToRobot(self,executionTime: int = 120, isExportData: bool = True, isEnableArm: bool = True):
        # ----- Process info ----- #
        self.loopCount      = 0
        self.taskTime       = []
        self.errorCount     = 0
        taskStartTime       = 0

        # ----- Instantiating custom classes ----- #
        Behaviour         = MotionBehaviour(defaultRigidBodyNum)
        xArmtransform     = xArmTransform()
        mikatatransform   = mikataTransform()
        motionManager     = MotionManager(defaultRigidBodyNum, defaultBendingSensorNum)
        mikatacontrol     = mikataControl()
        dataRecordManager = DataRecordManager(RigidBodyNum=defaultRigidBodyNum)

        if isEnableArm:
            arm = XArmAPI(self.xArmIpAddress)
            self.InitializeAll(arm, xArmtransform, mikatatransform, mikatacontrol)

        # ----- Control flags ----- #
        isMoving = False
        isRatio  = True

        try:
            while True:
                # if time.perf_counter() - taskStartTime > executionTime:
                #     # ----- Exit processing after task time elapses ----- #
                #     isMoving    = False

                #     self.taskTime.append(time.perf_counter() - taskStartTime)
                #     self.PrintProcessInfo()

                #     # ----- Export recorded data ----- #
                #     if isExportData:
                #         dataRecordManager.ExportSelf()

                #     if isEnableArm:
                #         arm.disconnect()
                #         mikatacontrol.ClosePort()

                #     print('----- Finish task -----')
                #     break

                if isMoving:
                    # ----- Get transform data ----- #
                    # ---------- Start control process timer ---------- #
                    localPosition    = motionManager.LocalPosition(loopCount=self.loopCount)
                    localRotation    = motionManager.LocalRotation(loopCount=self.loopCount)

                    # ----- (for Debug) ----- #
                    # print('localPosition:',localPosition)

                    if isRatio:
                        xArmPosition,xArmRotation       = Behaviour.GetSharedxArmTransform(localPosition,localRotation,xRatio)
                        mikataPosition                  = Behaviour.GetSharedmikataArmTransform(localPosition,localRotation,mikataRatio)
                    else:
                        xArmPosition,xArmRotation       = Behaviour.GetxArmTransform(localPosition,localRotation)
                        mikataPosition                  = Behaviour.GetmikataArmTransform(localPosition,localRotation)


                    xArmPosition   = xArmPosition * 1000
                    mikataPosition = mikataPosition * 1000

                    # ----- Set xArm transform ----- #
                    xArmtransform.x, xArmtransform.y, xArmtransform.z           = xArmPosition[2], xArmPosition[0], xArmPosition[1]
                    xArmtransform.roll, xArmtransform.pitch, xArmtransform.yaw  = xArmRotation[2], xArmRotation[0], xArmRotation[1]

                    # ----- Set mikata transform ----- #
                    mikatatransform.x, mikatatransform.y, mikatatransform.z     = mikataPosition[2], mikataPosition[0], mikataPosition[1]

                    # ----- (for Debug) ----- #
                    # print('xArmTransform:',xArmtransform.x,' ', xArmtransform.y,' ', xArmtransform.z)
                    # print('mikataTransform:',mikatatransform.x,' ', mikatatransform.y,' ', mikatatransform.z)

                    # ----- Bending sensor ----- #
                    dictBendingValue = motionManager.GripperControlValue(loopCount=self.loopCount)
                    #gripperValue = sum(dictBendingValue.values()) / len(dictBendingValue)
                    gripperValue = 0
                    for i in range(defaultBendingSensorNum):
                        gripperValue += dictBendingValue['gripperValue'+str(i+1)] * gripperRatio[i]


                    # ----- Calculate mikata Current ----- #
                    mikataC1, mikataC2, mikataC3, mikataC4 = mikatatransform.Transform()
                    mikataC5                               = mikatatransform.Degree2Current(gripperValue)

                    # ----- Safety Check -----#
                    xArmdiffX = xArmtransform.x - beforeX
                    xArmdiffY = xArmtransform.y - beforeY
                    xArmdiffZ = xArmtransform.z - beforeZ
                    beforeX, beforeY, beforeZ = xArmtransform.x, xArmtransform.y, xArmtransform.z

                    mikatadiffC1 = mikataC1 - beforeC1
                    mikatadiffC2 = mikataC2 - beforeC2
                    mikatadiffC3 = mikataC3 - beforeC3
                    mikatadiffC4 = mikataC4 - beforeC4
                    mikatadiffC5 = mikataC5 - beforeC5
                    beforeC1, beforeC2, beforeC3, beforeC4, beforeC5 = mikataC1, mikataC2, mikataC3, mikataC4, mikataC5

                    if abs(xArmdiffX) > xArmMovingLimit or abs(xArmdiffY) > xArmMovingLimit or abs(xArmdiffZ) > xArmMovingLimit:
                        isMoving = False
                        print('[ERROR] >> A rapid movement has occurred in xArm. Please enter "r" to reset xArm, or "q" to quit')
                    elif abs(mikatadiffC1) > mikataMovingLimit or abs(mikatadiffC2) > mikataMovingLimit or abs(mikatadiffC3) > mikataMovingLimit or abs(mikatadiffC4) > mikataMovingLimit:
                        isMoving = False
                        print('[ERROR] >> A rapid movement has occurred in mikataArm. Please enter "r" to reset xArm, or "q" to quit')
                    elif abs(mikatadiffC5) > mikataMovingLimit:
                        isMoving = False
                        print('[ERROR] >> A rapid movement has occurred in mikataArm Gripper. Please enter "r" to reset xArm, or "q" to quit')
                    else:
                        if isEnableArm:
                            # ----- Send to Arms ----- #
                            arm.set_servo_cartesian(xArmtransform.Transform(isOnlyPosition = False))
                            mikatacontrol.dxl_goal_position = [mikataC1, mikataC2, mikataC3, mikataC4, mikataC5]

                    # ----- Data recording ----- #
                    dataRecordManager.Record(localPosition, localRotation, dictBendingValue)

                    # ----- If xArm error has occured ----- #
                    if isEnableArm and arm.has_err_warn:
                        isMoving    = False
                        self.errorCount += 1
                        self.taskTime.append(time.perf_counter() - taskStartTime)
                        print('[ERROR] >> xArm Error has occured. Please enter "r" to reset xArm, or "q" to quit')

                    self.loopCount += 1

                else:
                    keycode = input('Input > "q": quit, "r": Clean error and init arm, "s": start control \n')
                    # ----- Quit program ----- #
                    if keycode == 'q':
                        if isEnableArm:
                            arm.disconnect()
                            mikatacontrol.mikataLoopAlive = False
                            mikatacontrol.ClosePort()

                        self.PrintProcessInfo()
                        break

                    # ----- Reset xArm and gripper ----- #
                    elif keycode == 'r':
                        if isEnableArm:
                            mikatacontrol.mikataLoopAlive = False
                            mikatacontrol.ClosePort()
                            self.InitializeAll(arm, xArmtransform, mikatatransform, mikatacontrol)

                    # ----- Start streaming ----- #
                    elif keycode == 's':
                        Behaviour.SetOriginPosition(motionManager.LocalPosition())
                        Behaviour.SetInversedMatrix(motionManager.LocalRotation())
                        
                        if isRatio:
                            xArmPosition,xArmRotation       = Behaviour.GetSharedxArmTransform(motionManager.LocalPosition(),motionManager.LocalRotation(),xRatio)
                            mikataPosition                  = Behaviour.GetSharedmikataArmTransform(motionManager.LocalPosition(),motionManager.LocalRotation(),mikataRatio)
                        else:
                            xArmPosition,xArmRotation       = Behaviour.GetxArmTransform(motionManager.LocalPosition(),motionManager.LocalRotation())
                            mikataPosition                  = Behaviour.GetmikataArmTransform(motionManager.LocalPosition(),motionManager.LocalRotation())
                        
                        xArmPosition   = xArmPosition * 1000
                        mikataPosition = mikataPosition * 1000

                        # ----- Set xArm transform ----- #
                        xArmtransform.x, xArmtransform.y, xArmtransform.z           = xArmPosition[2], xArmPosition[0], xArmPosition[1]
                        xArmtransform.roll, xArmtransform.pitch, xArmtransform.yaw  = xArmRotation[2], xArmRotation[0], xArmRotation[1]

                        # ----- Set mikata transform ----- #
                        mikatatransform.x, mikatatransform.y, mikatatransform.z     = mikataPosition[2], mikataPosition[0], mikataPosition[1]

                        # ----- Bending sensor ----- #
                        dictBendingValue = motionManager.GripperControlValue(loopCount=self.loopCount)
                        gripperValue = 0
                        for i in range(defaultBendingSensorNum):
                            gripperValue += dictBendingValue['gripperValue'+str(i+1)] * gripperRatio[i]

                        beforeX, beforeY, beforeZ              = xArmtransform.x, xArmtransform.y, xArmtransform.z
                        beforeC1, beforeC2, beforeC3, beforeC4 = mikatatransform.Transform()
                        beforeC5                               = mikatatransform.Degree2Current(gripperValue)

                        motionManager.SetInitialBendingValue()

                        isMoving    = True
                        taskStartTime = time.perf_counter()

        except KeyboardInterrupt:
            print('\nKeyboardInterrupt >> Stop: RobotControlManager.SendDataToRobot()')

            self.taskTime.append(time.perf_counter() - taskStartTime)
            self.PrintProcessInfo()
            
            if isExportData:
                dataRecordManager.ExportSelf()

            if isEnableArm:
                arm.disconnect()
                mikatacontrol.mikataLoopAlive = False
                mikatacontrol.ClosePort()

        except:
            print('----- Exception has occurred -----')
            import traceback
            traceback.print_exc()

    def InitializeAll(self, robotArm, xArmtransform, mikatatransform, mikatacontrol, isSetInitPosition = True):
        # ----- xArm ----- #
        robotArm.connect()
        if robotArm.warn_code != 0:
            robotArm.clean_warn()
        if robotArm.error_code != 0:
            robotArm.clean_error()
        robotArm.motion_enable(enable=True)
        robotArm.set_mode(0)             # set mode: position control mode
        robotArm.set_state(state=0)      # set state: sport state

        if isSetInitPosition:
            initX, initY, initZ, initRoll, initPitch, initYaw = xArmtransform.GetInitialTransform()
            robotArm.set_position(x=initX, y=initY, z=initZ, roll=initRoll, pitch=initPitch, yaw=initYaw, wait=True)
        else:
            robotArm.reset(wait=True)
        print('Initialized > xArm')

        robotArm.motion_enable(enable=True)
        robotArm.set_mode(1)
        robotArm.set_state(state=0)

        # ----- mikata Arm ----- #
        mikatacontrol.OpenPort()
        if isSetInitPosition:
            mikatacontrol.mikataLoopAlive = True
            mikatacontrol.dxl_goal_position = mikatatransform.GetmikataInitialTransform()
            mikataThread = threading.Thread(target=mikatacontrol.SendtomikataArm)
            mikataThread.setDaemon(True)
            mikataThread.start()
        print('Initialized > mikataArm')

    def PrintProcessInfo(self):
        """
        Print process information. 
        """

        print('----- Process info -----')
        print('Total loop count > ', self.loopCount)
        for ttask in self.taskTime:
            print('Task time\t > ', ttask, '[s]')
        print('Error count\t > ', self.errorCount)
        print('------------------------')