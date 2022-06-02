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
from BendingSensor.BendingSensorManager import BendingSensorManager
from MotionManager.MotionManager import MotionManager
from FileIO.FileIO import FileIO

# ----- Setting: Number ----- #
defaultRigidBodyNum = 3
xArmMovingLimit     = 100
mikataMovingLimit   = 2000
Ratio               = [0.8,0.2,0.2,0.8]  #[RigidBody1-to-xArm, RigidBody1-to-mikataArm, RigidBody2-to-xArm, RigidBody2-to-mikataArm]

class RobotControlManager:
    def __init__(self) ->None:
        fileIO = FileIO()

        dat = fileIO.Read('settings.csv',',')
        xArmIP = [addr for addr in dat if 'xArmIP' in addr[0]][0][1]
        self.xArmIpAddress = xArmIP

    def SendDataToRobot(self,executionTime: int = 120):
        # ----- Process info ----- #
        self.loopCount      = 0
        self.taskTime       = []
        self.errorCount     = 0
        taskStartTime       = 0

        # ----- Instantiating custom classes ----- #
        Behaviour       = MotionBehaviour(defaultRigidBodyNum)
        xArmtransform   = xArmTransform()
        mikatatransform = mikataTransform()
        motionManager   = MotionManager(defaultRigidBodyNum)
        mikatacontrol   = mikataControl()

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
                        xArmPosition,xArmRotation       = Behaviour.GetRatioxArmTransform(localPosition,localRotation,Ratio)
                        mikataPosition,mikataRotation   = Behaviour.GetRatiomikataArmTransform(localPosition,localRotation,Ratio)
                    else:
                        xArmPosition,xArmRotation       = Behaviour.GetxArmTransform(localPosition,localRotation)
                        mikataPosition,mikataRotation   = Behaviour.GetmikataArmTransform(localPosition,localRotation)


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
                    gripperValue = motionManager.GripperControlValue(loopCount=self.loopCount)

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
                        # ----- Send to Arms ----- #
                        arm.set_servo_cartesian(xArmtransform.Transform(isOnlyPosition = False))
                        mikataGoal = [mikataC1, mikataC2, mikataC3, mikataC4, mikataC5]
                        mikatacontrol.SendtomikataArm(mikataGoal)

                    # ----- If xArm error has occured ----- #
                    if arm.has_err_warn:
                        isMoving    = False
                        self.errorCount += 1
                        self.taskTime.append(time.perf_counter() - taskStartTime)
                        print('[ERROR] >> xArm Error has occured. Please enter "r" to reset xArm, or "q" to quit')

                    self.loopCount += 1

                else:
                    keycode = input('Input > "q": quit, "r": Clean error and init arm, "s": start control \n')
                    # ----- Quit program ----- #
                    if keycode == 'q':
                        arm.disconnect()
                        mikatacontrol.ClosePort()
                        break

                    # ----- Reset xArm and gripper ----- #
                    elif keycode == 'r':
                        self.InitializeAll(arm, xArmtransform, mikatatransform, mikatacontrol)
                        # self.InitRobotArm(arm, transform)
                        # self.InitGripper(arm)

                    # ----- Start streaming ----- #
                    elif keycode == 's':
                        Behaviour.SetOriginPosition(motionManager.LocalPosition())
                        Behaviour.SetInversedMatrix(motionManager.LocalRotation())
                        
                        if isRatio:
                            xArmPosition,xArmRotation       = Behaviour.GetRatioxArmTransform(motionManager.LocalPosition(),motionManager.LocalRotation(),Ratio)
                            mikataPosition,mikataRotation   = Behaviour.GetRatiomikataArmTransform(motionManager.LocalPosition(),motionManager.LocalRotation(),Ratio)
                        else:
                            xArmPosition,xArmRotation       = Behaviour.GetxArmTransform(motionManager.LocalPosition(),motionManager.LocalRotation())
                            mikataPosition,mikataRotation   = Behaviour.GetmikataArmTransform(motionManager.LocalPosition(),motionManager.LocalRotation())
                        
                        xArmPosition   = xArmPosition * 1000
                        mikataPosition = mikataPosition * 1000

                        # ----- Set xArm transform ----- #
                        xArmtransform.x, xArmtransform.y, xArmtransform.z           = xArmPosition[2], xArmPosition[0], xArmPosition[1]
                        xArmtransform.roll, xArmtransform.pitch, xArmtransform.yaw  = xArmRotation[2], xArmRotation[0], xArmRotation[1]

                        # ----- Set mikata transform ----- #
                        mikatatransform.x, mikatatransform.y, mikatatransform.z     = mikataPosition[2], mikataPosition[0], mikataPosition[1]

                        beforeX, beforeY, beforeZ              = xArmtransform.x, xArmtransform.y, xArmtransform.z
                        beforeC1, beforeC2, beforeC3, beforeC4 = mikatatransform.Transform()
                        beforeC5                               = mikatatransform.Degree2Current(motionManager.GripperControlValue(loopCount = self.loopCount))

                        motionManager.SetInitialBendingValue()

                        isMoving    = True
                        taskStartTime = time.perf_counter()

        except KeyboardInterrupt:
            print('\nKeyboardInterrupt >> Stop: RobotControlManager.SendDataToRobot()')

            self.taskTime.append(time.perf_counter() - taskStartTime)
            self.PrintProcessInfo()

            arm.disconnect()
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
            __initCurrent = mikatatransform.GetmikataInitialTransform()
            mikatacontrol.SendtomikataArm(__initCurrent)
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