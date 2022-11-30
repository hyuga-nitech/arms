# -----------------------------------------------------------------
# Author:          Hyuga Suzuki
# Original Author: Takayoshi Hagiwara (KMD)
# Created:         2022/5/24
# Summary:         xArm,mikataArm,BendingSensorの制御
# -----------------------------------------------------------------

import time
import threading
from xarm.wrapper import XArmAPI

# ----- Custom class ----- #
from xArm.xArmTransform import xArmTransform
from mikataArm.mikataArmTransform import mikataTransform
from mikataArm.mikataControl import mikataControl
from MotionBehaviour.MotionBehaviour import MotionBehaviour
from Recorder.DataRecordManager import DataRecordManager
from MotionManager.MotionManager import MotionManager
from FileIO.FileIO import FileIO
from VibrotactileFeedback.VibrotactileFeedbackManager import VibrotactileFeedbackManager
from SliderManager.SliderManager import SliderManager

# ----- Setting: Number ----- #
defaultRigidBodyNum     = 2
xArmMovingLimit         = 500
mikataMovingLimit       = 1000

class RobotControlManager:
    def __init__(self) ->None:
        fileIO = FileIO()

        dat = fileIO.Read('settings.csv',',')
        self.xArmIpAddress = [addr for addr in dat if 'xArmIP' in addr[0]][0][1]
        self.SliderPort    = [addr for addr in dat if 'SliderPort' in addr[0]][0][1]

    def SendDataToRobot(self,isExportData: bool = True, isEnableArm: bool = True):
        # ----- Process info ----- #
        self.loopCount      = 0
        self.taskTime       = []
        self.errorCount     = 0
        taskStartTime       = 0

        # ----- Instantiating custom classes ----- #
        Behaviour           = MotionBehaviour(defaultRigidBodyNum)
        xArmtransform       = xArmTransform()
        mikatatransform     = mikataTransform()
        motionManager       = MotionManager(defaultRigidBodyNum)
        mikatacontrol       = mikataControl()
        dataRecordManager   = DataRecordManager(RigidBodyNum=defaultRigidBodyNum)
        vibrotactileManager = VibrotactileFeedbackManager()
        slidermanager       = SliderManager(self.SliderPort)


        SliderThread = threading.Thread(target=slidermanager.receive)
        SliderThread.setDaemon(True)
        SliderThread.start()


        if isEnableArm:
            arm = XArmAPI(self.xArmIpAddress)
            self.InitializeAll(arm, xArmtransform, mikatatransform, mikatacontrol)

        # ----- Control flags ----- #
        isMoving = False

        try:
            while True:
                if isMoving:
                    # ----- Get transform data ----- #
                    localPosition    = motionManager.LocalPosition(loopCount=self.loopCount)
                    localRotation    = motionManager.LocalRotation(loopCount=self.loopCount)

                    slider_xratio      = slidermanager.slider_xratio
                    slider_mikataratio = slidermanager.slider_mikataratio

                    xArmPosition,xArmRotation  = Behaviour.GetSharedxArmTransform(localPosition,localRotation,slider_xratio)
                    mikataPosition             = Behaviour.GetSharedmikataArmTransform(localPosition,localRotation,slider_mikataratio)

                    xArmPosition   = xArmPosition * 1000
                    mikataPosition = mikataPosition * 1000

                    # ----- Set xArm transform ----- #
                    xArmtransform.x   , xArmtransform.y    , xArmtransform.z    = xArmPosition[2], xArmPosition[0], xArmPosition[1]
                    xArmtransform.roll, xArmtransform.pitch, xArmtransform.yaw  = xArmRotation[2], xArmRotation[0], xArmRotation[1]

                    # ----- Set mikata transform ----- #
                    mikatatransform.x , mikatatransform.y  , mikatatransform.z  = mikataPosition[2], mikataPosition[0], mikataPosition[1]

                    # ----- Bending sensor ----- #
                    gripperValue = motionManager.GripperControlValue(loopCount=self.loopCount)

                    # ----- Calculate mikata Current ----- #
                    mikataC1, mikataC2, mikataC3, mikataC4 = mikatatransform.Transform()
                    mikataC5                               = mikatatransform.Degree2Current(gripperValue)

                    # ----- Safety Check -----#
                    xArmdiffX    = xArmtransform.x - beforeX
                    xArmdiffY    = xArmtransform.y - beforeY
                    xArmdiffZ    = xArmtransform.z - beforeZ
                    
                    mikatadiffC1 = mikataC1 - beforeC1
                    mikatadiffC2 = mikataC2 - beforeC2
                    mikatadiffC3 = mikataC3 - beforeC3
                    mikatadiffC4 = mikataC4 - beforeC4
                    mikatadiffC5 = mikataC5 - beforeC5

                    beforeX , beforeY , beforeZ                      = xArmtransform.x, xArmtransform.y, xArmtransform.z
                    beforeC1, beforeC2, beforeC3, beforeC4, beforeC5 = mikataC1, mikataC2, mikataC3, mikataC4, mikataC5

                    if abs(xArmdiffX) > xArmMovingLimit or abs(xArmdiffY) > xArmMovingLimit or abs(xArmdiffZ) > xArmMovingLimit:
                        isMoving = False
                        print('[ERROR] >> A rapid movement has occurred in xArm. Please enter "q" to quit')
                    elif abs(mikatadiffC1) > mikataMovingLimit or abs(mikatadiffC2) > mikataMovingLimit or abs(mikatadiffC3) > mikataMovingLimit or abs(mikatadiffC4) > mikataMovingLimit:
                        isMoving = False
                        print('[ERROR] >> A rapid movement has occurred in mikataArm. Please enter "q" to quit')
                    elif abs(mikatadiffC5) > mikataMovingLimit:
                        isMoving = False
                        print('[ERROR] >> A rapid movement has occurred in mikataArm Gripper. Please enter "q" to quit')
                    else:
                        if isEnableArm:
                            # ----- Send to Arms ----- #
                            arm.set_servo_cartesian(xArmtransform.Transform(isOnlyPosition = False))
                            mikatacontrol.dxl_goal_position = [mikataC1, mikataC2, mikataC3, mikataC4, mikataC5]

                    # ----- Vibrotactile Feedback ----- #
                    vibrotactileManager.forPhantom(localPosition, localRotation, slider_xratio, slider_mikataratio)

                    # ----- Data recording ----- #
                    dataRecordManager.Record(localPosition, localRotation, gripperValue)

                    # ----- If xArm error has occured ----- #
                    if isEnableArm and arm.has_err_warn:
                        isMoving    = False
                        self.errorCount += 1
                        self.taskTime.append(time.perf_counter() - taskStartTime)
                        print('[ERROR] >> xArm Error has occured. Please enter "q" to quit')

                    self.loopCount += 1

                else:
                    keycode = input('Input > "q": quit, "s": start control \n')

                    # ----- Quit program ----- #
                    if keycode == 'q':
                        if isEnableArm:
                            arm.disconnect()
                            mikatacontrol.ClosePort()

                        self.PrintProcessInfo()
                        break

                    # ----- Start streaming ----- #
                    elif keycode == 's':
                        Behaviour.SetOriginPosition(motionManager.LocalPosition())
                        Behaviour.SetInversedMatrix(motionManager.LocalRotation())

                        slider_xratio      = slidermanager.slider_xratio
                        slider_mikataratio = slidermanager.slider_mikataratio
                        
                        xArmPosition,xArmRotation  = Behaviour.GetSharedxArmTransform(motionManager.LocalPosition(),motionManager.LocalRotation(),slider_xratio)
                        mikataPosition             = Behaviour.GetSharedmikataArmTransform(motionManager.LocalPosition(),motionManager.LocalRotation(),slider_mikataratio)
                        
                        xArmPosition   = xArmPosition * 1000
                        mikataPosition = mikataPosition * 1000

                        # ----- Set xArm transform ----- #
                        xArmtransform.x   , xArmtransform.y    , xArmtransform.z   = xArmPosition[2], xArmPosition[0], xArmPosition[1]
                        xArmtransform.roll, xArmtransform.pitch, xArmtransform.yaw = xArmRotation[2], xArmRotation[0], xArmRotation[1]

                        # ----- Set mikata transform ----- #
                        mikatatransform.x , mikatatransform.y  , mikatatransform.z = mikataPosition[2], mikataPosition[0], mikataPosition[1]

                        # ----- Bending sensor ----- #
                        gripperValue = motionManager.GripperControlValue(loopCount=self.loopCount)

                        beforeX , beforeY , beforeZ            = xArmtransform.x, xArmtransform.y, xArmtransform.z
                        beforeC1, beforeC2, beforeC3, beforeC4 = mikatatransform.Transform()
                        beforeC5                               = mikatatransform.Degree2Current(gripperValue)

                        isMoving = True
                        taskStartTime = time.perf_counter()

        except KeyboardInterrupt:
            print('\nKeyboardInterrupt >> Stop: RobotControlManager.SendDataToRobot()')

            self.taskTime.append(time.perf_counter() - taskStartTime)
            self.PrintProcessInfo()
            
            if isExportData:
                dataRecordManager.ExportSelf()

            if isEnableArm:
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

        robotArm.set_mode(1)
        robotArm.set_state(state=0)

        # ----- mikata Arm ----- #
        mikatacontrol.OpenPort()
        if isSetInitPosition:
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
        