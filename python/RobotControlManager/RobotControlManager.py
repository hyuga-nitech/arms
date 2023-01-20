# -----------------------------------------------------------------
# Author:          Hyuga Suzuki
# Original Author: Takayoshi Hagiwara (KMD)
# Created:         2022/5/24
# Summary:         xArm,mikataArm,BendingSensorの制御
# -----------------------------------------------------------------

import time
import threading
import json as js
from xarm.wrapper import XArmAPI

# ----- Custom class ----- #
from xArm.xArmTransform import xArmTransform
from mikataArm.mikataArmTransform import mikataTransform
from mikataArm.mikataControl import mikataControl
from MotionBehaviour.MotionBehaviour import MotionBehaviour
from Recorder.DataRecordManager import DataRecordManager
from MotionManager.MotionManager import MotionManager
from VibrotactileFeedback.VibrotactileFeedbackManager import VibrotactileFeedbackManager
from SliderManager.SliderManager import SliderManager

# ----- Setting: Number ----- #
FBmode                  = 1     #0:None, 1:FBEach, 2:FBArms

isTypeFilename          = 1
filename                = "Test" #defaultname

defaultRigidBodyNum     = 2
bendingSensorNum        = 1
xArmMovingLimit         = 500
mikataMovingLimit       = 1000


class RobotControlManager:
    def __init__(self) ->None:
        Parameter_f = open("parameter.json","r")
        self.Parameter_js = js.load(Parameter_f)

    def SendDataToRobot(self,isExportData: bool = True, isEnableArm: bool = True, isSlider: bool = False):
        # ----- Process info ----- #
        self.loopCount      = 0
        self.taskTime       = []
        self.errorCount     = 0
        taskStartTime       = 0

        # ----- Instantiating custom classes ----- #
        Behaviour           = MotionBehaviour(defaultRigidBodyNum)
        xArmtransform       = xArmTransform()
        mikatatransform     = mikataTransform()
        motionManager       = MotionManager(defaultRigidBodyNum,bendingSensorNum,self.Parameter_js["BendingSensorPorts"],self.Parameter_js["BendingSensorBaudrates"])
        mikatacontrol       = mikataControl(self.Parameter_js["mikataArmPort"])
        dataRecordManager   = DataRecordManager(RigidBodyNum=defaultRigidBodyNum)
        vibrotactileManager = VibrotactileFeedbackManager()

        if isSlider:
            slidermanager = SliderManager(self.Parameter_js["SliderPort"])
            SliderThread = threading.Thread(target=slidermanager.receive)
            SliderThread.setDaemon(True)
            SliderThread.start()

        if isEnableArm:
            arm = XArmAPI(self.Parameter_js["xArmIP"])
            self.InitializeAll(arm, xArmtransform, mikatatransform, mikatacontrol)

        # ----- Control flags ----- #
        isMoving = False

        try:
            while True:
                if isMoving:
                    # ----- Get transform data ----- #
                    localPosition    = motionManager.LocalPosition(loopCount=self.loopCount)
                    localRotation    = motionManager.LocalRotation(loopCount=self.loopCount)

                    if isSlider:
                        xratio      = slidermanager.slider_xratio
                        mikataratio = slidermanager.slider_mikataratio

                    else:
                        xratio = self.Parameter_js["xRatio"]
                        mikataratio = self.Parameter_js["mikataRatio"]

                    xArmPosition,xArmRotation  = Behaviour.GetSharedxArmTransform(localPosition,localRotation,xratio)
                    mikataPosition             = Behaviour.GetSharedmikataArmTransform(localPosition,localRotation,mikataratio)

                    xArmPosition   = xArmPosition * 1000
                    mikataPosition = mikataPosition * 1000

                    # ----- Set xArm transform ----- #
                    xArmtransform.x   , xArmtransform.y    , xArmtransform.z    = xArmPosition[2], xArmPosition[0], xArmPosition[1]
                    xArmtransform.roll, xArmtransform.pitch, xArmtransform.yaw  = xArmRotation[2], xArmRotation[0], xArmRotation[1]

                    # ----- Set mikata transform ----- #
                    mikatatransform.x , mikatatransform.y  , mikatatransform.z  = mikataPosition[2], mikataPosition[0], mikataPosition[1]

                    # ----- Bending sensor ----- #
                    dictBendingValue = motionManager.GripperControlValue(loopCount=self.loopCount)
                    gripperValue = 0
                    for i in range(bendingSensorNum):
                        gripperValue += dictBendingValue['gripperValue'+str(i+1)] * self.Parameter_js["GripperRatio"][i]

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
                    if FBmode == 1:
                        vibrotactileManager.FBEachOther(localPosition, localRotation, xratio, mikataratio)
                    elif FBmode == 2:
                        vibrotactileManager.FBArms(localPosition, localRotation, xratio, mikataratio)

                    # ----- Data recording ----- #
                    dataRecordManager.Record(localPosition, localRotation, dictBendingValue)

                    # ----- If xArm error has occured ----- #
                    if isEnableArm and arm.has_err_warn:
                        isMoving    = False
                        self.errorCount += 1
                        self.taskTime.append(time.perf_counter() - taskStartTime)
                        print('[ERROR] >> xArm Error has occured. Please enter "q" to quit')

                    self.loopCount += 1

                else:
                    if isTypeFilename == 1:
                        if FBmode == 0:
                            strFB = 'None'
                        elif FBmode == 1:
                            strFB = 'Each'
                        elif FBmode == 2:
                            strFB = 'Arms'
                        
                        filename = input('Filename?      (Current FBmode = ' + strFB + ' )')
                    
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

                        if isSlider:
                            xratio      = slidermanager.slider_xratio
                            mikataratio = slidermanager.slider_mikataratio

                        else:
                            xratio = self.Parameter_js["xRatio"]
                            mikataratio = self.Parameter_js["mikataRatio"]
                        
                        xArmPosition,xArmRotation  = Behaviour.GetSharedxArmTransform(motionManager.LocalPosition(),motionManager.LocalRotation(),xratio)
                        mikataPosition             = Behaviour.GetSharedmikataArmTransform(motionManager.LocalPosition(),motionManager.LocalRotation(),mikataratio)
                        
                        xArmPosition   = xArmPosition * 1000
                        mikataPosition = mikataPosition * 1000

                        # ----- Set xArm transform ----- #
                        xArmtransform.x   , xArmtransform.y    , xArmtransform.z   = xArmPosition[2], xArmPosition[0], xArmPosition[1]
                        xArmtransform.roll, xArmtransform.pitch, xArmtransform.yaw = xArmRotation[2], xArmRotation[0], xArmRotation[1]

                        # ----- Set mikata transform ----- #
                        mikatatransform.x , mikatatransform.y  , mikatatransform.z = mikataPosition[2], mikataPosition[0], mikataPosition[1]

                        # ----- Bending sensor ----- #
                        dictBendingValue = motionManager.GripperControlValue(loopCount=self.loopCount)
                        gripperValue = 0
                        for i in range(bendingSensorNum):
                            gripperValue += dictBendingValue['gripperValue'+str(i+1)] * self.Parameter_js["GripperRatio"][i]

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
                dataRecordManager.ExportSelf(filename)

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
        