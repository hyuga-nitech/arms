# -----------------------------------------------------------------
# Author:          Hyuga Suzuki
# Original Author: Takayoshi Hagiwara (KMD)
# Created:         2022/5/24
# Summary:         xArm,mikataArm,BendingSensorの制御
# -----------------------------------------------------------------

import time
import threading
import json as js
from xArm.wrapper import XArmAPI

# ----- Custom class ----- #
from xArmTransform.xArmTransform import xArmTransform
from mikataArm.mikataArmTransform import mikataTransform
from mikataArm.mikataControl import mikataControl
from MotionBehaviour.MotionBehaviour import MotionBehaviour
from Recorder.DataRecordManager import DataRecordManager
from MotionManager.MotionManager import MotionManager
from VibrotactileFeedback.VibrotactileFeedbackManager import VibrotactileFeedbackManager
from SliderManager.SliderManager import SliderManager

# ----- Setting ----- #
OperatorNum             = 2
PairID                  = 1
OperatorID              = 1   #Only OperatorNum = 2

Practice                = 1
PracArm                 = 0   # 0:xArm, 1:mikataArm
TaskNum                 = 1

# ----- Core Setting ----- #
RigidBodyNum            = 2
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

        # ----- Auto Set ----- #
        if Practice == 1:
            executionTime = 60
        else:
            executionTime = 9999

        if OperatorNum == 1:
            Opemode = 'Solo'
        elif OperatorNum == 2:
            Opemode = 'Pair'
        else:
            print('[ERROR] -> Please Check the OperatorNum')

        if Practice == 1:
            if PracArm == 0:
                Taskmode = 'PracticexArm'
            elif PracArm == 1:
                Taskmode = 'PracticemikataArm'
        elif Practice == 0:
            Taskmode = 'Avoidance'
        else:
            print('[ERROR] -> Please Check the Practice')

        if OperatorNum == 1:
            filename = Opemode + Taskmode + str(TaskNum)
        elif OperatorNum == 2:
            filename = Opemode + str(OperatorID) +Taskmode + str(TaskNum)
            
        # ----- Instantiating custom classes ----- #
        Behaviour           = MotionBehaviour(OperatorNum)
        xArmtransform       = xArmTransform()
        mikatatransform     = mikataTransform()
        motionManager       = MotionManager(RigidBodyNum,bendingSensorNum,self.Parameter_js["BendingSensorPorts"],self.Parameter_js["BendingSensorBaudrates"])
        mikatacontrol       = mikataControl(self.Parameter_js["mikataArmPort"])
        dataRecordManager   = DataRecordManager(rigidBodyNum=RigidBodyNum)
        vibrotactileManager = VibrotactileFeedbackManager()

        if isSlider:
            slidermanager = SliderManager(self.Parameter_js["SliderPort"],OperatorNum)
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
                if isMoving and (time.perf_counter() - taskStartTime > executionTime):
                    isMoving = False

                    self.taskTime.append(time.perf_counter() - taskStartTime)
                    self.PrintProcessInfo()
                    
                    if isExportData:
                        dataRecordManager.ExportSelf(filename)

                    if isEnableArm:
                        arm.disconnect()
                        mikatacontrol.ClosePort()

                    print('----- Finish Task -----')
                    break

                if isMoving:
                    # ----- Get transform data ----- #
                    localPosition    = motionManager.LocalPosition(loopCount=self.loopCount)
                    localRotation    = motionManager.LocalRotation(loopCount=self.loopCount)

                    if Practice == 1:
                        if PracArm == 0:
                            xratio      = [1,1,0,0]
                            mikataratio = [0,0]
                        elif PracArm == 1:
                            xratio      = [0,0,0,0]
                            mikataratio = [1,0]

                    else:
                        time2ratio = round((time.perf_counter() - taskStartTime)%20)
                        if time2ratio <= 10:
                            xratio      = [1-time2ratio*0.1,1-time2ratio*0.1,time2ratio*0.1,time2ratio*0.1]
                            mikataratio = [time2ratio*0.1,1-time2ratio*0.1]
                        else:
                            time2ratio = 20- time2ratio
                            xratio      = [1-time2ratio*0.1,1-time2ratio*0.1,time2ratio*0.1,time2ratio*0.1]
                            mikataratio = [time2ratio*0.1,1-time2ratio*0.1]

                    xArmPosition,xArmRotation      = Behaviour.GetSharedxArmTransform(localPosition,localRotation,xratio)
                    mikataPosition,mikataRotation  = Behaviour.GetSharedmikataArmTransform(localPosition,localRotation,mikataratio)

                    xArmPosition   = xArmPosition * 1000
                    mikataPosition = mikataPosition * 1000

                    # ----- Set xArm transform ----- #
                    xArmtransform.x   , xArmtransform.y    , xArmtransform.z    = xArmPosition[2], xArmPosition[1], -1*xArmPosition[0]
                    xArmtransform.roll, xArmtransform.pitch, xArmtransform.yaw  = xArmRotation[2], xArmRotation[1], -1*xArmRotation[0]

                    # ----- Set mikata transform ----- #
                    mikatatransform.x , mikatatransform.y  , mikatatransform.z  = mikataPosition[2], mikataPosition[1], -1*mikataPosition[0]
                    mikatatransform.pitch                                       = mikataRotation[1]

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
                    if OperatorNum == 2:
                        vibrotactileManager.FBEachOther(localPosition, localRotation, xratio, mikataratio)

                    # ----- Data recording ----- #
                    Time = time.perf_counter()
                    dataRecordManager.Record(Time, localPosition, localRotation, dictBendingValue)

                    # ----- If xArm error has occured ----- #
                    if isEnableArm and arm.has_err_warn:
                        isMoving    = False
                        self.errorCount += 1
                        self.taskTime.append(time.perf_counter() - taskStartTime)
                        print('[ERROR] >> xArm Error has occured. Please enter "q" to quit')

                    self.loopCount += 1

                else:
                    print('Cullent OutputFilename = ' + filename)
                    keycode = input('Input > "q": quit, "s": start control, "r": rename Outputfile \n')

                    # ----- Quit program ----- #
                    if keycode == 'q':
                        if isEnableArm:
                            arm.disconnect()
                            mikatacontrol.ClosePort()

                        self.PrintProcessInfo()
                        break

                    # ----- Rename ----- #
                    elif keycode == 'r':
                        filename = input('(Current Mode : '+Opemode+' '+Taskmode+' '+str(TaskNum)+')\nFilename?:')
                        continue

                    # ----- Start streaming ----- #
                    elif keycode == 's':
                        Behaviour.SetOriginPosition(motionManager.LocalPosition())
                        Behaviour.SetInversedMatrix(motionManager.LocalRotation())

                        if Practice == 1:
                            if PracArm == 0:
                                xratio      = [1,1,0,0]
                                mikataratio = [0,0]
                            elif PracArm == 1:
                                xratio      = [0,0,0,0]
                                mikataratio = [1,0]

                        else:
                            xratio      = [1,1,0,0]
                            mikataratio = [0,0]

                        xArmPosition,xArmRotation      = Behaviour.GetSharedxArmTransform(motionManager.LocalPosition(),motionManager.LocalRotation(),xratio)
                        mikataPosition,mikataRotation  = Behaviour.GetSharedmikataArmTransform(motionManager.LocalPosition(),motionManager.LocalRotation(),mikataratio)
                        
                        xArmPosition   = xArmPosition * 1000
                        mikataPosition = mikataPosition * 1000

                        # ----- Set xArm transform ----- #
                        xArmtransform.x   , xArmtransform.y    , xArmtransform.z   = xArmPosition[2], xArmPosition[1], -1*xArmPosition[0]
                        xArmtransform.roll, xArmtransform.pitch, xArmtransform.yaw = xArmRotation[2], xArmRotation[1], -1*xArmRotation[0]

                        # ----- Set mikata transform ----- #
                        mikatatransform.x , mikatatransform.y  , mikatatransform.z = mikataPosition[2], mikataPosition[1], -1*mikataPosition[0]
                        mikatatransform.pitch                                       = mikataRotation[1]

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
        