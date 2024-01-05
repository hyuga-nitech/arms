import json
import threading

import numpy as np

import lib.self.CustomFunction as cf
from src.DataManager import DataLoadManager, DataPlotManager, DataRecordManager
from src.MinimumJerk import MinimumJerk
# # ----- Custom class ----- #
from src.OptiTrackStreamingManager import OptiTrackStreamingManager
from src.SensorManager import GripperSensorManager


class ParticipantManager:
    with open('docs/settings_single.json', 'r') as settings_file:
        settings = json.load(settings_file)
    xArmConfig = {}
    for xArm in settings['xArmConfigs'].keys():
        xArmConfig[settings['xArmConfigs'][xArm]['Mount']] = settings['xArmConfigs'][xArm]

    def __init__(self, ParticipantConfig: dict, is_Recording) -> None:
        self.participantConfig = ParticipantConfig
        self.position = []
        self.rotation = []

        self.motionManagers= {}
        for Config in self.participantConfig:
            self.motionManagers[Config['Mount']] = MotionManager(Config, ParticipantManager.xArmConfig[Config['Mount']], is_Recording)

    def GetParticipantMotion(self):
        participantMotions = {}
        for Config in self.participantConfig:
            participantMotions[Config['Mount']] = self.motionManagers[Config['Mount']].GetMotionData()

        return participantMotions

    def SetParticipantInitPosition(self):
        for Config in self.participantConfig:
            self.motionManagers[Config['Mount']].SetInitPosition()

    def SetParticipantInitRotation(self):
        for Config in self.participantConfig:
            self.motionManagers[Config['Mount']].SetInitRotation()

    def SetElaspedTime(self, elaspedTime):
        for Config in self.participantConfig:
                    self.motionManagers[Config['Mount']].SetElaspedTime(elaspedTime)

    def ExportCSV(self):
        for Config in self.participantConfig:
            self.motionManagers[Config['Mount']].ExportCSV()

    def PlotGraph(self):
        for Config in self.participantConfig:
            self.motionManagers[Config['Mount']].PlotGraph()

class MotionManager:
    optiTrackStreamingManager = OptiTrackStreamingManager(mocapServer = "133.68.108.109", mocapLocal = "133.68.108.109")
    streamingThread = threading.Thread(target = optiTrackStreamingManager.stream_run)
    streamingThread.setDaemon(True)
    streamingThread.start()

    def __init__(self, Config, xArmConfig, is_Recording) -> None:
        self.mount = Config['Mount']
        self.rigidBody = str(Config['RigidBody'])
        self.weight = Config['Weight']
        self.initPosition = []
        self.initQuaternion = []
        self.initInverseMatrix = []
        self.initFlag = False
        self.updateInitPosition = []
        self.updateInitQuaternion = []
        self.updateInitInverseMatrix = []
        self.iter_initPos = self.iter_initRot = []
        self.isMoving_Pos = self.isMoving_Rot = self.isMoving_Grip = self.isMoving = False
        self.pos_list = []
        self.pos_list2 = []
        self.pos_list3 = []
        self.pos_box = []
        self.vel_list = []
        self.vel_box = []
        self.dt = 1/ 200
        self.before_time = 0
        self.recording = is_Recording
        self.elaspedTime = 0
        self.auto_list = []

        self.automation = MinimumJerk(Config['Target'], xArmConfig)

        self.sensorManager = GripperSensorManager(Config['SerialCOM'], BandRate = 115200)
        sensorThread = threading.Thread(target = self.sensorManager.start_receiving)
        sensorThread.setDaemon(True)
        sensorThread.start()

        if self.recording:
            self.recorder_pos = DataRecordManager(header = ['x', 'y', 'z'], fileName='pos')
            self.recorder_rot = DataRecordManager(header = ['x', 'y', 'z', 'w'], fileName='rot')
            self.recorder_grip = DataRecordManager(header = ['grip'], fileName='grip')
            self.recorder_time = DataRecordManager(header = ['time'], fileName='time')

        else:
            MotionManager.optiTrackStreamingManager.position[self.rigidBody] = np.zeros(3)
            MotionManager.optiTrackStreamingManager.rotation[self.rigidBody] = np.zeros(4)

    def GetMotionData(self):
        position, rotation, gripper = self.GetPosition(), self.GetRotation(), self.GetGripperValue()
        velocity, accelaration = self.GetParticipnatMotionInfo(position)

        if self.isMoving_Pos == self.isMoving_Rot == self.isMoving_Grip == False:
            if self.isMoving == True:
                self.UpdateInitPosition(position)
                self.UpdateInitRotation(rotation)
                self.isMoving = False

            if self.isMoving == False and self.initFlag == True:
                posFlag = self.LerpInitPosition()
                rotFlag = self.SlerpInitRotation()
                if posFlag == rotFlag == False:
                    self.initFlag = False
                    print('finish_automation')

            if self.initFlag == False:
                if self.automation.MonitoringMotion(position, rotation, gripper, velocity, accelaration):
                    self.isMoving_Pos = self.isMoving_Rot = self.isMoving_Grip = self.isMoving = self.initFlag = True

        return {'position': position, 'rotation': rotation, 'gripper': gripper, 'weight': self.weight}

    def GetPosition(self):
        if self.Simulation:
            position = self.data_pos.getdata()
        else:
            position = MotionManager.optiTrackStreamingManager.position[self.rigidBody]
            if self.recording:
                self.recorder_pos.record(position)

        if self.isMoving_Pos == self.isMoving_Rot == self.isMoving_Grip == False:
            self.position = cf.ConvertAxis_Position(position * 1000, self.mount) - np.array(self.initPosition)

        else:
            pos_auto, self.isMoving_Pos, weight, velocity_auto = self.automation.GetPosition(self.elaspedTime)
            position = cf.ConvertAxis_Position(position * 1000, self.mount) - np.array(self.initPosition)
            self.position = pos_auto * weight + position * (1 - weight)

            # self.recorder.record(np.hstack([position[0], pos_auto[0],  self.elaspedTime]))

        return self.position

    def GetRotation(self):
        if self.Simulation:
            rotation = self.data_rot.getdata()
        else:
            rotation = MotionManager.optiTrackStreamingManager.rotation[self.rigidBody]
            if self.recording:
                self.recorder_rot.record(rotation)

        if self.isMoving_Pos == self.isMoving_Rot == self.isMoving_Grip == False:
            quaternion = cf.CnvertAxis_Rotation(rotation, self.mount)
            if quaternion[3] < 0:
                quaternion = -np.array(quaternion)
            self.rotation = quaternion
        else:
            rot_auto, self.isMoving_Rot, weight = self.automation.GetRotation(self.elaspedTime)
            quaternion = cf.CnvertAxis_Rotation(rotation, self.mount)
            if quaternion[3] < 0:
                quaternion = -np.array(quaternion)
            self.rotation = cf.Slerp_Quaternion(rot_auto, quaternion, weight)
            # print(self.rotation, rot_auto)
            # print(weight)

        return [self.rotation, self.initQuaternion, self.initInverseMatrix]

    def GetGripperValue(self):
        if self.Simulation:
            grip = self.data_grip.getdata()[0]
        else:
            grip = cf.ConvertSensorToGripper(self.sensorManager.sensorValue)
            if self.recording:
                self.recorder_grip.record([grip])

        if self.isMoving_Pos == self.isMoving_Rot == self.isMoving_Grip == False:
            gripper = grip
        else:
            gripper, self.isMoving_Grip = self.automation.GetGripperValue()

        return gripper

    def SetInitPosition(self):
        if self.Simulation:
            initPosition = self.data_pos.getdata()
        else:
            initPosition = MotionManager.optiTrackStreamingManager.position[self.rigidBody]
            if self.recording:
                self.recorder_pos.record(initPosition)

        self.initPosition = cf.ConvertAxis_Position(initPosition * 1000, self.mount)

    def SetInitRotation(self):
        if self.Simulation:
            initQuaternion = self.data_rot.getdata()
        else:
            initQuaternion = MotionManager.optiTrackStreamingManager.rotation[self.rigidBody]
            if self.recording:
                self.recorder_rot.record(initQuaternion)

        quaternion = cf.CnvertAxis_Rotation(initQuaternion, self.mount)
        if quaternion[3] < 0:
            quaternion = -np.array(quaternion)
        self.initQuaternion = self.automation.q_init = quaternion
        self.initInverseMatrix = cf.Convert2InverseMatrix(quaternion)

    def UpdateInitPosition(self, position):
        self.updateInitPosition = self.initPosition - (np.array(position) - self.GetPosition())
        p_list = np.linspace(self.updateInitPosition, self.initPosition, 500)
        self.iter_initPos = iter(p_list)

    def UpdateInitRotation(self, rotation):
        q_zero = [0, 0, 0, 1]
        quaternion, initQuaternion, initInveseMatrix = self.GetRotation()
        q_inverse = np.dot(cf.Convert2InverseMatrix(quaternion), q_zero)
        self.updateInitQuaternion = np.dot(initInveseMatrix, np.dot(cf.convert_to_matrix(rotation[0]), q_inverse))
        self.updateInitQuaternion = np.dot(cf.Convert2InverseMatrix(self.updateInitQuaternion), q_zero)
        weight_list = np.linspace(0, 1, 300)
        q_list = []
        for weight in weight_list:
            q_list.append(cf.Slerp_Quaternion(self.initQuaternion, self.updateInitQuaternion, weight))

        self.iter_initRot = iter(q_list)

    def LerpInitPosition(self):
        try:
            self.initPosition, flag = next(self.iter_initPos), True
        except StopIteration:
            flag = False

        return flag

    def SlerpInitRotation(self):
        try:
            rot = next(self.iter_initRot)
            # print(rot)
            self.initQuaternion, self.initInverseMatrix, flag = rot, cf.Convert2InverseMatrix(rot), True
        except StopIteration:
            flag = False
            self.initQuaternion = self.automation.q_init
            self.initInverseMatrix = cf.Convert2InverseMatrix(self.automation.q_init)

        return flag

    def GetParticipnatMotionInfo(self, position, interval = 25):
        self.pos_list.append(position)

        if len(self.pos_list) == interval+1:
            vel = np.linalg.norm(np.polyfit(np.linspace(0, self.dt * (interval+1), (interval+1)), self.pos_list, 1)[0])
            del self.pos_list[0]

        else:
            vel = 0

        # self.recorder2.custom_record(np.hstack((self.elaspedTime, [vel])))

        # print(vel)

        return vel, 0

    def GetParticipnatMotionInfo2(self, position, interval = 25):
        self.pos_list2.append(position)

        if len(self.pos_list2) == interval+1:
            vel = np.linalg.norm(np.polyfit(np.linspace(0, self.dt * (interval+1), (interval+1)), self.pos_list2, 1)[0])
            del self.pos_list2[0]

        else:
            vel = 0

        # self.recorder2.record(np.hstack(([vel], self.elaspedTime)))

        # print(vel)

        return vel, 0

    def GetParticipnatMotionInfo3(self, position, interval = 25):
        self.pos_list3.append(position)

        if len(self.pos_list3) == interval+1:
            vel = np.linalg.norm(np.polyfit(np.linspace(0, self.dt * (interval+1), (interval+1)), self.pos_list3, 1)[0])
            del self.pos_list3[0]

        else:
            vel = 0

        # self.recorder2.record(np.hstack(([vel], self.elaspedTime)))

        # print(vel)

        return vel, 0

    def ExportCSV(self):
        self.recorder_pos.exportAsCSV()
        self.recorder_rot.exportAsCSV()
        self.recorder_grip.exportAsCSV()
        self.recorder_time.exportAsCSV()
        # self.recorder2.exportAsCSV()
        # self.recorder3.exportAsCSV()

    def SetElaspedTime(self, elaspedTime):
        self.elaspedTime = elaspedTime
        if self.recording == True:
            self.recorder_time.record([self.elaspedTime])
