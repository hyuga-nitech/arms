import numpy as np
import tqdm
import json as js

from FileIO.FileIO import FileIO

class DataRecordManager:
    dictPosition = {}
    dictRotation = {}
    dictGripperValue = {}
    dictRatio = {}
    listTime = []
    
    listAssistTime = []
    listCurrentArmVel = []
    listAtTimeArmVel = []

    def __init__(self) -> None:
        bending_f = open("bending_sensor_setting.json","r")
        self.bending_sensor_js = js.load(bending_f)
        xArm_setting_f = open("xArm_setting.json","r")
        self.xArm_js = js.load(xArm_setting_f)
        rigidbody_f = open("rigidbody_setting.json","r")
        self.rigidbody_js = js.load(rigidbody_f)

        self.RigidBodyNum = len(self.rigidbody_js["RigidBodyConfig"])
        self.bendingSensorNum = len(self.bending_sensor_js["BendingSensorConfig"])

        for rigidbody in self.rigidbody_js["RigidBodyConfig"]:
            self.dictPosition[rigidbody] = []
            self.dictRotation[rigidbody] = []
            self.dictRatio[rigidbody] = []

        for bendingsensor in self.bending_sensor_js["BendingSensorConfig"]:
            self.dictGripperValue[bendingsensor] = []

    def record(self, time, position, rotation, bendingSensor, ratio):
        self.listTime.append([time])

        for rigidbody in self.rigidbody_js["RigidBodyConfig"]:
            self.dictPosition[rigidbody].append(position[rigidbody])
            self.dictRotation[rigidbody].append(rotation[rigidbody])
            self.dictRatio[rigidbody].append(ratio[rigidbody])

        for bendingsensor in self.bending_sensor_js["BendingSensorConfig"]:
            self.dictGripperValue[bendingsensor].append([bendingSensor[bendingsensor]])

    def record_arm(self, time, current_vel, at_time_vel):
        self.listAssistTime.append([time])

        self.listCurrentArmVel.append([current_vel])
        self.listAtTimeArmVel.append([at_time_vel])

    def ExportSelf(self, name, dirPath: str = 'ExportData'):
        """
        Export the data recorded in DataRecordManager as CSV format.

        Parameters
        ----------
        dirPath:(Optional) str
            Directory path (not include the file name)
        """

        fileIO = FileIO()

        transformHeader = ['time','x','y','z','qx','qy','qz','qw']
        bendingSensorHeader = ['time','bendingValue']
        ratioHeader = ['time','position','rotation']

        print('\n---------- DataRecordManager.ExportSelf ----------')
        print('Making: Time axis list...')
        npTime = np.array(self.listTime)

        print('Writing: RigidBody transform...')
        for i in tqdm.tqdm(range(self.RigidBodyNum), ncols=150):
            npPosition = np.array(self.dictPosition['RigidBody'+str(i+1)])
            npRotation = np.array(self.dictRotation['RigidBody'+str(i+1)])
            npRigidBodyTransform = np.concatenate([npTime, npPosition, npRotation], axis=1)

            fileIO.ExportAsCSV(npRigidBodyTransform, dirPath, name+'_Transform_RigidBody_'+str(i+1), transformHeader)

        print('Writing: Gripper value...')
        for i in tqdm.tqdm(range(self.bendingSensorNum), ncols=150):
            npGripper = np.array(self.dictGripperValue['BendingSensor'+str(i+1)])
            npBendingSensorValue = np.concatenate([npTime, npGripper], axis=1)
            fileIO.ExportAsCSV(npBendingSensorValue, dirPath, name+'_GripperValue_'+str(i+1), bendingSensorHeader)

        print('Writing: Ratio...')
        for i in tqdm.tqdm(range(self.RigidBodyNum), ncols=150):
            npRatio = np.array(self.dictRatio['RigidBody'+str(i+1)])
            npRatioValue = np.concatenate([npTime, npRatio], axis=1)

            fileIO.ExportAsCSV(npRatioValue, dirPath, name+'_Ratio_'+str(i+1), ratioHeader)

        print('---------- Export completed ----------\n') 

    def export_arm(self, name, dirPath: str = 'ExportData'):

        fileIO = FileIO()

        armHeader = ['time','CurrentVel','AtTimeVel']

        npTime = np.array(self.listAssistTime)
        npCurrent = np.array(self.listCurrentArmVel)
        npAtTime = np.array(self.listAtTimeArmVel)

        npArmVel = np.concatenate([npTime, npCurrent, npAtTime], axis=1)

        fileIO.ExportAsCSV(npArmVel, dirPath, name+'_MinimumJerk_ArmVel', armHeader)