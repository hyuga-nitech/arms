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

    def __init__(self) -> None:
        bending_f = open("bending_sensor_setting.json","r")
        self.bending_sensor_js = js.load(bending_f)
        xArm_setting_f = open("xArm_setting.json","r")
        self.xArm_js = js.load(xArm_setting_f)
        rigidbody_f = open("rigidbody_setting.json","r")
        self.rigidbody_js = js.load(rigidbody_f)

        self.RigidBodyNum = len(self.rigidbody_js["RigidBodyConfig"])

        for i in range(self.RigidBodyNum):
            self.dictPosition['RigidBody'+str(i+1)] = []
            self.dictRotation['RigidBody'+str(i+1)] = []

        for i in range(self.bendingSensorNum):
            self.dictGripperValue['gripperValue'+str(i+1)] = []

        for i in range(self.RigidBodyNum):
            self.dictRatio['RigidBody'+str(i+1)] = []

    def Record(self, time, position, rotation, bendingSensor, ratio):
        self.listTime.append([time])

        for i in range(self.RigidBodyNum):
            self.dictPosition['RigidBody'+str(i+1)].append(position['RigidBody'+str(i+1)])
            self.dictRotation['RigidBody'+str(i+1)].append(rotation['RigidBody'+str(i+1)])

        for i in range(self.bendingSensorNum):
            self.dictGripperValue['gripperValue'+str(i+1)].append([bendingSensor['gripperValue'+str(i+1)]])

        for i in range(self.RigidBodyNum):
            self.dictRatio['RigidBody'+str(i+1)].append(ratio['RigidBody'+str(i+1)])

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
            npGripper = np.array(self.dictGripperValue['gripperValue'+str(i+1)])
            npBendingSensorValue = np.concatenate([npTime, npGripper], axis=1)
            fileIO.ExportAsCSV(npBendingSensorValue, dirPath, name+'_GripperValue_'+str(i+1), bendingSensorHeader)

        print('Writing: Ratio...')
        for i in tqdm.tqdm(range(self.RigidBodyNum), ncols=150):
            npRatio = np.array(self.dictRatio['RigidBody'+str(i+1)])
            npRatioValue = np.concatenate([npTime, npRatio], axis=1)

            fileIO.ExportAsCSV(npRatioValue, dirPath, name+'_Ratio_'+str(i+1), ratioHeader)

        print('---------- Export completed ----------\n') 
                   