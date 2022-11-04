# -----------------------------------------------------------------
# Author:          Hyuga Suzuki
# Original Author: Takayoshi Hagiwara (KMD)
# Created:         2022/6/3
# Summary:         BendingSensorの値を取得（シリアル通信）
# -----------------------------------------------------------------

import numpy as np
import tqdm

from FileIO.FileIO import FileIO

class DataRecordManager:
    dictPosition = {}
    dictRotation = {}
    dictGripperValue = {}

    def __init__(self, RigidBodyNum: int = 2, bendingSensorNum: int = 1) -> None:
        self.RigidBodyNum       = RigidBodyNum
        self.bendingSensorNum   = bendingSensorNum

        for i in range(self.RigidBodyNum):
            self.dictPosition['RigidBody'+str(i+1)] = []
            self.dictRotation['RigidBody'+str(i+1)] = []

        for i in range(self.bendingSensorNum):
            self.dictGripperValue['gripperValue'+str(i+1)] = []

    def Record(self, position, rotation, bendingSensor):
        """
        Record the data.

        Parameters
        ----------
        position: dict
            Position
        ratation: dict
            Rotation
        bendingSensor: dict
            Bending Sensor values
        """

        for i in range(self.RigidBodyNum):
            self.dictPosition['RigidBody'+str(i+1)].append(position['RigidBody'+str(i+1)])
            self.dictRotation['RigidBody'+str(i+1)].append(rotation['RigidBody'+str(i+1)])

        self.dictGripperValue['gripperValue'].append(bendingSensor)

    def ExportSelf(self, dirPath: str = 'ExportData'):
        """
        Export the data recorded in DataRecordManager as CSV format.

        Parameters
        ----------
        dirPath:(Optional) str
            Directory path (not include the file name)
        """

        fileIO = FileIO()

        transformHeader = ['x','y','z','qx','qy','qz','qw']
        bendingSensorHeader = ['bendingValue']

        print('\n---------- DataRecordManager.ExportSelf ----------')
        print('Writing: RigidBody transform...')
        for i in tqdm.tqdm(range(self.RigidBodyNum), ncols=150):
            npPosition = np.array(self.dictPosition['RigidBody'+str(i+1)])
            npRotation = np.array(self.dictRotation['RigidBody'+str(i+1)])
            npRigidBodyTransform = np.concatenate([npPosition, npRotation], axis=1)

            fileIO.ExportAsCSV(npRigidBodyTransform, dirPath, 'Transform_RigidBody_'+str(i+1), transformHeader)

        print('Writing: Gripper value...')
        npBendingSensorValue = np.array(self.dictGripperValue['gripperValue'])

        fileIO.ExportAsCSV(npBendingSensorValue, dirPath, 'GripperValue', bendingSensorHeader)

        print('---------- Export completed ----------\n')            