import numpy as np
import tqdm

from FileIO.FileIO import FileIO

class DataRecordManager:
    dictPosition = {}
    dictRotation = {}
    dictGripperValue = {}
    listTime = []

    def __init__(self, rigidBodyNum: int = 2, bendingSensorNum: int = 1) -> None:
        self.RigidBodyNum       = rigidBodyNum
        self.bendingSensorNum   = bendingSensorNum

        for i in range(self.RigidBodyNum):
            self.dictPosition['RigidBody'+str(i+1)] = []
            self.dictRotation['RigidBody'+str(i+1)] = []

        for i in range(bendingSensorNum):
            self.dictGripperValue['gripperValue'+str(i+1)] = []

    def Record(self, time, position, rotation, bendingSensor):
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
        self.listTime.append([time])

        for i in range(self.RigidBodyNum):
            self.dictPosition['RigidBody'+str(i+1)].append(position['RigidBody'+str(i+1)])
            self.dictRotation['RigidBody'+str(i+1)].append(rotation['RigidBody'+str(i+1)])

        for i in range(self.bendingSensorNum):
            self.dictGripperValue['gripperValue'+str(i+1)].append([bendingSensor['gripperValue'+str(i+1)]])

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

        print('---------- Export completed ----------\n') 
                   