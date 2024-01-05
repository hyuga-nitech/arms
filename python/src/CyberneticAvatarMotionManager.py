import numpy as np
from src.ParticipantMotionManager import ParticipantManager
import lib.self.CustomFunction as cf


class CyberneticAvatarMotionManager:
    def __init__(self, ParticipantConfigs: dict, xArmConfigs: dict, is_Recording) -> None:
        self.xArmConfigs = xArmConfigs

        self.participantManagers = {}
        for participant in ParticipantConfigs.keys():
            self.participantManagers[participant] = ParticipantManager(ParticipantConfigs[participant], is_Recording)

    def GetSharedTransform(self):
        return self.IntegrationWithWeight(self.GetParticipantMotion())

    def IntegrationWithWeight(self, participantMotions: dict):
        sharedMotions = {}
        for xArm in self.xArmConfigs.keys():
            sharedMotions[self.xArmConfigs[xArm]['Mount']] = {'position': [0, 0, 0], 'rotation': [0, 0, 0, 1], 'gripper': 0}

        for participant in participantMotions.keys():
            for mount in participantMotions[participant].keys():
                motions = participantMotions[participant][mount]
                sharedMotions[mount]['position'] += np.array(motions['position']) * motions['weight']
                sharedMotions[mount]['rotation'] = np.dot(cf.convert_to_matrix(np.dot(motions['rotation'][2], cf.Slerp_Quaternion(motions['rotation'][0], motions['rotation'][1], motions['weight']))), sharedMotions[mount]['rotation'])
                sharedMotions[mount]['gripper'] += np.array(motions['gripper']) * motions['weight']

        return sharedMotions

    def GetParticipantMotion(self):
        motions = {}
        for participant in self.participantManagers.keys():
            motions[participant] = self.participantManagers[participant].GetParticipantMotion()

        return motions

    def SetParticipantInitMotion(self):
        for participant in self.participantManagers.keys():
            self.participantManagers[participant].SetParticipantInitPosition()
            self.participantManagers[participant].SetParticipantInitRotation()

    def SetElaspedTime(self, elaspedTtime):
        for participant in self.participantManagers.keys():
            self.participantManagers[participant].SetElaspedTime(elaspedTtime)

    def ExportCSV(self):
        for participant in self.participantManagers.keys():
            self.participantManagers[participant].ExportCSV()
