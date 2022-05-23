# -----------------------------------------------------------------
# Author:          Hyuga Suzuki
# Original Author: Takayoshi Hagiwara (KMD)
# Created:         2022/5/20
# Summary:         RigidBody座標に基づくアーム座標の指令値計算
# -----------------------------------------------------------------

import numpy as np
import scipy.spatial.transform as scitransform
import math

class MotionBehaviour:
    originPositions     = {}
    inversedMatrix      = {}

    Positions           = {}
    Rotations           = {}

    def __init__(self,defaultRigidBodyNum: int = 3) -> None:
        for i in range(defaultRigidBodyNum):
            self.originPositions['RigidBody'+str(i+1)] = np.zeros(3)
            self.inversedMatrix['RigidBody'+str(i+1)] = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

            self.Positions['RigidBody'+str(i+1)] = np.zeros(3)

            self.Rotations['RigidBody'+str(i+1)] = np.array([0,0,0,1])

        self.RigidBodyNum = defaultRigidBodyNum

    def GetxArmTransform(self,position: dict,rotation: dict) :
        """
        Calculate the xArm transforms
        Use relative Position & relative Rotation

        初期位置、初期回転角との差を利用する
        """

        # ----- numpy array to dict: position ----- #
        if type(position) is np.ndarray:
            position = self.NumpyArray2Dict(position)
        
        # ----- numpy array to dict: rotation ----- #
        if type(rotation) is np.ndarray:
            rotation = self.NumpyArray2Dict(rotation)

        relativePos = position['RigidBody1'] - self.originPositions['RigidBody1']
        relativeRot = self.GetRelativeRotation(rotation)

        return relativePos , relativeRot['RigidBody1']

    def GetmikataArmTransform(self,position :dict,rotation: dict) :
        """
        Calculate the mikataArm transforms
        Use  gap Position & relative Rotation

        初期位置、初期回転角との差を利用する
        """

        # ----- numpy array to dict: position ----- #
        if type(position) is np.ndarray:
            position = self.NumpyArray2Dict(position)
        
        # ----- numpy array to dict: rotation ----- #
        if type(rotation) is np.ndarray:
            rotation = self.NumpyArray2Dict(rotation)

        gapPos = position['RigidBody2'] - position['RigidBody3']
        relativeRot = self.GetRelativeRotation(rotation)

        return gapPos , relativeRot['RigidBody2']

    def SetOriginPosition(self, position) -> None:
        """
        Set the origin position

        Parameters
        ----------
        position: dict, numpy array
            Origin position
        """
        # ----- numpy array to dict: position ----- #
        if type(position) is np.ndarray:
            position = self.NumpyArray2Dict(position)
        
        #print(position)

        listRigidBody = [RigidBody for RigidBody in list(position.keys()) if 'RigidBody' in RigidBody]
        self.RigidBodyNum = len(listRigidBody)
        
        for i in range(self.RigidBodyNum):
            self.originPositions['RigidBody'+str(i+1)] = position['RigidBody'+str(i+1)]
       
    def SetInversedMatrix(self, rotation) -> None:
        """
        Set the inversed matrix

        Parameters
        ----------
        rotation: dict, numpy array
            Quaternion.
            Rotation for inverse matrix calculation
        """

        # ----- numpy array to dict: rotation ----- #
        if type(rotation) is np.ndarray:
            rotation = self.NumpyArray2Dict(rotation)
        
        listRigidBody = [RigidBody for RigidBody in list(rotation.keys()) if 'RigidBody' in RigidBody]
        self.RigidBodyNum = len(listRigidBody)

        for i in range(self.RigidBodyNum):
            q = rotation['RigidBody'+str(i+1)]
            qw, qx, qy, qz = q[3], q[1], q[2], q[0]
            mat4x4 = np.array([ [qw, -qy, qx, qz],
                                [qy, qw, -qz, qx],
                                [-qx, qz, qw, qy],
                                [-qz,-qx, -qy, qw]])
            self.inversedMatrix['RigidBody'+str(i+1)] = np.linalg.inv(mat4x4)

    def GetRelativeRotation(self, rotation):
        """
        Get the relative rotation

        Parameters
        ----------
        rotation: dict, numpy array
            Rotation to compare with the origin rotation.
            [x, y, z, w]
        
        Returns
        ----------
        relativeRot: dict
            Rotation relative to the origin rotation.
            [x, y, z, w]
        """

        # ----- numpy array to dict: rotation ----- #
        if type(rotation) is np.ndarray:
            rotation = self.NumpyArray2Dict(rotation)
        
        relativeRot = {}
        for i in range(self.RigidBodyNum):
            relativeRot['RigidBody'+str(i+1)] = np.dot(self.inversedMatrix['RigidBody'+str(i+1)], rotation['RigidBody'+str(i+1)])

        return relativeRot