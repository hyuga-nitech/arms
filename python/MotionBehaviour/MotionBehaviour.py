# -----------------------------------------------------------------
# Author:          Hyuga Suzuki
# Original Author: Takayoshi Hagiwara (KMD)
# Created:         2022/5/20
# Summary:         RigidBody座標に基づくアーム座標の指令値計算
# -----------------------------------------------------------------

from turtle import pos
import numpy as np
import scipy.spatial.transform as scitransform
import math

class MotionBehaviour:
    originPositions         = {}
    inversedMatrix          = {}

    Positions               = {}
    Rotations               = {}

    xBeforePositions        = {}
    xWeightedPositions      = {}

    xBeforeRotations        = {}
    xWeightedRotations      = {}

    mikataBeforePositions     = {}
    mikataWeightedPositions   = {}

    def __init__(self,defaultRigidBodyNum: int = 3) -> None:
        for i in range(defaultRigidBodyNum):
            self.originPositions['RigidBody'+str(i+1)] = np.zeros(3)
            self.inversedMatrix['RigidBody'+str(i+1)] = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

            self.Positions['RigidBody'+str(i+1)] = np.zeros(3)
            self.Rotations['RigidBody'+str(i+1)] = np.array([0,0,0,1])
            
            self.xBeforePositions['RigidBody'+str(i+1)] = np.zeros(3)
            self.xWeightedPositions['RigidBody'+str(i+1)] = np.zeros(3)

            self.xBeforeRotations['RigidBody'+str(i+1)] = np.array([0,0,0,1])
            self.xWeightedRotations['RigidBody'+str(i+1)] = np.array([0,0,0,1])

            self.mikataBeforePositions['RigidBody'+str(i+1)] = np.zeros(3)
            self.mikataWeightedPositions['RigidBody'+str(i+1)] = np.zeros(3)

        self.RigidBodyNum = defaultRigidBodyNum

    def GetxArmTransform(self,position: dict,rotation: dict) :
        """
        Calculate the xArm transforms
        Use relative Position & relative Rotation

        初期位置、初期回転角との差を利用する
        """

        Pos = self.GetRelativePosition(position)
        Rot = self.GetRelativeRotation(rotation)

        relativePos = Pos['RigidBody1']
        relativeRot = self.Quaternion2Euler(Rot['RigidBody1'])

        return relativePos , relativeRot

    def GetmikataArmTransform(self,position :dict,rotation: dict) :
        """
        Calculate the mikataArm transforms
        Use  relative Position & relative Rotation

        初期位置、初期回転角との差を利用する
        """

        Pos = self.GetRelativePosition(position)

        relativePos = Pos['RigidBody2']

        return relativePos

    def GetSharedxArmTransform(self,position: dict,rotation: dict, xRatio) :
        """
        Calculate the xArm transforms
        Use relative Position & relative Rotation

        前回位置、前回回転角との差を利用する
        ※初期位置を使用してしまうと互いの剛体が引きずり合う
        """

        # ----- Shared transform ----- #
        sharedPosition = [0, 0, 0]
        sharedRotation_euler = [0, 0, 0]

        pos = self.GetRelativePosition(position)
        rot = self.GetRelativeRotation(rotation)

        for i in range(self.RigidBodyNum):
            # ----- Position ----- #
            diffPos     = pos['RigidBody'+str(i+1)] - self.xBeforePositions['RigidBody'+str(i+1)]
            weightedPos = diffPos * xRatio[i*2] + self.xWeightedPositions['RigidBody'+str(i+1)]
            sharedPosition += weightedPos

            self.xWeightedPositions['RigidBody'+str(i+1)] = weightedPos
            self.xBeforePositions['RigidBody'+str(i+1)]   = pos['RigidBody'+str(i+1)]

            # ----- Rotation ----- #
            qw, qx, qy, qz = self.xBeforeRotations['RigidBody'+str(i+1)][3], self.xBeforeRotations['RigidBody'+str(i+1)][0], self.xBeforeRotations['RigidBody'+str(i+1)][1], self.xBeforeRotations['RigidBody'+str(i+1)][2]
            mat4x4 = np.array([ [qw, qz, -qy, qx],
                                [-qz, qw, qx, qy],
                                [qy, -qx, qw, qz],
                                [-qx,-qy, -qz, qw]])
            currentRot = rot['RigidBody'+str(i+1)]
            diffRot = np.dot(np.linalg.inv(mat4x4), currentRot)
            diffRotEuler = self.Quaternion2Euler(np.array(diffRot))
            
            weightedDiffRotEuler = list(map(lambda x: x * xRatio[i*2+1] , diffRotEuler))
            weightedDiffRot = self.Euler2Quaternion(np.array(weightedDiffRotEuler))

            nqw, nqx, nqy, nqz = weightedDiffRot[3], weightedDiffRot[0], weightedDiffRot[1], weightedDiffRot[2]
            neomat4x4 = np.array([[nqw, -nqz, nqy, nqx],
                                    [nqz, nqw, -nqx, nqy],
                                    [-nqy, nqx, nqw, nqz],
                                    [-nqx,-nqy, -nqz, nqw]])
            weightedRot = np.dot(neomat4x4,  self.xWeightedRotations['RigidBody'+str(i+1)])
            sharedRotation_euler += self.Quaternion2Euler(weightedRot)

            self.xWeightedRotations['RigidBody'+str(i+1)]  = weightedRot
            self.xBeforeRotations['RigidBody'+str(i+1)]    = rot['RigidBody'+str(i+1)]
        
        return sharedPosition , sharedRotation_euler

    def GetSharedmikataArmTransform(self,position :dict,rotation: dict, mikataRatio) :
        """
        Calculate the mikataArm transforms
        Use  relative Position & relative Rotation

        前回位置、前回回転角との差を利用する
        ※初期位置を使用してしまうと互いの剛体が引きずり合う
        """

        # ----- Shared transform ----- #
        sharedPosition = [0, 0, 0]

        pos = self.GetRelativePosition(position)

        for i in range(self.RigidBodyNum):
            # ----- Position ----- #
            diffPos     = pos['RigidBody'+str(i+1)] - self.mikataBeforePositions['RigidBody'+str(i+1)]
            weightedPos = diffPos * mikataRatio[i] + self.mikataWeightedPositions['RigidBody'+str(i+1)]
            sharedPosition += weightedPos

            self.mikataWeightedPositions['RigidBody'+str(i+1)] = weightedPos
            self.mikataBeforePositions['RigidBody'+str(i+1)]   = pos['RigidBody'+str(i+1)]
        
        return sharedPosition

    def SetOriginPosition(self, position) -> None:
        """
        Set the origin position

        Parameters
        ----------
        position: dict, numpy array
            Origin position
        """
        
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

        listRigidBody = [RigidBody for RigidBody in list(rotation.keys()) if 'RigidBody' in RigidBody]
        self.RigidBodyNum = len(listRigidBody)

        for i in range(self.RigidBodyNum):
            q = rotation['RigidBody'+str(i+1)]
            qw, qx, qy, qz = q[3], q[1], q[2], q[0]
            mat4x4 = np.array([ [qw, -qy, qx, qz],
                                [qy, qw, -qz, qx],
                                [-qx, qz, qw, qy],
                                [-qz,-qx, -qy, qw]])
            self.inversedMatrix['RigidBody'+str(i+1)] = np.linalg.pinv(mat4x4)

    def GetRelativePosition(self, position):
        """
        Get the relative position

        Parameters
        ----------
        position: dict, numpy array
            Position to compare with the origin position.
            [x, y, z]
        
        Returns
        ----------
        relativePos: dict
            Position relative to the origin position.
            [x, y, z]
        """

        relativePos = {}
        for i in range(self.RigidBodyNum):
            relativePos['RigidBody'+str(i+1)] = position['RigidBody'+str(i+1)] - self.originPositions['RigidBody'+str(i+1)]
        
        return relativePos

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

        relativeRot = {}
        for i in range(self.RigidBodyNum):
            relativeRot['RigidBody'+str(i+1)] = np.dot(self.inversedMatrix['RigidBody'+str(i+1)], rotation['RigidBody'+str(i+1)])

        return relativeRot

    def Quaternion2Euler(self, q, isDeg: bool = True):
        """
        Calculate the Euler angle from the Quaternion.

        
        Rotation matrix
        |m00 m01 m02 0|
        |m10 m11 m12 0|
        |m20 m21 m22 0|
        | 0   0   0  1|

        Parameters
        ----------
        q: np.ndarray
            Quaternion.
            [x, y, z, w]
        isDeg: (Optional) bool
            Returned angles are in degrees if this flag is True, else they are in radians.
            The default is True.
        
        Returns
        ----------
        rotEuler: np.ndarray
            Euler angle.
            [x, y, z]
        """
        
        qx = q[0]
        qy = q[1]
        qz = q[2]
        qw = q[3]

        # 1 - 2y^2 - 2z^2
        m00 = 1 - (2 * qy**2) - (2 * qz**2)
        # 2xy + 2wz
        m01 = (2 * qx * qy) + (2 * qw * qz)
        # 2xz - 2wy
        m02 = (2 * qx * qz) - (2 * qw * qy)
        # 2xy - 2wz
        m10 = (2 * qx * qy) - (2 * qw * qz)
        # 1 - 2x^2 - 2z^2
        m11 = 1 - (2 * qx**2) - (2 * qz**2)
        # 2yz + 2wx
        m12 = (2 * qy * qz) + (2 * qw * qx)
        # 2xz + 2wy
        m20 = (2 * qx * qz) + (2 * qw * qy)
        # 2yz - 2wx
        m21 = (2 * qy * qz) - (2 * qw * qx)
        # 1 - 2x^2 - 2y^2
        m22 = 1 - (2 * qx**2) - (2 * qy**2)

        # 回転軸の順番がX->Y->Zの固定角(Rz*Ry*Rx)
        # if m01 == -1:
        # 	tx = 0
        # 	ty = math.pi/2
        # 	tz = math.atan2(m20, m10)
        # elif m20 == 1:
        # 	tx = 0
        # 	ty = -math.pi/2
        # 	tz = math.atan2(m20, m10)
        # else:
        # 	tx = -math.atan2(m02, m00)
        # 	ty = -math.asin(-m01)
        # 	tz = -math.atan2(m21, m11)

        # 回転軸の順番がX->Y->Zのオイラー角(Rx*Ry*Rz)
        if m02 == 1:
            tx = math.atan2(m10, m11)
            ty = math.pi/2
            tz = 0
        elif m02 == -1:
            tx = math.atan2(m21, m20)
            ty = -math.pi/2
            tz = 0
        else:
            tx = -math.atan2(-m12, m22)
            ty = -math.asin(m02)
            tz = -math.atan2(-m01, m00)

        if isDeg:
            tx = np.rad2deg(tx)
            ty = np.rad2deg(ty)
            tz = np.rad2deg(tz)

        rotEuler = np.array([tx, ty, tz])
        return rotEuler

    def Euler2Quaternion(self, e):
        """
        Calculate the Quaternion from the Euler angle.

        Parameters
        ----------
        e: np.ndarray
            Euler.
            [x, y, z]
        
        Returns
        ----------
        rotQuat: np.ndarray
            Quaternion
            [x, y, z, w]
        """

        roll = np.deg2rad(e[0])
        pitch = np.deg2rad(e[1])
        yaw = np.deg2rad(e[2])

        cosRoll = np.cos(roll/2.0)
        sinRoll = np.sin(roll / 2.0)
        cosPitch = np.cos(pitch / 2.0)
        sinPitch = np.sin(pitch / 2.0)
        cosYaw = np.cos(yaw / 2.0)
        sinYaw = np.sin(yaw / 2.0)

        q0 = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw
        q1 = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw
        q2 = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw
        q3 = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw

        rotQuat = [q1, q2, q3, q0]
        return rotQuat
