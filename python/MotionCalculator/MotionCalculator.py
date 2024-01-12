import numpy as np
import math
import json as js

from MotionCalculator.MinimumJerk import MinimumJerk

class MotionCalculator:
    origin_position_dict = {}
    inversed_matrix_dict = {}

    position_dict = {}
    rotation_dict = {}

    before_position_dict = {}
    before_rotation_dict = {}

    weighted_position_dict = {}
    weighted_rotation_dict = {}

    rigidbody_list_dict = {}

    def __init__(self,operatorNum: int = 2) -> None:
        bending_f = open("bending_sensor_setting.json","r")
        self.bending_sensor_js = js.load(bending_f)
        xArm_setting_f = open("xArm_setting.json","r")
        self.xArm_js = js.load(xArm_setting_f)
        rigidbody_f = open("rigidbody_setting.json","r")
        self.rigidbody_js = js.load(rigidbody_f)

        self.minimumJerk_dict = {}

        # ここから下は要書き換え

        for arm in self.xArm_js["xArmConfig"]:
            self.rigidbody_list_dict[arm] = []
            for rigidbody in self.rigidbody_js["RigidBodyConfig"]:
                if self.rigidbody_js["RigidBodyConfig"][rigidbody]["Arm"] == arm:
                    self.rigidbody_list_dict[arm].append(rigidbody)
            self.minimumJerk_dict[arm] = MinimumJerk(self.rigidbody_list_dict[arm])

        for rigidbody in self.rigidbody_js["RigidBodyConfig"]:
            self.origin_position_dict[rigidbody] = np.zeros(3)
            self.inversed_matrix_dict[rigidbody] = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

            self.position_dict[rigidbody] = np.zeros(3)
            self.rotation_dict[rigidbody] = np.array([0,0,0,1])

            self.before_position_dict[rigidbody] = np.zeros(3)
            self.before_rotation_dict[rigidbody] = np.array([0,0,0,1])

            self.weighted_position_dict[rigidbody] = np.zeros(3)
            self.weighted_rotation_dict[rigidbody] = np.array([0,0,0,1])

        self.OperatorNum = operatorNum

    def calculate_shared_pos_rot(self, position: dict, rotation: dict, arm_position: dict):
        # ----- Shared transform ----- #
        shared_pos_dict = {}
        shared_rot_dict = {}

        relative_pos = self.get_relative_position(position)
        relative_rot = self.get_relative_rotation(rotation)

        for arm in self.xArm_js["xArmConfig"]:
            diffpos_dict = {}
            diffrot_dict = {}

            diffpos_dict["Assist"], diffrot_dict["Assist"], ratio_dict = self.minimumJerk_dict[arm].assist_calculate(relative_pos, relative_rot, arm_position)

            for rigidbody in self.rigidbody_list_dict[arm]:
                diffpos_dict[rigidbody] = self.get_diff_position(rigidbody, relative_pos)
                diffrot_dict[rigidbody] = self.get_diff_rotation(rigidbody, relative_rot)
            
            shared_pos_dict[arm] = self.calculate_ratio_pos(diffpos_dict, ratio_dict)
            shared_rot_dict[arm] = self.calculate_ratio_rot(diffrot_dict, ratio_dict)

        return shared_pos_dict, shared_rot_dict, ratio_dict

    def calculate_ratio_pos(self, diffpos: dict, ratio: dict):
        ratio_pos = [0, 0, 0]
        for operator in diffpos.keys():
            weighted_pos = diffpos[operator] * ratio[operator][0]
            ratio_pos += weighted_pos
            self.weighted_rotation_dict[operator] = weighted_pos

        return ratio_pos
    
    def calculate_ratio_rot(self, diffrot: dict, ratio: dict):
        ratio_rot_euler = [0, 0, 0]
        for operator in diffrot.keys():
            diffrot_euler = self.quaternion_to_euler(np.array(diffrot[operator]))
            weighted__diff_rot_euler = list(map(lambda x: x * ratio[operator][1] , diffrot_euler))
            weighted_diff_rot = self.euler_to_quaternion(np.array(weighted__diff_rot_euler))

            nqw, nqx, nqy, nqz = weighted_diff_rot[3], weighted_diff_rot[0], weighted_diff_rot[1], weighted_diff_rot[2]
            neomat4x4 = np.array([[nqw, -nqz, nqy, nqx],
                                  [nqz, nqw, -nqx, nqy],
                                  [-nqy, nqx, nqw, nqz],
                                  [-nqx,-nqy, -nqz, nqw]])
            
            weighted_rot = np.dot(neomat4x4, self.weighted_rotation_dict[operator])
            ratio_rot_euler += self.quaternion_to_euler(weighted_rot)

            self.weighted_rotation_dict[operator] = weighted_rot

        return ratio_rot_euler
    
    def calculate_shared_grip(self, bending_sensor_value: dict):
        gripper_value_dict = {}

        for arm in self.xArm_js["xArmConfig"]:
            gripper_value_dict[arm] = 0
            count = 0
            for sensor in self.bending_sensor_js["BendingSensorConfig"]:
                if self.bending_sensor_js["BendingSensorConfig"][sensor]["Arm"] == arm:
                    count += 1
                    gripper_value_dict[arm] += bending_sensor_value[sensor]
                
                if count > 1:
                    gripper_value_dict[arm] = gripper_value_dict[arm] / count
                else:
                    pass

        return gripper_value_dict
    
    def calculate_ratio(self, position: dict, rotation: dict, ratio: dict):
        sharedPosition = [0, 0, 0]
        sharedRotation_euler = [0, 0, 0]

        return sharedPosition, sharedRotation_euler

    def set_origin_position(self, position) -> None:
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
            self.origin_position_dict['RigidBody'+str(i+1)] = position['RigidBody'+str(i+1)]
       
    def set_inversed_matrix(self, rotation) -> None:
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
            self.inversed_matrix_dict['RigidBody'+str(i+1)] = np.linalg.pinv(mat4x4)

    def get_relative_position(self, position):
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
            relativePos['RigidBody'+str(i+1)] = position['RigidBody'+str(i+1)] - self.origin_position_dict['RigidBody'+str(i+1)]
        
        return relativePos

    def get_relative_rotation(self, rotation):
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
    
    def get_diff_position(self, rigidbody, relative_pos):
        diffpos = relative_pos[rigidbody] - self.before_position_dict[rigidbody]
        self.before_position_dict[rigidbody] = relative_pos[rigidbody]
        return diffpos
    
    def get_diff_rotation(self, rigidbody, relative_rot):
        qw, qx, qy, qz = self.before_rotation_dict[rigidbody][3], self.before_rotation_dict[rigidbody][0], self.before_rotation_dict[rigidbody][1], self.before_rotation_dict[rigidbody][2]
        mat4x4 = np.array([ [qw, qz, -qy, qx],
                            [-qz, qw, qx, qy],
                            [qy, -qx, qw, qz],
                            [-qx,-qy, -qz, qw]])
        current_rot = relative_rot[rigidbody]
        diffrot = np.dot(np.linalg.inv(mat4x4), current_rot)

        self.before_rotation_dict[rigidbody] = relative_rot[rigidbody]

        return diffrot

    def quaternion_to_euler(self, q, isDeg: bool = True):
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

    def euler_to_quaternion(self, e):
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
