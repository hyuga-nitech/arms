import math

import numpy as np


def quaternion_to_euler(quaternion, isDeg: bool = True):
    qx, qy, qz, qw = quaternion[0], quaternion[1], quaternion[2], quaternion[3]

    m00 = 1 - (2 * qy**2) - (2 * qz**2)
    m01 = (2 * qx * qy) + (2 * qw * qz)
    m02 = (2 * qx * qz) - (2 * qw * qy)
    m10 = (2 * qx * qy) - (2 * qw * qz)
    m11 = 1 - (2 * qx**2) - (2 * qz**2)
    m12 = (2 * qy * qz) + (2 * qw * qx)
    m20 = (2 * qx * qz) + (2 * qw * qy)
    m21 = (2 * qy * qz) - (2 * qw * qx)
    m22 = 1 - (2 * qx**2) - (2 * qy**2)

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

def euler_to_quaternion(euler):
    roll = np.deg2rad(euler[0])
    pitch = np.deg2rad(euler[1])
    yaw = np.deg2rad(euler[2])

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

def Slerp_Quaternion(Quaternion, initQuaternion, weight):
    if weight == 1:
        return Quaternion

    elif weight == 0:
        return initQuaternion

    else:
        e = 10e-30
        dot = np.dot(initQuaternion, Quaternion)
        if dot > 1:
            dot = 1
        elif dot < -1:
            dot = -1
        theta = math.acos(dot)
        return (math.sin((1 - weight) * theta)/ (math.sin(theta) + e)) * np.array(initQuaternion) + (math.sin(weight * theta)/ (math.sin(theta) + e)) * np.array(Quaternion)

def ConvertAxis_Position(position, axis):
    if axis == 'vertical':
        position = [position[2], position[0], position[1]]
    elif axis == 'left':
        position = [position[2], -1 * position[1], position[0]]
    elif axis == 'right':
        position = [position[2], position[1], -1 * position[0]]

    return position

def CnvertAxis_Rotation(rotation, axis):
    if axis == 'vertical':
        rotation = [rotation[2], rotation[0], rotation[1], rotation[3]]
    elif axis == 'left':
        rotation = [rotation[2], -1 * rotation[1], rotation[0], rotation[3]]
    elif axis == 'right':
        rotation = [rotation[2], rotation[1], -1 * rotation[0], rotation[3]]

    return rotation

def convert_to_matrix(quaternion):
    qw, qx, qy, qz = quaternion[3], quaternion[1], quaternion[2], quaternion[0]
    matrix = np.array([ [qw, -qy, qx, qz],
                        [qy, qw, -qz, qx],
                        [-qx, qz, qw, qy],
                        [-qz,-qx, -qy, qw]])

    return matrix

def Convert2InverseMatrix(quaternion):
    qw, qx, qy, qz = quaternion[3], quaternion[1], quaternion[2], quaternion[0]
    matrix = np.array([ [qw, -qy, qx, qz],
                        [qy, qw, -qz, qx],
                        [-qx, qz, qw, qy],
                        [-qz,-qx, -qy, qw]])
    matrix = np.linalg.inv(matrix)

    return matrix

def ConvertSensorToGripper(sensorValue, InputMax = 1, InputMin = 0, TargetMax = 850, TargetMin = 0):
    gripperValue = ((sensorValue - InputMin) / (InputMax - InputMin)) * (TargetMax - TargetMin) + TargetMin

    if gripperValue > TargetMax:
        gripperValue = TargetMax
    elif gripperValue < TargetMin:
        gripperValue = TargetMin

    return gripperValue

def solve_nploy(vec,is_complex=False):
    dim =len(vec)
    if is_complex:
        A = np.zeros((dim,dim),dtype=complex)
    else:
        A = np.zeros((dim,dim))
    A[np.arange(dim-1),1+np.arange(dim-1)] =1
    A[-1,:] = -vec
    ans,vec = np.linalg.eig(A)
    return ans
