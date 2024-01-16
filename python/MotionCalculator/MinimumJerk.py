import json as js
import numpy as np
from scipy.spatial.transform import Slerp, Rotation

from FileIO.FileIO import FileIO

class MinimumJerk:
    def __init__(self, rigidbody: list, isDemo = True, isRecord = True) -> None:
        self.isAssistStandby = False
        self.isAssist = False
        self.before_flag = False
        self.start_pos = []
        self.start_rot = []
        
        self.rigidbody_list = rigidbody

        self.target_dict = {}
        self.target = ""

        self.isDemo = isDemo
        self.isRecord = isRecord

        self.pos_list_dict = {}
        # 一旦使用していない．ロボットアームの速度だけでなく，各rigidbodyの速度が必要になったら使う
        self.arm_pos_list = []

        self.interval = 5
        self.dt = 1/120

        self.route_length

        self.predictional_time = 0.25
        
    def assist_calculate(self, flag: bool, pos: dict, rot: dict, arm_pos, arm_rot):
        self.flag_check(flag, arm_pos, arm_rot)

        assist_diff_pos = [0,0,0]
        assist_diff_rot = [0,0,0,1]

        arm_velocity = self.calculate_arm_velocity(arm_pos)

        if self.isAssistStandby:
            t = self.get_elasped_time()
            # positionの情報を元に，進捗を0~1の値で取得する

            unit_velocity = self.get_velocity_per_unit()
            # 全体の距離を１としたときに，進捗時に取りうる速度の値を取得する

            self.route_length = self.get_ideal_route_length()

            ideal_velocity = unit_velocity * self.route_length
            # 1秒でスタートからゴールまで移動できる場合に，今の進捗度でとるべき速度

            pos_at_time = self.calculate_position_at_time(t + (self.predictional_time * (arm_velocity / ideal_velocity)))
            rot_at_time = self.calculate_rotation_at_time(t + (self.predictional_time * (arm_velocity / ideal_velocity)))
            vel_at_time = self.calculate_velocity_at_time(t + (self.predictional_time * (arm_velocity / ideal_velocity)))
            # 最短軌道上で今いるべき場所 or 数秒後に居たい場所を算出
            
            assist_diff_pos = self.get_arm_diff_position(arm_pos, pos_at_time)
            assist_diff_rot = self.get_arm_diff_rotation(arm_rot, rot_at_time)
            # 理想の場所と現在の場所とのdiffを出力する

        ratio_dict = {}

        if self.isDemo:
            ratio_dict["Assist"] = [0,0]

            for rigidbody in self.rigidbody_list:
                ratio_dict[rigidbody] = [(1 / len(self.rigidbody_list)),(1 / len(self.rigidbody_list))]

        else:
            ratio_dict["Assist"] = [t,t]

            for rigidbody in self.rigidbody_list:
                ratio_dict[rigidbody] = [((1-t) / len(self.rigidbody_list)),((1-t) / len(self.rigidbody_list))]

        return assist_diff_pos, assist_diff_rot, ratio_dict
    
    def set_target(self, target_pos: dict, target_rot: dict):
        self.target_pos_dict = target_pos
        self.target_rot_dict = target_rot

    def flag_check(self, flag, arm_pos, arm_rot):
        if (flag == True)&(self.before_flag == False):
            self.isAssistStandby == True
            self.start_pos = arm_pos
            self.start_rot = arm_rot

        elif (flag == False)&(self.before_flag == True):
            self.isAssistStandby == False

        if self.isAssistStandby == True:
            self.target = self.target_decide()

    def target_decide(self):
        return "Target1"
    
    def get_ideal_route_length(self):
        ideal_vector = self.target_pos_dict[self.target] - self.start_pos
        route_length = np.linalg.norm(ideal_vector)

        return route_length
    
    def calculate_arm_velocity(self, arm_pos):
        self.arm_pos_list.append(arm_pos)

        if len(self.arm_pos_list) == self.interval:
            diff_norm = np.linalg.norm((self.arm_pos_list[self.interval - 1] - self.arm_pos_list[0]), ord=2)
            arm_v = diff_norm / (self.dt * (self.interval - 4))
            del self.arm_pos_list[0]
        else:
            arm_v = 0

        return arm_v

    def get_elasped_time(self, arm_pos):
        ideal_vector = self.target_pos_dict[self.target] - self.start_pos
        current_vector = arm_pos - self.start_pos
        
        diff_vector = current_vector - ideal_vector
        unit_normal = ideal_vector / np.linalg.norm(ideal_vector)
        projection_OC = np.dot(diff_vector, unit_normal) * unit_normal
        OC = ideal_vector + projection_OC

        t = np.linalg.norm(OC) / np.linalg.norm(ideal_vector)

        if t < 0:
            t = 0
        elif 1 < t:
            t = 1

        return t
    
    def get_velocity_per_unit(self, t):
        v = 30 * (t ** 4) - 60 * (t ** 3) + 30 * (t ** 2)

        return v
    
    def calculate_position_at_time(self, t):
        if t < 0:
            t = 0
        elif 1 < t:
            t = 1

        ideal_vector = self.target_pos_dict[self.target] - self.start_pos
        pos = self.start_pos + ((6 * (t ** 5) - 15 * (t ** 4) + 10 * (t ** 3)) * ideal_vector)
        
        return pos
    
    def calculate_rotation_at_time(self, t):
        if t < 0:
            t = 0
        elif 1 < t:
            t = 1
    
        start = self.start_rot / np.linalg.norm(self.start_rot)
        end = self.target_rot_dict[self.target] / np.linalg.norm(self.target_rot_dict[self.target])

        # Perform Slerp
        rotation = Slerp([0, 1], Rotation.from_quat([start, end]))
        interpolated_rotation = rotation([(6 * (t ** 5) - 15 * (t ** 4) + 10 * (t ** 3))])[0]

        return interpolated_rotation.as_quat()
    
    def calculate_velocity_at_time(self, t):
        if t < 0:
            t = 0
        elif 1 < t:
            t = 1

        v = 30 * (t ** 4) - 60 * (t ** 3) + 30 * (t ** 2)
        
        return (v * self.route_length)
    
    def get_arm_diff_position(self, rigidbody, relative_pos):
        diffpos = relative_pos[rigidbody] - self.before_position_dict[rigidbody]
        self.before_position_dict[rigidbody] = relative_pos[rigidbody]
        return diffpos
    
    def get_arm_diff_rotation(self, rigidbody, relative_rot):
        qw, qx, qy, qz = self.before_rotation_dict[rigidbody][3], self.before_rotation_dict[rigidbody][0], self.before_rotation_dict[rigidbody][1], self.before_rotation_dict[rigidbody][2]
        mat4x4 = np.array([ [qw, qz, -qy, qx],
                            [-qz, qw, qx, qy],
                            [qy, -qx, qw, qz],
                            [-qx,-qy, -qz, qw]])
        current_rot = relative_rot[rigidbody]
        diffrot = np.dot(np.linalg.inv(mat4x4), current_rot)