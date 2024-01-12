import json as js

class MinimumJerk:
    def __init__(self, rigidbody: list) -> None:
        self.isAssistStandby = 0
        self.isAssist = 0
        self.start_pos = []
        self.start_rot = []
        
        self.rigidbody_list = rigidbody

    def flag_check(self):
        pass
        
    def assist_calculate(self):
        self.flag_check()

        assist_diff_pos = [0,0,0]
        assist_diff_rot = [0,0,0,1]

        ratio_dict = {}

        ratio_dict["Assist"] = [0,0]

        for rigidbody in self.rigidbody_list:
            ratio_dict[rigidbody] = [0.5,0.5]

        # ロボットの現在の位置，姿勢を直接取得する必要あり
        # フラッグの判定，アシストスタンバイ開始時のスタート位置設定にもロボットの位置・姿勢を使用する
        # 速度値などを元に，理想的な軌道を想定し，差分をアシスト量とするとともに，軌道の進行度に応じてレシオを決定
        # ターゲットの値はどうしよう？　json？

        return assist_diff_pos, assist_diff_rot, ratio_dict