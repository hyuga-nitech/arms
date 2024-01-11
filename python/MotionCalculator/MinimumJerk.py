import json as js

class MinimumJerk:
    def __init__(self) -> None:
        self.isAssistStandby = 0
        self.isAssist = 0
        self.start_pos = []
        self.start_rot = []

    def flag_check(self):
        
        
    def assist_calculate(self):
        self.flag_check()

        # ロボットの現在の位置，姿勢を直接取得する必要あり
        # フラッグの判定，アシストスタンバイ開始時のスタート位置設定にもロボットの位置・姿勢を使用する
        # 速度値などを元に，理想的な軌道を想定し，差分をアシスト量とするとともに，軌道の進行度に応じてレシオを決定
        # ターゲットの値はどうしよう？　json？

        return ratio_dict, Assist_diff_pos, Assist_diff_rot