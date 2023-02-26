# arms/Python
<img src="https://img.shields.io/badge/Python-3.10.4-blue?&logo=Python">

xArmとmikataArmを接続し、多自由度化を目指すプロジェクトのリポジトリです．
複数人の操作者の手の動きを２つのアームに分散して反映させ、操作を行います．

# Requirement

- Python            3.10.4
- numpy             1.22.4
- pyaudio           0.2.11
- pyserial          3.5
- tqdm              4.64.0
- dynamixel-sdk     3.7.51
- xArm-Python-SDK   1.9.0

# FixMe

- 操作者数と監視する剛体数を別々に定義できるように改善中です．モーションキャプチャ，データ記録はRigidBodyNum，アーム動作の形成にはOperatorNumを使用します．
- ファイル名は実験条件をもとに自動で決定するシステムを使用しています．ただし，同一の名前で複数回実験を行うと上書きされてしまいます．
- ユーザーが決定できる設定値と，ユーザーが変更する必要のない設定値が混在しています．特に，設定が重複している部分，参照する設定値を決定する設定値があるため，改善が必要です．
- （RobotControlを除き）１モジュールに対し１クラス，１役割を徹底したいです．
