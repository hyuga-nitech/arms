# -----------------------------------------------------------------
# Author:          Hyuga Suzuki
# Created:         2023/8/1
# Summary:         ratioの値を取得（Bluetooth経由シリアル通信）
# -----------------------------------------------------------------

import serial

class LEDdirectionManager:
    def __init__(self, port):
        try:
            self.LED_serial = serial.Serial(port, 9600, timeout=0.1)
            self.gain = 1

        except:
            print('Failed to connect to LED')

    def send(self, position):
        while True:
            try:
                posRigidBody1 = position['RigidBody1'] * 1000
                posRigidBody2 = position['RigidBody2'] * 1000

                xArmy = posRigidBody1[0]
                xArmz = posRigidBody1[1]
                mArmy = posRigidBody2[0]
                mArmz = posRigidBody2[1]

                message = str(int(xArmy * self.gain)) + ", " + str(int(xArmz * self.gain)) + ", " + str(int(mArmy * self.gain)) + ", " + str(int(mArmz * self.gain))
                serial.write(message.encode('utf-8'))

            except:
                print('Failed to send to LED')


# posRigidBody1 = position['RigidBody1'] * 1000
# posRigidBody2 = position['RigidBody2'] * 1000
# rotRigidBody1 = np.rad2deg(self.Behaviour.Quaternion2Euler(rotation['RigidBody1']))
# rotRigidBody2 = np.rad2deg(self.Behaviour.Quaternion2Euler(rotation['RigidBody2']))

# self.get_pos_1_box.append(np.concatenate([posRigidBody1, rotRigidBody1], 0))
# get_pos_1_filt = self.filter_FB.lowpass2(self.get_pos_1_box, self.get_pos_1_filt_box)
# self.get_pos_1_filt_box.append(get_pos_1_filt)
# del self.get_pos_1_box[0]
# del self.get_pos_1_filt_box[0]

# self.get_pos_2_box.append(np.concatenate([posRigidBody2, rotRigidBody2]))
# get_pos_2_filt = self.filter_FB.lowpass2(self.get_pos_2_box, self.get_pos_2_filt_box)
# self.get_pos_2_filt_box.append(get_pos_2_filt)
# del self.get_pos_2_box[0]
# del self.get_pos_2_filt_box[0]

# self.listRigidBodyPos1.append(get_pos_1_filt[0:3])
# self.listRigidBodyPos2.append(get_pos_2_filt[0:3])

# if len(self.listRigidBodyPos1)== 2:
#     listvelPosP1 = (np.diff(self.listRigidBodyPos1, n=1, axis=0)/self.dt)
#     listvelPosP2 = (np.diff(self.listRigidBodyPos2, n=1, axis=0)/self.dt)

#     vel_gain1 = 10
#     vel_gain2 = 10

#     #FB1:左、FB2:右、FB3:前、FB4:後
    
#     if listvelPosP1[0][0] >= 0:
#         fb_vel_1 = listvelPosP1[0][0] * vel_gain1
#         fb_vel_2 = 0
#     else:
#         fb_vel_1 = 0
#         fb_vel_2 = -1 * listvelPosP1[0][0] * vel_gain1

#     if listvelPosP1[0][2] >= 0:
#         fb_vel_3 = listvelPosP1[0][2] * vel_gain1
#         fb_vel_4 = 0
#     else:
#         fb_vel_3 = 0
#         fb_vel_4 = -1 * listvelPosP1[0][2] * vel_gain1

#     #FB5:左、FB6:右、FB7:前、FB8:後
    
#     if listvelPosP2[0][0] >= 0:
#         fb_vel_5 = listvelPosP2[0][0] * vel_gain2
#         fb_vel_6 = 0
#     else:
#         fb_vel_5 = 0
#         fb_vel_6 = -1 * listvelPosP2[0][0] * vel_gain2

#     if listvelPosP2[0][2] >= 0:
#         fb_vel_7 = listvelPosP2[0][2] * vel_gain2
#         fb_vel_8 = 0
#     else:
#         fb_vel_7 = 0
#         fb_vel_8 = -1 * listvelPosP2[0][2] * vel_gain2

#     #FeedBack about xArm
#     self.data_out_1 = fb_vel_1 * xratio[0]
#     self.data_out_2 = fb_vel_2 * xratio[0]
#     self.data_out_3 = fb_vel_3 * xratio[0]
#     self.data_out_4 = fb_vel_4 * xratio[0]
#     self.data_out_5 = fb_vel_1 * mikataratio[0]
#     self.data_out_6 = fb_vel_2 * mikataratio[0]
#     self.data_out_7 = fb_vel_3 * mikataratio[0]
#     self.data_out_8 = fb_vel_4 * mikataratio[0]

#     #FeedBack about mikata
#     self.data_out_9 = fb_vel_5 * xratio[2]
#     self.data_out_10 = fb_vel_6 * xratio[2]
#     self.data_out_11 = fb_vel_7 * xratio[2]
#     self.data_out_12 = fb_vel_8 * xratio[2]
#     self.data_out_13 = fb_vel_5 * mikataratio[1]
#     self.data_out_14 = fb_vel_6 * mikataratio[1]
#     self.data_out_15 = fb_vel_7 * mikataratio[1]
#     self.data_out_16 = fb_vel_8 * mikataratio[1]

#     del self.listRigidBodyPos1[0]
#     del self.listRigidBodyPos2[0]