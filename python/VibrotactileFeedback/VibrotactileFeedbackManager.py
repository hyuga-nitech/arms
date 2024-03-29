# -----------------------------------------------------------------
# Author:          Hyuga Suzuki
# Original Author: Takumi Katagiri, Takumi Nishimura (Nagoya Institute of Technology)
# Created:         2022/6/13
# Summary:         振動フィードバック制御マネージャー
# -----------------------------------------------------------------

import pyaudio
import numpy as np
import math

from MotionFilter.MotionFilter import MotionFilter
from VibrotactileFeedback.AudioDeviceIndexes import AudioDeviceIndexes
from MotionBehaviour.MotionBehaviour import MotionBehaviour

class VibrotactileFeedbackManager:
    def __init__(self):
        # ----- Find audio device indexes ----- #
        audioDeviceIndexes = AudioDeviceIndexes()
        ListIndexNum = audioDeviceIndexes.Find(host_api='Windows DirectSound', name='Sound Blaster Play! 3')
        OutputDeviceNum = len(ListIndexNum)
        print(ListIndexNum)

        self.Behaviour = MotionBehaviour()
    
        self.p = pyaudio.PyAudio()
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 44100
        self.amp = 30

        self.freq = 75
        self.CHUNK = int(self.rate / self.freq)
        self.sin = np.sin(2.0 * np.pi * np.arange(self.CHUNK) * float(self.freq) / float(self.rate))

        # ----- Initialize the parameter of data_out according as OutputDeviceNum ----- #
        for i in range(OutputDeviceNum):    # note: 2 is for stereo
            data_out_command = 'self.data_out_' + str(i+1) + '= 0.0'
            exec(data_out_command)

        # ----- Define streamming command according as OutputDeviceNum ----- #
        for i in range(OutputDeviceNum):
            stream_command = 'self.stream' + str(i+1) + '= self.p.open('\
                + 'rate = self.rate,'\
                + 'channels = self.channels,'\
                + 'format = self.format,'\
                + 'output = True,'\
                + 'output_device_index = ListIndexNum[' + str(i) + '],'\
                + 'frames_per_buffer = self.CHUNK,'\
                + 'stream_callback = self.callback' + str(i+1)\
                + ')'
            exec(stream_command)
            
            start_streamming_command = 'self.stream' + str(i+1) + '.start_stream()'
            exec(start_streamming_command)

        n = 2
        fp = 10
        fs = 180
        self.filter_FB = MotionFilter()
        self.filter_FB.InitLowPassFilterWithOrder(fs, fp, n)
        self.dt = round(1/fs, 4)

        self.initGripperPosition = np.array([0, 0, -200])

        self.get_pos_1_box = [[0, 0, 0, 0, 0, 0]] * n
        self.get_pos_2_box = [[0, 0, 0, 0, 0, 0]] * n
        self.get_pos_1_filt_box = [[0, 0, 0, 0, 0, 0]] * n
        self.get_pos_2_filt_box = [[0, 0, 0, 0, 0, 0]] * n

        self.beforelpf_vel    = [0] * n
        self.afterlpf_vel     = [0] * n

        self.listRigidBodyPos1 = []
        self.listRigidBodyPos2 = []

        self.listRigidBodyRot1 = []
        self.listRigidBodyRot2 = []

    def callback1(self, in_data, frame_count, time_info, status):
        out_data_1 = ((503.0463*math.exp((self.data_out_1*self.amp*0.00008)/0.3752)-500) * self.sin).astype(np.int16)
        return (out_data_1, pyaudio.paContinue)


    def callback2(self, in_data, frame_count, time_info, status):
        out_data_2 = ((503.0463*math.exp((self.data_out_2*self.amp*0.00008)/0.3752)-500) * self.sin).astype(np.int16)
        return (out_data_2, pyaudio.paContinue)


    def callback3(self, in_data, frame_count, time_info, status):
        out_data_3 = ((503.0463*math.exp((self.data_out_3*self.amp*0.00008)/0.3752)-500) * self.sin).astype(np.int16)
        return (out_data_3, pyaudio.paContinue)


    def callback4(self, in_data, frame_count, time_info, status):
        out_data_4 = ((503.0463*math.exp((self.data_out_4*self.amp*0.00008)/0.3752)-500) * self.sin).astype(np.int16)
        return (out_data_4, pyaudio.paContinue)

    def close(self):
        self.p.terminate()

    def FBArms(self, position: dict, rotation: dict, xratio, mikataratio):
        posRigidBody1 = position['RigidBody1'] * 1000
        posRigidBody2 = position['RigidBody2'] * 1000
        rotRigidBody1 = np.rad2deg(self.Behaviour.Quaternion2Euler(rotation['RigidBody1']))
        rotRigidBody2 = np.rad2deg(self.Behaviour.Quaternion2Euler(rotation['RigidBody2']))

        self.get_pos_1_box.append(np.concatenate([posRigidBody1, rotRigidBody1], 0))
        get_pos_1_filt = self.filter_FB.lowpass2(self.get_pos_1_box, self.get_pos_1_filt_box)
        self.get_pos_1_filt_box.append(get_pos_1_filt)
        del self.get_pos_1_box[0]
        del self.get_pos_1_filt_box[0]

        self.get_pos_2_box.append(np.concatenate([posRigidBody2, rotRigidBody2]))
        get_pos_2_filt = self.filter_FB.lowpass2(self.get_pos_2_box, self.get_pos_2_filt_box)
        self.get_pos_2_filt_box.append(get_pos_2_filt)
        del self.get_pos_2_box[0]
        del self.get_pos_2_filt_box[0]

        self.listRigidBodyPos1.append(get_pos_1_filt[0:3])
        self.listRigidBodyPos2.append(get_pos_2_filt[0:3])

        self.listRigidBodyRot1.append(get_pos_1_filt[3:7])
        self.listRigidBodyRot2.append(get_pos_2_filt[3:7])

        if len(self.listRigidBodyPos1) == 2:
            velPosP1 = np.linalg.norm((np.diff(self.listRigidBodyPos1, n=1, axis=0)/self.dt))
            velPosP2 = np.linalg.norm((np.diff(self.listRigidBodyPos2, n=1, axis=0)/self.dt))

            velRotP1 = np.linalg.norm((np.diff(self.listRigidBodyRot1, n=1, axis=0)/self.dt))
            velRotP2 = np.linalg.norm((np.diff(self.listRigidBodyRot2, n=1, axis=0)/self.dt))

            p_r_gain = 0
            vel_gain = 1.0
            fb_vel_1 = (velPosP1+velRotP1*p_r_gain)*vel_gain
            fb_vel_2 = (velPosP2+velRotP2*p_r_gain)*vel_gain

            self.data_out_1 = fb_vel_1 * xratio[0] + fb_vel_2 * xratio[2]
            self.data_out_2 = fb_vel_1 * xratio[0] + fb_vel_2 * xratio[2]
            self.data_out_3 = fb_vel_1 * mikataratio[0] + fb_vel_2 * mikataratio[1]
            self.data_out_4 = fb_vel_1 * mikataratio[0] + fb_vel_2 * mikataratio[1]

            del self.listRigidBodyPos1[0]
            del self.listRigidBodyPos2[0]

            del self.listRigidBodyRot1[0]
            del self.listRigidBodyRot2[0]

    def FBEachOther(self,position: dict, rotation: dict, xratio, mikataratio):
        posRigidBody1 = position['RigidBody1'] * 1000
        posRigidBody2 = position['RigidBody2'] * 1000
        rotRigidBody1 = np.rad2deg(self.Behaviour.Quaternion2Euler(rotation['RigidBody1']))
        rotRigidBody2 = np.rad2deg(self.Behaviour.Quaternion2Euler(rotation['RigidBody2']))

        self.get_pos_1_box.append(np.concatenate([posRigidBody1, rotRigidBody1], 0))
        get_pos_1_filt = self.filter_FB.lowpass2(self.get_pos_1_box, self.get_pos_1_filt_box)
        self.get_pos_1_filt_box.append(get_pos_1_filt)
        del self.get_pos_1_box[0]
        del self.get_pos_1_filt_box[0]

        self.get_pos_2_box.append(np.concatenate([posRigidBody2, rotRigidBody2]))
        get_pos_2_filt = self.filter_FB.lowpass2(self.get_pos_2_box, self.get_pos_2_filt_box)
        self.get_pos_2_filt_box.append(get_pos_2_filt)
        del self.get_pos_2_box[0]
        del self.get_pos_2_filt_box[0]

        self.listRigidBodyPos1.append(get_pos_1_filt[0:3])
        self.listRigidBodyPos2.append(get_pos_2_filt[0:3])

        self.listRigidBodyRot1.append(get_pos_1_filt[3:7])
        self.listRigidBodyRot2.append(get_pos_2_filt[3:7])

        if len(self.listRigidBodyPos1) == 2:
            velPosP1 = np.linalg.norm((np.diff(self.listRigidBodyPos1, n=1, axis=0)/self.dt))
            velPosP2 = np.linalg.norm((np.diff(self.listRigidBodyPos2, n=1, axis=0)/self.dt))

            velRotP1 = np.linalg.norm((np.diff(self.listRigidBodyRot1, n=1, axis=0)/self.dt))
            velRotP2 = np.linalg.norm((np.diff(self.listRigidBodyRot2, n=1, axis=0)/self.dt))

            p_r_gain = 0
            vel_gain = 1.0
            fb_vel_1 = (velPosP1+velRotP1*p_r_gain)*vel_gain
            fb_vel_2 = (velPosP2+velRotP2*p_r_gain)*vel_gain

            self.data_out_1 = fb_vel_2 * xratio[2]
            self.data_out_2 = fb_vel_1 * xratio[0]
            self.data_out_3 = fb_vel_2 * mikataratio[1]
            self.data_out_4 = fb_vel_1 * mikataratio[0]

            del self.listRigidBodyPos1[0]
            del self.listRigidBodyPos2[0]

            del self.listRigidBodyRot1[0]
            del self.listRigidBodyRot2[0]