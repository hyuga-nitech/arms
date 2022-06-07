# -----------------------------------------------------------------------
# Author:   Takayoshi Hagiwara (KMD)
# Created:  2021/10/19
# Summary:  オーディオI/Oマネージャー
#
# Issues:
#   >pip install simpleaudio に失敗する
#       -> https://visualstudio.microsoft.com/ja/downloads/ からBuild Tools for Visual Studio 2019をダウンロードし，
#       　「C++ Build Tools」もしくは「C++によるデスクトップ開発」をインストールする
# -----------------------------------------------------------------------

import simpleaudio
import pyaudio
import numpy as np
from scipy import signal
import threading

class AudioManager:
    def __init__(self, odeviceIdx: int = 10) -> None:
        self.audioPositive = simpleaudio.WaveObject.from_wave_file('Audio/Resources/10hz_hanshuki_positive.wav')
        self.audioNegative = simpleaudio.WaveObject.from_wave_file('Audio/Resources/10hz_hanshuki_negative.wav')

        # ----- Output audio parameters ----- #
        self.amp = 300000
        self.freq = 10


        # ----- Audio stream parameters for callback ----- #
        self.cbPyAudioFormat      = pyaudio.paInt16
        self.cbChannels           = 1
        self.cbRate               = 44100
        self.cbChunk              = int(self.cbRate / self.freq)
        self.cbOutputDeviceIdx    = odeviceIdx

        self.pyAudioObj     = pyaudio.PyAudio()
        self.callbackRawAnalogStream    = self.pyAudioObj.open( format              = self.cbPyAudioFormat,
                                                                channels            = self.cbChannels,
                                                                rate                = self.cbRate,
                                                                frames_per_buffer   = self.cbChunk,
                                                                output              = True,
                                                                output_device_index = self.cbOutputDeviceIdx,
                                                                stream_callback     = self.CallbackRawAnalog)


        # ----- Audio stream parameters for tactile feedback ----- #
        self.tactPyAudioFormat      = pyaudio.paFloat32
        self.tactChannels           = 1
        self.tactRate               = 44100
        #self.tactChunk              = int(self.cbRate / self.freq)
        self.tactChunk              = 10
        self.tactOutputDeviceIdx    = odeviceIdx

        self.audioStream  = self.pyAudioObj.open( format              = self.tactPyAudioFormat,
                                                  channels            = self.tactChannels,
                                                  rate                = self.tactRate,
                                                  frames_per_buffer   = self.tactChunk,
                                                  output              = True,
                                                  output_device_index = self.tactOutputDeviceIdx)
        
        # ----- Create signals ----- #
        self.sinWave    = np.sin(2.0 * np.pi * np.arange(self.cbChunk) * float(self.freq) / float(self.cbRate)) + 1/2
        self.squareWave = signal.square(2.0 * np.pi *  np.arange(self.cbChunk) * float(self.freq) / float(self.cbRate))

        # ----- Raw data of the load cell ----- #
        self.rawAnalogValue     = 0
        self.rawAnalogValueList = [0] * self.cbChunk

        # ----- Start stream ----- #
        #self.callbackRawAnalogStream.start_stream()

        # ----- Close the stream when the program is terminated ----- #
        import atexit
        atexit.register(self.CloseStream)
        
    
    def CloseStream(self):
        self.callbackRawAnalogStream.close()
        self.audioStream.close()
    
    def PlayPositive(self):
        playPositive = self.audioPositive.play()
        playPositive.wait_done()
    
    def PlayNegative(self):
        playNegative = self.audioNegative.play()
        playNegative.wait_done()
    
    def PlayFromFile(self, fileName):
        """
        Play audio file.

        Parameters
        ----------
        fileName: str
            Audio file name.
            Include .wav
        """
        
        audioObject = simpleaudio.WaveObject.from_wave_file(fileName)
        playAudioObject = audioObject.play()
        playAudioObject.wait_done()
    
    def PlaySinWave(self):
        self.audioStream.write(self.sinWave.astype(np.float32).tostring())
    
    def PlaySquareWave(self):
        self.audioStream.write(self.squareWave.astype(np.float32).tostring())
    
    def PlayRawAnalog(self, data):
        self.audioStream.write(data.astype(np.float32).tostring())
    

    def CallbackRawAnalog(self, in_data, frame_count, time_info, status):
        #output = (self.rawAnalogValue * float(self.amp) * (self.sinWave)).astype(np.int16)
        output = (self.rawAnalogValue * float(self.amp) * (self.squareWave)).astype(np.int16)
        
        # sin込みの場合と生データのみの場合でグラフを描画

        return (output, pyaudio.paContinue)


    def AddRawAnalogValue(self, analogValue):
        self.rawAnalogValue = analogValue

    def AddRawAnalogValueList(self, analogValue):
        self.rawAnalogValueList = np.append(self.rawAnalogValueList, analogValue)
        
        if len(self.rawAnalogValueList) > self.cbChunk:
            self.rawAnalogValueList = np.delete(self.rawAnalogValueList, 0)
    