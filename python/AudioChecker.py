from VibrotactileFeedback.VibrotactileFeedbackManager import VibrotactileFeedbackManager
import pyaudio
import numpy as np

mode = 'A'

if __name__ == '__main__':
    vibrotactileFeedbackManager = VibrotactileFeedbackManager()
    vibrotactileFeedbackManager.forAudioCheck(mode)
    print('\n--- End Audio Check ---')