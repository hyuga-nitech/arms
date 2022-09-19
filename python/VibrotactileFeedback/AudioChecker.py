from VibrotactileFeedback.VibrotactileFeedbackManager import VibrotactileFeedbackManager
import pyaudio
import numpy as np

if __name__ == '__main__':
    vibrotactileFeedbackManager = VibrotactileFeedbackManager()
    vibrotactileFeedbackManager.forAudioCheck()
    print('\n--- End Audio Check ---')