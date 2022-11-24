# -----------------------------------------------------------------------
# Author:   Takumi Katagiri (Nagoya Institute of Technology), Takayoshi Hagiwara (KMD)
# Created:  2021/11/17
# Summary:  Get index of audio output devices
# -----------------------------------------------------------------------

import pyaudio

class AudioDeviceIndexes:
    def __init__(self) -> None:
        pass
    
    def Find(self, host_api: str = 'Windows DirectSound', name: str = 'Sound Blaster Play! 3'):
        """
        Find devices that contain the specified string in their names

        Parameters
        ----------
        host_api: (Optional) str
            Host api name
            Options: MME, Windows DirectSound, ASIO, Windows WASAPI, Windows WDM-KS
        name: (Optional) str
            Device name
        """
        p = pyaudio.PyAudio()
        ListIndexNumbefore = []
        ListIndexNum = []

        for host_index in range(0, p.get_host_api_count()):
            host_dict = p.get_host_api_info_by_index(host_index)
            if host_dict['name'] == host_api:
                for device_index in range(0, host_dict['deviceCount']):
                    device_dict = p.get_device_info_by_host_api_device_index(host_index, device_index)

                    # for debug option
                    # print(device_dict)

                    if ('スピーカー' in device_dict['name']) and (name in device_dict['name']):
                        ListIndexNumbefore.append(device_index)

                for soundi in range(2,10):
                    for gained_index in ListIndexNumbefore:
                        gained_dict = p.get_device_info_by_host_api_device_index(host_index, gained_index)

                        if ((str(soundi) + '- ') in gained_dict['name']):
                            ListIndexNum.append(gained_dict['index'])

        if len(ListIndexNum) == 0:
            print('サウンドカードが見つかりません')

        return ListIndexNum
