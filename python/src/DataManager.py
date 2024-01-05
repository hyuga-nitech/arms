import csv
import threading
import time
from datetime import datetime

import numpy as np
from matplotlib import pyplot as plt


class DataRecordManager():
    def __init__(self, custom: bool = False,  header: list = None, fileName: str = '') -> None:
        self.data = []
        self.header = header
        self.fileName = fileName
        self.record_flag = False

        if custom == True:
            switch_thread = threading.Thread(target=self.key_thread)
            switch_thread.setDaemon(True)
            switch_thread.start()

    def record(self, data):
        self.data.append(data)

    def custom_record(self, data):
        if self.record_flag == True:
            self.data.append(data)
        else:
            pass

    def key_thread(self):
        key_count = 0

        while True:
            key = input('push foot switch start recording')

            if key == 'f':
                key_count += 1

                if key_count % 2 == 1:
                    self.record_flag = True
                else:
                    self.record_flag = False

            time.sleep(0.5)

    def exportAsCSV(self):
        date = datetime.now().strftime("%Y%m%d_%H%M%S")
        with open('resource/' + self.fileName + date + '.csv', 'w', newline='') as exportFile:
            writer = csv.writer(exportFile)
            writer.writerow(self.header)
            writer.writerows(self.data)

class DataLoadManager:
    def __init__(self, path) -> None:
        self.data = 0
        self.load(path)

    def load(self, path):
        with open(path) as file:
            reader = csv.reader(file)
            data = [row for row in reader][1:]
            data = [[float(v) for v in row] for row in data]
            data = np.array(data)
            self.data_iter = iter(data)

    def getdata(self):
        try:
            self.data = next(self.data_iter)
        except StopIteration:
            pass

        return self.data

class DataPlotManager:
    thres = 0
    def __init__(self, legend: list = None, xlabel: str = None, ylabel: str = None) -> None:
        self.data = []
        self.legend = legend
        self.xlabel = xlabel
        self.ylabel = ylabel

    def record(self, data):
        self.data.append(data)

    def plotGraph(self):
        data = np.array(self.data)
        # for i in range(len(self.legend)):
        #     if i == 1:
        #         index = np.where(data[:, -1] == DataPlotManager.thres)[0][0]
        #         print(index)
        #         plt.plot(data[index+1:, -1], data[index+1:, i], label = self.legend[i])
        #     else:
        #         plt.plot(data[:, -1], data[:, i], label = self.legend[i])
        for i in range(len(self.legend)):
            plt.plot(data[:, -1], data[:, i], label = self.legend[i])
        if self.xlabel:
            plt.xlabel(self.xlabel)
        if self.ylabel:
            plt.ylabel(self.ylabel)

        # plt.vlines(x = DataPlotManager.thres, ymin=700, ymax=0, linestyles='dotted', colors='k')
        plt.legend()
        plt.show()

if __name__ == '__main__':
    recorder = DataRecordManager(['x', 'y', 'z'], fileName='position')

    try:
        while True:
            recorder.record([np.random.rand(), np.random.rand(), np.random.rand()])
            time.sleep(0.01)
            # print('111')

    except KeyboardInterrupt:
        recorder.exportAsCSV()
