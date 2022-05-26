# -----------------------------------------------------------------
# Author:          Hyuga Suzuki
# Original Author: Takayoshi Hagiwara (KMD)
# Created:         2022/5/25
# Summary:         Experiment manager
# -----------------------------------------------------------------

from RobotControlManager.RobotControlManager import RobotControlManager

if __name__ == '__main__':
    robotControlManager = RobotControlManager()
    robotControlManager.SendDataToRobot()
    print('\n----- End program: ExManager.py -----')