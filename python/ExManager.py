import logging
from RobotControlManager.RobotControlManager import RobotControlManager

logging.basicConfig(filename='mylog.log', encoding='utf-8', format='%(asctime)s - %(levelname)s:%(message)s', level=logging.DEBUG)

if __name__ == '__main__':
    robot_control_manager = RobotControlManager()
    robot_control_manager.SendDataToRobot()
    logging.info('----- End program: ExManager.py -----')
