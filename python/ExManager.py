import logging
from RobotControlManager.RobotControlManager import RobotControlManager

def main():
    logging.basicConfig(filename='mylog.log', encoding='utf-8', format='%(asctime)s - %(levelname)s:%(message)s', level=logging.DEBUG)
    robot_control_manager = RobotControlManager()
    robot_control_manager.SendDataToRobot()
    logging.info('----- End program: ExManager.py -----')

if __name__ == '__main__':
    main()