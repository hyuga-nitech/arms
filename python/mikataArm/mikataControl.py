# -----------------------------------------------------------------
# Author:          Hyuga Suzuki
# Created:         2022/5/24
# Summary:         mikataArm制御クラス（win用）
# -----------------------------------------------------------------

from dynamixel_sdk import * # Uses Dynamixel SDK library

class mikataControl:
    def __init__(self,mikataPort):
        # ----- setting:dynamixel ----- #

        self.ADDR_TORQUE_ENABLE          = 64
        self.ADDR_PROFILE_ACCELERATION   = 108
        self.ADDR_PROFILE_VELOCITY       = 112
        self.ADDR_GOAL_POSITION          = 116
        self.ADDR_PRESENT_POSITION       = 132

        self.prflAcc                     = 10000
        self.prflVel                     = 10000
        self.BAUDRATE                    = 1000000
        #DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
        #DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual

        # ----- setting:serial ----- #

        # https://emanual.robotis.com/docs/en/dxl/protocol2/
        self.PROTOCOL_VERSION            = 2.0

        # Use the actual port assigned to the U2D2.
        # ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"

        self.DEVICENAME = mikataPort

        self.DXL_ID                      = [1,2,3,4,5]

        self.TORQUE_ENABLE               = 1     # Value for enabling the torque
        self.TORQUE_DISABLE              = 0     # Value for disabling the torque
        self.DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

        # ----- setup: port & packet ----- #

        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        self.dxl_goal_position = []


    def OpenPort(self):
        # Open Port
        if self.portHandler.openPort():

            # Set Baudrate
            if self.portHandler.setBaudRate(self.BAUDRATE):

                for i in range(len(self.DXL_ID)):
                    self.dxl_comm_result, self.dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID[i], self.ADDR_PROFILE_ACCELERATION, self.prflAcc)
                    if self.dxl_comm_result != COMM_SUCCESS:
                        print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
                    elif self.dxl_error != 0:
                        print("%s" % self.packetHandler.getRxPacketError(self.dxl_error))

                    self.dxl_comm_result, self.dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID[i], self.ADDR_PROFILE_VELOCITY, self.prflVel)
                    if self.dxl_comm_result != COMM_SUCCESS:
                        print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
                    elif self.dxl_error != 0:
                        print("%s" % self.packetHandler.getRxPacketError(self.dxl_error))
                    
                # Enable Dynamixel Torque
                for i in range(len(self.DXL_ID)):
                    self.dxl_comm_result, self.dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID[i], self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
                    if self.dxl_comm_result != COMM_SUCCESS:
                        print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
                    elif self.dxl_error != 0:
                        print("%s" % self.packetHandler.getRxPacketError(self.dxl_error))
                    else:
                        pass
            else:
                print("Failed to change the baudrate")
                quit()
        else:
            print("Failed to open the port")
            quit()
        
    def SendtomikataArm(self):
        try:
            while True:
                for i in range(len(self.DXL_ID)):
                    self.dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID[i], self.ADDR_GOAL_POSITION, self.dxl_goal_position[i])
                    if self.dxl_comm_result != COMM_SUCCESS:
                        print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
                        pass
                    elif dxl_error != 0:
                        print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                        pass

        except KeyboardInterrupt:
            print('KeyboardInterrupt >> Stop: SendtomikataArm')

    def ClosePort(self):
        self.portHandler.closePort()

    def DisableTorque(self):
        for i in range(len(self.DXL_ID)):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID[i], self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))