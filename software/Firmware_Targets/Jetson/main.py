import math #uhh
from queue import Queue
from I2C_Driver.I2C_Handler import Jetson_I2C_Handler
from I2C_Driver.I2C_Devices.Jetson_Flywheel_Controller_Driver import FlywheelControllerDriver
from I2C_Driver.I2C_Devices.Jetson_Gimbal_Controller_Driver import GimbalControllerDriver

YAW_I2C_ADDRESS = 0x54 
PITCH_I2C_ADDRESS = 0x55
LAUNCHER_I2C_ADDRESS = 0x62


def main():
    i2c_queue = Queue()
    i2cHandler = Jetson_I2C_Handler(1, i2c_queue)
    
    gimbal = GimbalControllerDriver(i2cHandler, YAW_I2C_ADDRESS, PITCH_I2C_ADDRESS)
    launcher = FlywheelControllerDriver(i2cHandler, LAUNCHER_I2C_ADDRESS)

    #start the camera
    #start the predictor
    #start the webserver
    #<3
