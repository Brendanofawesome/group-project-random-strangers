from queue import Queue
import math
from Jetson_Gimbal_Controller_Driver import GimbalControllerDriver
from Jetson_Flywheel_Controller_Driver import FlywheelControllerDriver
from I2C_Handler import Jetson_I2C_Handler

q = Queue(maxsize=0)
i2c_handler = Jetson_I2C_Handler(1, q, False)

gimbal_driver = GimbalControllerDriver(i2c_handler,0x54,0x55)
flywheel_driver = FlywheelControllerDriver(i2c_handler,0x62)

assert gimbal_driver.PITCH_Ready().result(timeout=10), "Pitch timed out"
assert gimbal_driver.YAW_Ready().result(timeout=10), "Yaw timed out"
gimbal_driver.set_target_angles(0,0)
gimbal_driver.set_target_angles(math.pi/4, math.pi/4)


flywheel_driver.armFlywheels()
assert flywheel_driver.flywheelsArmed().result(timeout=10), "flywheels timed out"

flywheel_driver.setFlywheelRPMTarget(30)
assert flywheel_driver.flywheelRPMTarget().result() == 30
print("Left flywheel speed = " + flywheel_driver.leftActualFlywheelRPM().result())
print("Right flywheel speed = " + flywheel_driver.rightActualFlywheelRPM().result())
flywheel_driver.setFlywheelRPMTarget(0)
assert flywheel_driver.flywheelRPMTarget().result() == 0
flywheel_driver.disarmFlywheels()
assert not flywheel_driver.flywheelsArmed()

flywheel_driver.armSolenoid()
assert flywheel_driver.solenoidArmed().result()
flywheel_driver.fireSolenoid()
flywheel_driver.disarmSolenoid()
assert not flywheel_driver.armSolenoid().result()




