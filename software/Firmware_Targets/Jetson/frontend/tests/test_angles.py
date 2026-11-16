#tests to make sure the angles make sense and are reasonable. 

from software.Firmware_Targets.Jetson.I2C_Driver.Jetson_Gimbal_Controller_Driver import encode_angles

def testEncodedAngles(): 
    yaw, pitch = encode_angles(12.34, -43.21)

    assert pitch == -4321
    assert yaw == 12.34

def testAngleZero():
    yaw, pitch = encode_angles(0,0)

    assert pitch == 0
    assert yaw == 0


    

