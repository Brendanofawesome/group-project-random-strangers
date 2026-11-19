import math
from queue import Queue

import sys
from pathlib import Path

TESTMODE = True #for testing the pipeline w/o physical camera inputs. 

JETSON_DIR = Path(__file__).resolve().parent.parent

sys.path.insert(0, str(JETSON_DIR))

from I2C_Driver.I2C_Handler import (
    I2CTransaction,
    Jetson_I2C_Handler,
    I2CResult
)

from I2C_Driver.I2C_Devices.Jetson_Flywheel_Controller_Driver import (
    FlywheelControllerDriver
)

def get_coords(x, y, z) : #probably don't touch this 

    #recieves inctercpt coords from projectileMotion

    print("recieved: ")
    print(f"x={x:.3f}, y={y:.3f}, z={z:.3f}")

    yawRadians = math.atan2(
        x,
        -z
    )

    horizontalDistance = math.sqrt( 
        x**2 + z**2
    )

    pitchRadians = math.atan2( #find radians w/ y coord
        y,
        horizontalDistance
    )

    yawDegrees = math.degrees( #convert to degrees 
        yawRadians
    )

    pitchDegrees = math.degrees(
        pitchRadians
    )

    return yawDegrees, pitchDegrees

FLYWHEEL_I2C_ADDRESS = 0x62

transactionQueue = Queue()

i2cHandler = Jetson_I2C_Handler(
    1,
    transactionQueue
)

i2cHandler.register_device(
    "flywheel",
    FLYWHEEL_I2C_ADDRESS,
    False
)

i2cHandler.startDriverThread()

flywheelDriver = FlywheelControllerDriver(
    i2cHandler,
    FLYWHEEL_I2C_ADDRESS
)

angleTolerance = math.radians(1.0)


def gimbalTARGET ( 
    targetYaw, 
    targetPitch, 
    currentYaw = None,
    currentPitch = None
): 

    if TESTMODE: 
        currentYaw = targetYaw
        currentPitch = targetPitch
    return (
        abs(targetYaw - currentYaw) <= angleTolerance
        and 
        abs(targetPitch - currentPitch) <= angleTolerance
    )

fired = False

def fire(): #flywheelDriver import here prveiously
    global fired

    if fired:
        return False

    flywheelDriver.fireSolenoid()
    print("[LAUNCHER] FIRE command sent")

    fired = True
    return True

def reset ():
    global fired
    fired=False
    print("[LAUNCHER] Fire state reset")