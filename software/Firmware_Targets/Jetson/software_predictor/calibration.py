#new file to make setup a lot easier. can set x y z offset of camera to the turret for adjusted math, 
#as well as a straigten function to make the current positioning of the turret "0,0,0", or just neutral

import math
import BANG
import numpy 

CameraOffset = numpy.array ([ #these values need changing. 

    -0.10, #x
    -5 #y
    -16  #z

])
def straightenUp (x , y , z , yawDegrees, pitchDegrees):  #zeroes out all coords, will call this at startup of launcher

    yaw = math.radians(-yawDegrees)
    pitch = math.radians(-pitchDegrees)

    yawRotation = numpy.array([
        [ math.cos(yaw), 0, math.sin(yaw)],
        [ 0,             1, 0            ],
        [-math.sin(yaw), 0, math.cos(yaw)]
    ])

    pitchRotation = numpy.array([
        [1, 0,                0               ],
        [0, math.cos(pitch), -math.sin(pitch)],
        [0, math.sin(pitch),  math.cos(pitch)]
    ])

    point = numpy.array([x,y,z])

    zeroed = pitchRotation @ yawRotation @ point

    return zeroed


def camera_to_turret(x, y, z):
    point = numpy.array([x, y, z])

    corrected = point + CameraOffset

    return corrected