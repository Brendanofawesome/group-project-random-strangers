import struct
import BANG
import math


def test_gimbal_at_target():
    targetYaw = math.radians(10)
    targetPitch = math.radians(5)

    currentYaw = math.radians(10.5)
    currentPitch = math.radians(4.5)

    result = BANG.gimbal_at_target(
        targetYaw,
        targetPitch,
        currentYaw,
        currentPitch
    )

    assert result is True

def test_gimbal_not_at_target():
    targetYaw = math.radians(10)
    targetPitch = math.radians(5)

    currentYaw = math.radians(15)
    currentPitch = math.radians(5)

    result = BANG.gimbal_at_target(
        targetYaw,
        targetPitch,
        currentYaw,
        currentPitch
    )

    assert result is False