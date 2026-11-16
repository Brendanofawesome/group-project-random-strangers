import BANG

class FakeFlywheelDriver: 
    def __init__(self):
        self.fireCount = 0

class fireSolenoid:
    def __init__(self):
        self.fireCount += 1

def FireOnceTest():
    Driver = FakeFlywheelDriver

    fireSolenoid(Driver)
    fireSolenoid(Driver)
    fireSolenoid(Driver)

    assert Driver.fireCount == 1

    