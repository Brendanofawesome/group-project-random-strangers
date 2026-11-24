"""Driver-level controls and an optional GUI demo. All angles here are degrees."""
from dataclasses import dataclass, replace
from math import degrees, radians, isfinite
from time import monotonic

YAW_ADDRESS = 0x54
PITCH_ADDRESS = 0x55
LAUNCHER_ADDRESS = 0x62
I2C_BUS = 1


@dataclass
class State:
    yaw: float = 0.0
    pitch: float = 0.0
    yaw_target: float = 0.0
    pitch_target: float = 0.0
    yaw_ready: bool = False
    pitch_ready: bool = False
    launch_available: bool = False
    launch_ready: bool = False
    flywheel_armed: bool = False
    solenoid_armed: bool = False
    percent: float = 0.0


def finite(value):
    value = float(value)
    if not isfinite(value):
        raise ValueError("Enter a finite number")
    return value


def percent_value(value):
    value = finite(value)
    if not 0 <= value <= 100:
        raise ValueError("Flywheel target must be between 0 and 100%")
    return value


class Hardware:
    def __init__(self):
        # Import lazily so --demo needs only Python and Tkinter.
        from I2C_Handler import Jetson_I2C_Handler
        from Jetson_Gimbal_Controller_Driver import GimbalControllerDriver
        from Jetson_Flywheel_Controller_Driver import FlywheelControllerDriver

        self._handler = Jetson_I2C_Handler(I2C_BUS, doErrorChecking=False)
        try:
            self.gimbal = GimbalControllerDriver(self._handler, YAW_ADDRESS=YAW_ADDRESS, PITCH_ADDRESS=PITCH_ADDRESS)
            self.launch = FlywheelControllerDriver(self._handler, I2C_Address=LAUNCHER_ADDRESS)
            self._handler.startDriverThread()
        except Exception:
            self._handler.close()
            raise
        self.state = State(launch_available=True)

    @staticmethod
    def _wait(results):
        # Submit a batch first, then use one deadline for the entire batch.
        deadline = monotonic() + 1.0
        return [result.result(timeout=max(0, deadline - monotonic())) for result in results]

    def snapshot(self):
        g, launch = self.gimbal, self.launch
        reads = [g.read_current_yaw(), g.read_current_pitch(), g.read_target_yaw(),
                 g.read_target_pitch(), g.YAW_Ready(), g.PITCH_Ready()]
        if launch:
            reads += [launch.ready(), launch.flywheelsArmed(), launch.solenoidArmed(), launch.flywheelRPMTarget()]
        values = self._wait(reads)
        angles = [degrees(finite(value)) for value in values[:4]]
        state = State(*angles, bool(values[4]), bool(values[5]), launch is not None)
        if launch:
            state.launch_ready, state.flywheel_armed, state.solenoid_armed = map(bool, values[6:9])
            state.percent = percent_value(values[9])
        self.state = state
        return replace(state)

    def move(self, yaw, pitch):
        yaw, pitch = finite(yaw), finite(pitch)
        if not (self.state.yaw_ready and self.state.pitch_ready):
            raise RuntimeError("Gimbal is not ready")
        self._wait(self.gimbal.set_target_angles(radians(yaw), radians(pitch)))
        self.state.yaw_target, self.state.pitch_target = yaw, pitch

    def jog(self, axis, amount):
        yaw, pitch = self.state.yaw_target, self.state.pitch_target
        self.move(yaw + (amount if axis == "yaw" else 0), pitch + (amount if axis == "pitch" else 0))

    def hold(self):
        state = self.snapshot()
        self.move(state.yaw, state.pitch)

    def set_percent(self, percent):
        percent = percent_value(percent)
        self._require_launch()
        self._wait([self.launch.setFlywheelRPMTarget(percent)])

    def _require_launch(self):
        if not self.launch or not self.state.launch_ready:
            raise RuntimeError("Launch controller is not ready")

    def toggle_arm(self, solenoid=False):
        self._require_launch()
        launch = self.launch
        read, arm, disarm = ((launch.solenoidArmed, launch.armSolenoid, launch.disarmSolenoid) if solenoid
                             else (launch.flywheelsArmed, launch.armFlywheels, launch.disarmFlywheels))
        armed = self._wait([read()])[0]
        if not solenoid and not armed:
            # Arming never reuses an old nonzero throttle target.
            self._wait([launch.setFlywheelRPMTarget(0.0)])
        self._wait([disarm() if armed else arm()])

    def fire(self):
        self._require_launch()
        if not self._wait([self.launch.solenoidArmed()])[0]:
            raise RuntimeError("Arm the solenoid before firing")
        self._wait([self.launch.fireSolenoid()])

    def stop_launch(self):
        if self.launch:
            # Attempt every command even if another command fails.
            errors = []
            for action in (self.launch.cancelSolenoidFire, self.launch.disarmSolenoid,
                           lambda: self.launch.setFlywheelRPMTarget(0.0), self.launch.disarmFlywheels):
                try:
                    self._wait([action()])
                except Exception as error:
                    errors.append(str(error))
            if errors:
                raise RuntimeError("Launch stop was not fully acknowledged: " + "; ".join(errors))

    def close(self, stop_launch=True):
        try:
            if stop_launch:
                self.stop_launch()
        finally:
            self._handler.close()


class Demo:
    def __init__(self, *args):
        self.state = State(yaw_ready=True, pitch_ready=True, launch_available=True, launch_ready=True)

    def snapshot(self):
        for axis in ("yaw", "pitch"):
            current, target = getattr(self.state, axis), getattr(self.state, axis + "_target")
            setattr(self.state, axis, current + max(-2.0, min(2.0, target - current)))
        return replace(self.state)

    def move(self, yaw, pitch):
        self.state.yaw_target, self.state.pitch_target = finite(yaw), finite(pitch)

    def jog(self, axis, amount):
        name = axis + "_target"
        setattr(self.state, name, getattr(self.state, name) + finite(amount))

    def hold(self):
        self.move(self.state.yaw, self.state.pitch)

    def set_percent(self, value):
        self.state.percent = percent_value(value)

    def toggle_arm(self, solenoid=False):
        name = "solenoid_armed" if solenoid else "flywheel_armed"
        if not solenoid and not getattr(self.state, name):
            self.state.percent = 0.0
        setattr(self.state, name, not getattr(self.state, name))

    def fire(self):
        if not self.state.solenoid_armed:
            raise RuntimeError("Arm the solenoid before firing")

    def stop_launch(self):
        self.state.percent = 0.0
        self.state.flywheel_armed = self.state.solenoid_armed = False

    def close(self, stop_launch=True):
        if stop_launch:
            self.stop_launch()
