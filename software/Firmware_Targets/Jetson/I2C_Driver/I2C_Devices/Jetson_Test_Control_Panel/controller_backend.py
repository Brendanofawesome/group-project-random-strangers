"""Driver-level controls and an optional GUI demo. All angles here are degrees."""
from dataclasses import dataclass, replace
import errno
import logging
from math import degrees, radians, isfinite
from time import monotonic

YAW_ADDRESS = 0x54
PITCH_ADDRESS = 0x55
LAUNCHER_ADDRESS = 0x62
I2C_BUS = 1

logger = logging.getLogger(__name__)
ACK_FAILURES = {errno.ENXIO, getattr(errno, "EREMOTEIO", 121)}


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
    yaw_available: bool = False
    pitch_available: bool = False


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
        self.state = State()
        self._missing = set()

    @staticmethod
    def _wait(results):
        # Submit a batch first, then use one deadline for the entire batch.
        deadline = monotonic() + 1.0
        return [result.result(timeout=max(0, deadline - monotonic())) for result in results]

    def _read_device(self, name, address, ready_reader, readers):
        try:
            ready = bool(self._wait([ready_reader()])[0])
            values = self._wait([reader() for reader in readers]) if ready else None
        except OSError as error:
            if error.errno not in ACK_FAILURES:
                raise
            if name not in self._missing:
                logger.exception("%s (0x%02X): not connected", name.capitalize(), address)
            self._missing.add(name)
            return False, False, None
        self._missing.discard(name)
        return True, ready, values

    def snapshot(self):
        state = replace(self.state)
        g, launch = self.gimbal, self.launch
        for axis, address, ready_reader, readers in (
            ("yaw", YAW_ADDRESS, g.YAW_Ready, (g.read_current_yaw, g.read_target_yaw)),
            ("pitch", PITCH_ADDRESS, g.PITCH_Ready, (g.read_current_pitch, g.read_target_pitch)),
        ):
            available, ready, values = self._read_device(axis, address, ready_reader, readers)
            setattr(state, axis + "_available", available)
            setattr(state, axis + "_ready", ready)
            if values is not None:
                setattr(state, axis, degrees(finite(values[0])))
                setattr(state, axis + "_target", degrees(finite(values[1])))
        state.launch_available, state.launch_ready, values = self._read_device(
            "launcher", LAUNCHER_ADDRESS, launch.ready,
            (launch.flywheelsArmed, launch.solenoidArmed, launch.flywheelRPMTarget),
        )
        state.flywheel_armed = state.solenoid_armed = False
        if values is not None:
            state.flywheel_armed, state.solenoid_armed = map(bool, values[:2])
            state.percent = percent_value(values[2])
        self.state = state
        return replace(state)

    def move(self, yaw=None, pitch=None):
        targets = {axis: finite(value) for axis, value in (("yaw", yaw), ("pitch", pitch)) if value is not None}
        if not targets:
            raise RuntimeError("No gimbal axis is ready")
        for axis in targets:
            if not getattr(self.state, axis + "_ready"):
                raise RuntimeError(f"{axis.capitalize()} is not ready")
        for axis, value in targets.items():
            self._wait([getattr(self.gimbal, "set_target_" + axis)(radians(value))])
            setattr(self.state, axis + "_target", value)

    def jog(self, axis, amount):
        if axis not in ("yaw", "pitch"):
            raise ValueError("Unknown gimbal axis")
        self.move(**{axis: getattr(self.state, axis + "_target") + finite(amount)})

    def hold(self):
        state = self.snapshot()
        self.move(state.yaw if state.yaw_ready else None, state.pitch if state.pitch_ready else None)

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
        if self.launch and "launcher" not in self._missing:
            # Attempt every command even if another command fails.
            errors = []
            for name, action in (("cancel fire", self.launch.cancelSolenoidFire),
                                 ("disarm solenoid", self.launch.disarmSolenoid),
                                 ("zero flywheel target", lambda: self.launch.setFlywheelRPMTarget(0.0)),
                                 ("disarm flywheels", self.launch.disarmFlywheels)):
                try:
                    self._wait([action()])
                except Exception as error:
                    logger.exception("Launch stop command failed: %s", name)
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
        self.state = State(yaw_ready=True, pitch_ready=True, launch_available=True, launch_ready=True,
                           yaw_available=True, pitch_available=True)

    def snapshot(self):
        for axis in ("yaw", "pitch"):
            current, target = getattr(self.state, axis), getattr(self.state, axis + "_target")
            setattr(self.state, axis, current + max(-2.0, min(2.0, target - current)))
        return replace(self.state)

    def move(self, yaw=None, pitch=None):
        if yaw is not None:
            self.state.yaw_target = finite(yaw)
        if pitch is not None:
            self.state.pitch_target = finite(pitch)

    def jog(self, axis, amount):
        name = axis + "_target"
        setattr(self.state, name, getattr(self.state, name) + finite(amount))

    def hold(self):
        self.move(self.state.yaw if self.state.yaw_ready else None,
                  self.state.pitch if self.state.pitch_ready else None)

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
