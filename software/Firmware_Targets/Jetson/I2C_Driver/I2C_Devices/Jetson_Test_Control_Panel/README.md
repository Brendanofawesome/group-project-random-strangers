# Jetson Gimbal + Launch Control

A compact Tkinter panel using the supplied Jetson drivers. Requires Python 3.10+.

## Run on the Jetson

Extract the folder, open a terminal in it, and install the dependency:

```bash
python3 -m pip install -r requirements.txt
python3 control_panel.py
```

Click **Connect** to use I2C bus **1**. Your user must have access to `/dev/i2c-1`. Tkinter must be installed; on Ubuntu it is provided by `python3-tk`.

The controller addresses are fixed: **Pitch 0x55**, **Yaw 0x54**, **Launcher 0x62**. The drivers handle their own register maps using their existing `address_map_grabber` calls. Run with the working directory and driver header paths used by your Jetson project.

To preview the panel without hardware or smbus2:

```bash
python3 control_panel.py --demo
```

## Controls

- **Yaw +/- / Pitch +/-:** move the target by the selected degree step, once per click. Positive and negative follow your firmware's axis conventions.
- **Live yaw/pitch:** measured angles from `CURRENT_ANGLE`, displayed in degrees. Targets and I2C payloads are converted to/from radians.
- **Hold:** read the measured angles and command those positions for the ready axes. This keeps motor control active; it is not a motor-disable command.
- **Yaw / Pitch + Go:** command absolute target angles in degrees for the ready axes. A missing axis has greyed-out controls and is skipped by Go and Hold. Entries are initialized from the controller targets when connecting or recovering. The panel does not invent mechanical travel limits; your firmware must enforce the system's limits.
- **Flywheel Target + Apply:** send a float percentage directly, from **0 to 100**, through `setFlywheelRPMTarget`. The slider only edits the proposed value; Apply sends it. Applied shows the target read back from the controller.
- **Arm wheels / Disarm wheels:** request the corresponding arm state. Arming first sets the target to 0%; then set and apply the desired percentage.
- **Arm solenoid / Disarm solenoid:** control the launcher separately. **FIRE** becomes available when solenoid arming is reported by the controller. Each click sends one fire request.
- **Stop launch:** discard pending GUI commands and send cancel-fire, solenoid-disarm, zero-target, and wheel-disarm requests. Every stop command is attempted even if another fails. This button does not disable the gimbal.
- **Disconnect / window close:** attempt the launch stop sequence if the launcher is connected, then close I2C. A failed stop is shown in the status line.

Actual flywheel RPM is unavailable on this hardware. The panel does not read RPM telemetry or treat a returned zero as a stopped-wheel measurement. Firing does not depend on a measured RPM threshold.

All I2C work runs outside Tkinter's UI thread. Readouts refresh about five times per second. Yaw, pitch, and the launcher are checked independently. An ACK failure shows **Not connected** for that specific device and greys out its commands while the others remain usable. Missing devices are checked again during polling; their controls recover automatically after they respond and become ready. A responding controller that is still initializing shows **Starting**. General bus or unexpected communication failures still show an error and disable ordinary controls until communication recovers.

Errors keep their short GUI status messages and also print full Python tracebacks to the terminal on stderr, including connection, polling, control, and shutdown failures. Each failed launch-stop command prints its original traceback before the combined failure is reported. Run with `python3 control_panel.py` from an open terminal (or `python control_panel.py` from Windows cmd) to see the output. To save terminal errors, use `python3 control_panel.py 2>errors.log`.

A missing device prints its original ACK-failure traceback once when detected, rather than repeating it at every poll. Another traceback is printed if that device recovers and later disconnects again.

## Driver integration

`controller_backend.py` creates the shared handler on bus **1**, instantiates the devices with their fixed addresses, and starts the handler:

```python
self._handler = Jetson_I2C_Handler(1, doErrorChecking=False)
self.gimbal = GimbalControllerDriver(self._handler, YAW_ADDRESS=0x54, PITCH_ADDRESS=0x55)
self.launch = FlywheelControllerDriver(self._handler, I2C_Address=0x62)
self._handler.startDriverThread()
```

The backend owns startup and shutdown of that shared handler and uses driver methods for angles, target percentages, arming, and firing. Register addresses and register-map loading stay inside the drivers. The GUI connects directly through `Hardware()`; no additional connection layer is needed.

The drivers retain their existing register-map loading paths (`../../../Gimbal_Controller/config.h` and `../../../Launch_Controller/config.h`). If your project stores those headers elsewhere, adjust the lookup inside the respective driver. The included `config.h` is the uploaded gimbal header; the launch header remains part of your existing firmware project.

Runtime options:

```bash
python3 control_panel.py --help
python3 control_panel.py --demo
```

The firmware's `SDA_PIN`, `SCL_PIN`, and `I2C_FREQUENCY` describe the microcontroller configuration. The Jetson chooses its pins and clock through Linux/device-tree configuration; this panel opens `/dev/i2c-1`.

Keep the bundled Python files together. They include copies of your drivers with these corrections:

- A separate completion event for every transaction.
- Correct worker start, read validation, completion on errors, and shutdown ordering.
- Correct `smbus2.i2c_msg.write/read`, single-byte register prefixes, and received-byte extraction.
- Local imports and optional per-instance register maps instead of loading firmware headers during import.
- Gimbal measured-angle read methods and independent yaw/pitch target setters, so one axis can operate while the other is missing.

The bundled grabber supports the literal integer macros, `constexpr` integers, and `enum I2C_ADDRESS_MAP` used in the attached config. It does not execute arbitrary C++ expressions. The original driver method names and little-endian float format are retained.

The GUI and message handling were tested against a simulated I2C bus, including invalid transactions, partial stop failures, and GUI interaction. Physical Jetson/controller operation still needs to be checked on your hardware.

Run the included checks with `python3 -m unittest discover -s tests -v`. GUI checks run when a display is available.
