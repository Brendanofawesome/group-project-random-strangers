#!/usr/bin/env python3
"""Compact Jetson I2C control panel. Run with --demo to preview without hardware."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from time import monotonic
import tkinter as tk
from tkinter import ttk

from .controller_backend import Demo, Hardware, I2C_BUS, finite


class ControlPanel(tk.Tk):
    def __init__(self, options):
        super().__init__()
        self.options = options
        self.title("Gimbal + Launch")
        self.resizable(False, False)
        self.configure(bg="#111827")
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="panel")
        self.backend = None
        self.jobs = []
        self.connected = self.online = self.closing = self.transition = False
        self.next_poll = 0.0
        self.state = None
        self.status = tk.StringVar(value="Demo mode" if options.demo else "Disconnected")
        self.current_yaw = tk.StringVar(value="—")
        self.current_pitch = tk.StringVar(value="—")
        self.target_label = tk.StringVar(value="Target   — / —")
        self.yaw_input = tk.StringVar(value="0")
        self.pitch_input = tk.StringVar(value="0")
        self.step = tk.StringVar(value="1")
        self.percent = tk.DoubleVar(value=0)
        self.percent_input = tk.StringVar(value="0")
        self.applied = tk.StringVar(value="Applied   —")
        self.launch_status = tk.StringVar(value="Not connected")
        self.gimbal_controls, self.launch_controls = [], []
        self._styles()
        self._layout()
        self._enable_controls()
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.after(50, self._tick)
        if options.demo:
            self.after(100, self.connect)

    def _styles(self):
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure(".", background="#111827", foreground="#e5e7eb", font=("DejaVu Sans", 10))
        style.configure("Card.TFrame", background="#1f2937")
        style.configure("Card.TLabel", background="#1f2937")
        style.configure("Muted.TLabel", foreground="#9ca3af")
        style.configure("Small.TLabel", background="#1f2937", foreground="#9ca3af", font=("DejaVu Sans", 9))
        style.configure("Value.TLabel", background="#1f2937", foreground="#67e8f9", font=("DejaVu Sans", 19, "bold"))
        style.configure("Title.TLabel", font=("DejaVu Sans", 14, "bold"))
        style.configure("Heading.TLabel", background="#1f2937", font=("DejaVu Sans", 11, "bold"))
        style.configure("TButton", background="#374151", borderwidth=0, padding=(9, 6))
        style.map("TButton", background=[("active", "#4b5563"), ("disabled", "#273244")], foreground=[("disabled", "#6b7280")])
        style.configure("Fire.TButton", background="#0e7490", foreground="white")
        style.map("Fire.TButton", background=[("active", "#0891b2"), ("disabled", "#273244")])
        style.configure("Stop.TButton", background="#991b1b", foreground="white")
        style.map("Stop.TButton", background=[("active", "#b91c1c"), ("disabled", "#273244")])
        style.configure("TEntry", fieldbackground="#111827", foreground="#f9fafb", insertcolor="white", padding=4)
        style.configure("TSpinbox", fieldbackground="#111827", foreground="#f9fafb", arrowcolor="#e5e7eb", padding=4)
        style.configure("TCombobox", fieldbackground="#111827", foreground="#f9fafb", arrowcolor="#e5e7eb", padding=3)
        style.map("TCombobox", fieldbackground=[("readonly", "#111827")], foreground=[("readonly", "#e5e7eb")])
        style.configure("Horizontal.TScale", background="#1f2937", troughcolor="#111827")

    def _button(self, parent, text, command, group=None, **grid):
        button = ttk.Button(parent, text=text, command=command, takefocus=False)
        button.grid(**grid)
        if group is not None:
            group.append(button)
        return button

    def _layout(self):
        body = ttk.Frame(self, padding=14)
        body.grid()
        header = ttk.Frame(body)
        header.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 9))
        ttk.Label(header, text="SYSTEM CONTROL", style="Title.TLabel").grid(row=0, column=0, sticky="w")
        header.columnconfigure(0, weight=1)
        ttk.Label(header, text=f"Bus {I2C_BUS}", style="Muted.TLabel").grid(row=0, column=1, padx=(22, 10))
        self.connect_button = self._button(header, "Connect", self.connect, row=0, column=3)
        ttk.Label(header, textvariable=self.status, style="Muted.TLabel", wraplength=535).grid(row=1, column=0, columnspan=4, sticky="w", pady=(5, 0))

        gimbal = ttk.Frame(body, padding=12, style="Card.TFrame")
        gimbal.grid(row=1, column=0, sticky="nsew", padx=(0, 9))
        ttk.Label(gimbal, text="GIMBAL", style="Heading.TLabel").grid(row=0, column=0, columnspan=3, sticky="w")
        for col, title, value in ((0, "YAW", self.current_yaw), (2, "PITCH", self.current_pitch)):
            ttk.Label(gimbal, text=title, style="Small.TLabel").grid(row=1, column=col, pady=(10, 0))
            ttk.Label(gimbal, textvariable=value, width=8, anchor="center", style="Value.TLabel").grid(row=2, column=col)
        arrows = ttk.Frame(gimbal, style="Card.TFrame")
        arrows.grid(row=3, column=0, columnspan=3, pady=(7, 5))
        for text, row, col, axis, sign in (("Pitch +", 0, 1, "pitch", 1), ("Yaw -", 1, 0, "yaw", -1),
                                          ("Yaw +", 1, 2, "yaw", 1), ("Pitch -", 2, 1, "pitch", -1)):
            self._button(arrows, text, lambda a=axis, s=sign: self.jog(a, s), self.gimbal_controls,
                         row=row, column=col, padx=2, pady=2)
        self._button(arrows, "Hold", lambda: self.action("Hold requested", "hold"), self.gimbal_controls,
                     row=1, column=1, padx=2, pady=2)
        ttk.Label(gimbal, textvariable=self.target_label, style="Small.TLabel").grid(row=4, column=0, columnspan=3)
        inputs = ttk.Frame(gimbal, style="Card.TFrame")
        inputs.grid(row=5, column=0, columnspan=3, pady=(10, 0))
        for col, label, variable in ((0, "Yaw °", self.yaw_input), (2, "Pitch °", self.pitch_input)):
            ttk.Label(inputs, text=label, style="Card.TLabel").grid(row=0, column=col, padx=(0, 4))
            entry = ttk.Entry(inputs, width=6, textvariable=variable)
            entry.grid(row=0, column=col + 1, padx=(0, 8))
            entry.bind("<Return>", lambda event: self.move())
            self.gimbal_controls.append(entry)
        self._button(inputs, "Go", self.move, self.gimbal_controls, row=0, column=4)
        ttk.Label(inputs, text="Step °", style="Small.TLabel").grid(row=1, column=0, pady=(8, 0))
        ttk.Combobox(inputs, width=5, textvariable=self.step, values=("0.5", "1", "5", "10"), state="readonly").grid(row=1, column=1, sticky="w", pady=(8, 0))

        launch = ttk.Frame(body, padding=12, style="Card.TFrame")
        launch.grid(row=1, column=1, sticky="nsew")
        launch.columnconfigure(0, weight=1)
        ttk.Label(launch, text="LAUNCH", style="Heading.TLabel").grid(row=0, column=0, columnspan=2, sticky="w")
        ttk.Label(launch, textvariable=self.launch_status, style="Small.TLabel").grid(row=1, column=0, columnspan=2, sticky="w", pady=(10, 0))
        throttle = ttk.Frame(launch, style="Card.TFrame")
        throttle.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(12, 2))
        ttk.Label(throttle, text="Target", style="Card.TLabel").grid(row=0, column=0, padx=(0, 7))
        self.percent_entry = ttk.Spinbox(throttle, from_=0, to=100, increment=1, width=5, textvariable=self.percent_input,
                                         command=self._entry_to_scale)
        self.percent_entry.grid(row=0, column=1)
        self.percent_entry.bind("<Return>", lambda event: self.apply_percent())
        self.percent_entry.bind("<FocusOut>", lambda event: self._entry_to_scale())
        self.launch_controls.append(self.percent_entry)
        ttk.Label(throttle, text="%", style="Card.TLabel").grid(row=0, column=2, padx=(4, 0))
        self.scale = ttk.Scale(launch, from_=0, to=100, variable=self.percent, command=self._scale_changed, length=190)
        self.scale.grid(row=3, column=0, columnspan=2, sticky="ew", pady=5)
        self.launch_controls.append(self.scale)
        self._button(launch, "Apply", self.apply_percent, self.launch_controls, row=4, column=0, sticky="ew", padx=(0, 5))
        self.flywheel_arm = self._button(launch, "Arm wheels", lambda: self.action("Flywheel arm request sent", "toggle_arm"),
                                         self.launch_controls, row=4, column=1, sticky="ew")
        ttk.Label(launch, textvariable=self.applied, style="Small.TLabel").grid(row=5, column=0, columnspan=2, sticky="w", pady=(6, 0))
        ttk.Label(launch, text="RPM feedback unavailable", style="Small.TLabel").grid(row=6, column=0, columnspan=2, sticky="w", pady=(3, 13))
        self.solenoid_arm = self._button(launch, "Arm solenoid", lambda: self.action("Solenoid arm request sent", "toggle_arm", True),
                                         self.launch_controls, row=7, column=0, sticky="ew", padx=(0, 5))
        self.fire_button = self._button(launch, "FIRE", lambda: self.action("Fire request sent", "fire"), row=7, column=1, sticky="ew")
        self.fire_button.configure(style="Fire.TButton")
        self.stop_button = self._button(launch, "Stop launch", self.stop_launch, row=8, column=0, columnspan=2, sticky="ew", pady=(9, 0))
        self.stop_button.configure(style="Stop.TButton")
        ttk.Label(body, text="Arrows move one step per click · Hold captures the measured position", style="Muted.TLabel").grid(row=2, column=0, columnspan=2, sticky="w", pady=(10, 0))

    def _scale_changed(self, value):
        self.percent_input.set(f"{float(value):.0f}")

    def _entry_to_scale(self):
        try:
            value = finite(self.percent_input.get())
            if 0 <= value <= 100:
                self.percent.set(value)
        except ValueError:
            pass  # Apply reports incomplete or invalid input.

    def _submit(self, label, function):
        self.jobs.append((self.executor.submit(function), label))

    def _open(self):
        self.backend = Demo() if self.options.demo else Hardware()
        try:
            return self.backend.snapshot()
        except Exception:
            try:
                self.backend.close(stop_launch=False)
            finally:
                self.backend = None
            raise

    def connect(self):
        if self.connected:
            self.disconnect()
            return
        self.status.set("Connecting…")
        self.transition = True
        self.connect_button.state(["disabled"])
        self._submit("connect", self._open)

    def _enable_controls(self):
        state = self.state
        gimbal_ok = self.online and state and state.yaw_ready and state.pitch_ready and not (self.closing or self.transition)
        launch_ok = self.online and state and state.launch_ready and not (self.closing or self.transition)
        for widgets, enabled in ((self.gimbal_controls, gimbal_ok), (self.launch_controls, launch_ok)):
            for widget in widgets:
                widget.state(["!disabled"] if enabled else ["disabled"])
        self.fire_button.state(["!disabled"] if launch_ok and state.solenoid_armed else ["disabled"])
        self.stop_button.state(["!disabled"] if self.connected and state and state.launch_available and not (self.closing or self.transition) else ["disabled"])

    def _show(self, state):
        self.state, self.online = state, True
        self.current_yaw.set(f"{state.yaw:.1f}°")
        self.current_pitch.set(f"{state.pitch:.1f}°")
        self.target_label.set(f"Target   {state.yaw_target:.1f}° / {state.pitch_target:.1f}°")
        self.flywheel_arm.configure(text="Disarm wheels" if state.flywheel_armed else "Arm wheels")
        self.solenoid_arm.configure(text="Disarm solenoid" if state.solenoid_armed else "Arm solenoid")
        self.applied.set(f"Applied   {state.percent:.1f}%" if state.launch_available else "Applied   —")
        self.launch_status.set(("Ready" if state.launch_ready else "Starting…") if state.launch_available else "Unavailable")
        self._enable_controls()

    def action(self, message, method, *args):
        if not self.connected or not self.online or self.closing or self.transition:
            return
        if len(self.jobs) >= 4:
            self.status.set("Please wait for pending commands")
            return
        self._submit(message, lambda: getattr(self.backend, method)(*args))

    def move(self):
        try:
            yaw, pitch = finite(self.yaw_input.get()), finite(self.pitch_input.get())
        except ValueError as error:
            self.status.set(str(error))
            return
        self.action("Gimbal target sent", "move", yaw, pitch)

    def jog(self, axis, sign):
        self.action("Gimbal step sent", "jog", axis, sign * float(self.step.get()))

    def apply_percent(self):
        try:
            value = finite(self.percent_input.get())
            if not 0 <= value <= 100:
                raise ValueError("Flywheel target must be between 0 and 100%")
        except ValueError as error:
            self.status.set(str(error))
            return
        self.percent.set(value)
        self.action("Flywheel percentage sent", "set_percent", value)

    def _cancel_pending(self):
        self.jobs = [(future, label) for future, label in self.jobs if not future.cancel()]

    def stop_launch(self):
        if not self.connected or self.closing:
            return
        self._cancel_pending()
        self.percent.set(0)
        self.percent_input.set("0")
        self._submit("Launch stop sent", self.backend.stop_launch)

    def _release(self):
        try:
            if self.backend:
                self.backend.close()
        finally:
            self.backend = None

    def disconnect(self):
        self.online = False
        self.transition = True
        self.connect_button.state(["disabled"])
        self._enable_controls()
        self._cancel_pending()
        self._submit("disconnect", self._release)

    def close(self):
        if self.closing:
            return
        self.closing = True
        self.status.set("Disarming launch and closing…")
        self._enable_controls()
        self.connect_button.state(["disabled"])
        self._cancel_pending()
        self._submit("close", self._release)

    def _tick(self):
        remaining = []
        for future, label in self.jobs:
            if not future.done():
                remaining.append((future, label))
                continue
            try:
                result = future.result()
                if label == "connect":
                    self.transition = False
                    self.connected = True
                    self.connect_button.configure(text="Disconnect")
                    self.connect_button.state(["!disabled"])
                    self.yaw_input.set(f"{result.yaw_target:.1f}")
                    self.pitch_input.set(f"{result.pitch_target:.1f}")
                    self.percent.set(result.percent)
                    self.percent_input.set(f"{result.percent:g}")
                    self._show(result)
                    self.status.set("Demo · simulated angles" if self.options.demo else f"Connected · bus {I2C_BUS}")
                elif label == "poll":
                    recovering = not self.online
                    self._show(result)
                    if recovering:
                        self.status.set("Communication restored")
                elif label not in ("close", "disconnect"):
                    self.status.set(label)
                    self.next_poll = 0
            except Exception as error:
                self.status.set(str(error))
                if label in ("connect", "poll"):
                    self.online = False
                    self.current_yaw.set("—")
                    self.current_pitch.set("—")
                    self.applied.set("Applied   —")
                    self.launch_status.set("Communication unavailable")
                    self._enable_controls()
            finally:
                if label == "connect" and not self.connected:
                    self.transition = False
                    self.connect_button.state(["!disabled"])
                if label in ("close", "disconnect"):
                    self.transition = False
                    self.connected = self.online = False
                    self.current_yaw.set("—")
                    self.current_pitch.set("—")
                    self.target_label.set("Target   — / —")
                    self.applied.set("Applied   —")
                    self.launch_status.set("Not connected")
                    self._enable_controls()
                    if label == "close":
                        if future.exception():
                            # Keep a failed stop visible until the user closes again.
                            self.closing = False
                            self.status.set("Close failed: " + str(future.exception()))
                        else:
                            self.executor.shutdown(wait=False, cancel_futures=True)
                            self.destroy()
                            return
                    self.connect_button.configure(text="Connect")
                    self.connect_button.state(["!disabled"])
                    if not future.exception():
                        self.status.set("Disconnected")
        self.jobs = remaining
        if self.connected and not self.closing and not self.transition and not self.jobs and monotonic() >= self.next_poll:
            self.next_poll = monotonic() + 0.2
            self._submit("poll", self.backend.snapshot)
        self.after(50, self._tick)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", action="store_true", help="Preview without hardware or smbus2")
    ControlPanel(parser.parse_args()).mainloop()


if __name__ == "__main__":
    main()
