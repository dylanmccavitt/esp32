#!/usr/bin/env python3
"""Exercise firmware-shell behavior against a connected board."""

from __future__ import annotations

import argparse
import collections
import dataclasses
import hashlib
import math
import os
import re
import sys
import threading
import time

import device_dev
from device_dev import Device, DeviceGone, FrameEvent, UserError, save_frame

if device_dev.serial is not None:
    from serial.tools import list_ports
else:  # pragma: no cover - main() reports the missing dependency cleanly
    list_ports = None


PANEL_WIDTH = 240
PANEL_HEIGHT = 240
PANEL_BYTES = PANEL_WIDTH * PANEL_HEIGHT * 2
DEFAULT_TIMEOUT = 12.0
DEFAULT_RECONNECT_TIMEOUT = 25.0
DEFAULT_TELEMETRY_TIMEOUT = 4.0
DEFAULT_SOAK_SECONDS = 60.0
DEFAULT_CYCLES = 100
DEFAULT_TOUCH_TIMEOUT = 30.0
TOUCH_RELEASE_QUIET_SECONDS = 0.5
DEFAULT_POWER_OFF_MIN_ABSENCE = 2.0
STATUS_RETRY_SECONDS = 1.0
# @DEV POWEROFF fires at 2000 ms; the 3000 ms gesture then releases and
# traverses the 40 ms debouncer. This margin keeps STATUS strictly post-release.
POWER_OFF_POST_MARKER_RELEASE_SECONDS = 1.1
FREEZE_DRIFT_CALIBRATION_FRAMES = 3
PORT_POLL_SECONDS = 0.05

MODE_NAMES = (
    "launcher",
    "fluid_box",
    "tilt_maze",
    "ragdoll_avalanche",
    "orient_cube",
    "attitude",
)
MODE_MARKERS = {mode: "@DEV MODE {}".format(mode) for mode in MODE_NAMES}
MODE_LAUNCHER = MODE_MARKERS["launcher"]
MODE_FLUID = MODE_MARKERS["fluid_box"]
MODE_MAZE = MODE_MARKERS["tilt_maze"]
MODE_AVALANCHE = MODE_MARKERS["ragdoll_avalanche"]
MODE_CUBE = MODE_MARKERS["orient_cube"]
MODE_LEVEL = MODE_MARKERS["attitude"]
RUNNING_MODES = MODE_NAMES[1:]
CORE_CHECKS = ("launcher", "once", "cycles", "freeze", "reboot")
ALL_CHECKS = CORE_CHECKS + ("soak",)
CHECK_CHOICES = ALL_CHECKS + ("touch", "apps")
COMMAND_FAILURE_MARKERS = (
    "unrecognized command",
    "unknown command",
    "command not found",
    "invalid command",
)
TOUCH_LAUNCH_LOG = re.compile(
    r"I \([0-9]+\) fluid_demo: touch launch x=([0-9]+) y=([0-9]+)"
    r"(?:\x1b\[[0-9;]*m)?$"
)
TELEMETRY_FIELD = re.compile(r"\b(reset_epoch|epoch|missed|nonfinite)=(\d+)\b")
STATUS_PREFIX = "@DEV STATUS "
STATUS_VALUE_PATTERNS = {
    "uptime_ms": re.compile(r"\d+"),
    "override": re.compile(r"[01]"),
    "accel": re.compile(r"-?\d+\.\d{3},-?\d+\.\d{3},-?\d+\.\d{3}"),
    "capture_ready": re.compile(r"[01]"),
    "mode": re.compile("|".join(MODE_NAMES)),
    "battery_hold": re.compile(r"[01]"),
}
STATUS_VALUE_DESCRIPTIONS = {
    "uptime_ms": "an unsigned decimal integer",
    "override": "0 or 1",
    "accel": "three comma-separated fixed-point values with three decimals",
    "capture_ready": "0 or 1",
    "mode": ", ".join(MODE_NAMES),
    "battery_hold": "0 or 1",
}


class AcceptanceTimeout(UserError):
    pass


@dataclasses.dataclass(frozen=True)
class Status:
    uptime_ms: int
    override: int
    accel: str
    capture_ready: int
    mode: str
    battery_hold: int


@dataclasses.dataclass(frozen=True)
class Telemetry:
    sequence: int
    epoch: int | None
    missed: int | None
    nonfinite: int | None
    line: str


@dataclasses.dataclass(frozen=True)
class SerialPortIdentity:
    device: str
    serial_number: str | None
    vid: int | None
    pid: int | None
    location: str | None
    explicit: bool

    @classmethod
    def capture(cls, device: str, explicit: bool) -> "SerialPortIdentity":
        if list_ports is not None:
            for info in list_ports.comports():
                if info.device == device:
                    return cls(
                        device=device,
                        serial_number=info.serial_number,
                        vid=info.vid,
                        pid=info.pid,
                        location=info.location,
                        explicit=explicit,
                    )
        return cls(device, None, None, None, None, explicit)

    def matches(self, candidate) -> bool:
        same_usb_kind = (
            self.vid is None
            or self.pid is None
            or candidate.vid is None
            or candidate.pid is None
            or (candidate.vid, candidate.pid) == (self.vid, self.pid)
        )
        if not same_usb_kind:
            return False
        if self.serial_number is not None:
            if candidate.serial_number != self.serial_number:
                return False
            return (
                self.location is None
                or candidate.location is None
                or candidate.location == self.location
            )
        if self.location is not None:
            return candidate.location == self.location
        return candidate.device == self.device

    def present_devices(self) -> tuple[str, ...]:
        if list_ports is not None:
            matches = tuple(
                sorted(
                    info.device
                    for info in list_ports.comports()
                    if self.matches(info)
                )
            )
            if matches:
                return matches
            if self.explicit and os.path.exists(self.device):
                return (self.device,)
            return ()
        if self.explicit and os.path.exists(self.device):
            return (self.device,)
        return ()


class AcceptanceDevice(Device):

    def __init__(self, *args, **kwargs):
        self._line_condition = threading.Condition()
        self._line_sequence = 0
        self._lines = collections.deque(maxlen=8192)
        self._telemetry = collections.deque()
        super().__init__(*args, **kwargs)

    def emit(self, text: bytes | str):
        decoded = text.decode("utf-8", "replace") if isinstance(text, bytes) else text
        with self._line_condition:
            for raw_line in decoded.splitlines():
                line = raw_line.rstrip("\r")
                if not line:
                    continue
                self._line_sequence += 1
                self._lines.append((self._line_sequence, line))
                if (sample := parse_telemetry(self._line_sequence, line)) is not None:
                    self._telemetry.append(sample)
            self._line_condition.notify_all()
        super().emit(decoded)

    def checkpoint(self) -> int:
        with self._line_condition:
            return self._line_sequence

    def wait_line(self, predicate, after: int, timeout: float, description: str):
        deadline = time.monotonic() + timeout
        cursor = after
        while True:
            with self._line_condition:
                pending_lines = [
                    (sequence, line)
                    for sequence, line in self._lines
                    if sequence > cursor
                ]
                if pending_lines:
                    cursor = pending_lines[-1][0]
                for sequence, line in pending_lines:
                    self._raise_if_command_rejected(line, description)
                    if predicate(line):
                        return sequence, line

                if not self.reader.is_alive():
                    raise DeviceGone(
                        "serial reader stopped while waiting for {}"
                        .format(description)
                    )
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise AcceptanceTimeout(
                        "timed out after {:.1f}s waiting for {}; the firmware "
                        "did not emit the required protocol response"
                        .format(timeout, description)
                    )
                self._line_condition.wait(timeout=min(remaining, 0.25))

    def wait_exact(self, marker: str, after: int, timeout: float):
        return self.wait_line(
            lambda line: line.strip() == marker,
            after,
            timeout,
            repr(marker),
        )

    def wait_prefix(self, marker: str, after: int, timeout: float):
        return self.wait_line(
            lambda line: line.strip().startswith(marker),
            after,
            timeout,
            "a line beginning {!r}".format(marker),
        )

    def wait_telemetry(self, predicate, after: int, timeout: float, description: str):
        return self.wait_line(
            lambda line: (
                (sample := parse_telemetry(0, line)) is not None
                and predicate(sample)
            ),
            after,
            timeout,
            description,
        )

    def telemetry_since(self, after: int) -> list[Telemetry]:
        with self._line_condition:
            return [sample for sample in self._telemetry if sample.sequence > after]

    def lines_since(self, after: int) -> list[tuple[int, str]]:
        with self._line_condition:
            return [(sequence, line) for sequence, line in self._lines if sequence > after]

    def scan_command_failures(self, after: int, description: str) -> int:
        cursor = after
        with self._line_condition:
            for sequence, line in self._lines:
                if sequence <= after:
                    continue
                self._raise_if_command_rejected(line, description)
                cursor = sequence
        return cursor


    @staticmethod
    def _raise_if_command_rejected(line: str, description: str):
        lowered = line.lower()
        if any(marker in lowered for marker in COMMAND_FAILURE_MARKERS):
            raise UserError(
                "firmware rejected a command while waiting for {}: {}. "
                "This firmware does not implement the acceptance-shell protocol."
                .format(description, line.strip())
            )


def parse_telemetry(sequence: int, line: str) -> Telemetry | None:
    fields = {name: int(value) for name, value in TELEMETRY_FIELD.findall(line)}
    if not fields:
        return None
    return Telemetry(
        sequence=sequence,
        epoch=fields.get("reset_epoch", fields.get("epoch")),
        missed=fields.get("missed"),
        nonfinite=fields.get("nonfinite"),
        line=line,
    )


def validate_status(line: str, label: str) -> Status:
    stripped = line.strip()
    if not stripped.startswith(STATUS_PREFIX):
        raise UserError(
            "{} expected an @DEV STATUS response; got: {}".format(label, stripped)
        )

    observed = {name: [] for name in STATUS_VALUE_PATTERNS}
    for token in stripped[len(STATUS_PREFIX):].split():
        name, separator, value = token.partition("=")
        if separator and name in observed:
            observed[name].append(value)

    for name, pattern in STATUS_VALUE_PATTERNS.items():
        values = observed[name]
        if len(values) != 1:
            raise UserError(
                "{} status must contain exactly one {} field; got: {}"
                .format(label, name, stripped)
            )
        if pattern.fullmatch(values[0]) is None:
            raise UserError(
                "{} status {} must be {}; got: {}"
                .format(
                    label,
                    name,
                    STATUS_VALUE_DESCRIPTIONS[name],
                    stripped,
                )
            )

    return Status(
        uptime_ms=int(observed["uptime_ms"][0]),
        override=int(observed["override"][0]),
        accel=observed["accel"][0],
        capture_ready=int(observed["capture_ready"][0]),
        mode=observed["mode"][0],
        battery_hold=int(observed["battery_hold"][0]),
    )


def positive_int(text: str) -> int:
    try:
        value = int(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be an integer") from exc
    if value <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return value


def positive_finite(text: str) -> float:
    try:
        value = float(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a number") from exc
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
    return value


def frame_signature(frame: FrameEvent) -> str:
    return hashlib.sha256(frame.data).hexdigest()


def mean_frame_distance(left: FrameEvent, right: FrameEvent) -> float:
    if (left.width, left.height, len(left.data)) != (
        right.width,
        right.height,
        len(right.data),
    ):
        raise UserError("cannot compare framebuffer signatures with different shapes")

    total = 0
    for offset in range(0, len(left.data), 2):
        left_pixel = (left.data[offset] << 8) | left.data[offset + 1]
        right_pixel = (right.data[offset] << 8) | right.data[offset + 1]
        total += abs((left_pixel >> 11) - (right_pixel >> 11))
        total += abs(((left_pixel >> 5) & 0x3F) - ((right_pixel >> 5) & 0x3F))
        total += abs((left_pixel & 0x1F) - (right_pixel & 0x1F))
    return total / ((len(left.data) // 2) * (31 + 63 + 31))


class FirmwareShellAcceptance:
    def __init__(self, args):
        self.args = args
        self.requested_port = args.port
        self.port = args.port
        self.device = self._connect(args.reconnect_timeout)
        self.port_identity = SerialPortIdentity.capture(
            self.port, explicit=args.port is not None
        )
        self._identity_was_enumerated = bool(
            self.port_identity.present_devices()
        )
        self.mode = None
        self.launcher_frame = None
        self.fluid_frame = None

    def close(self):
        if self.device is not None:
            self.device.close()
            self.device = None

    def _connect(
        self,
        timeout: float,
        identity: SerialPortIdentity | None = None,
    ) -> AcceptanceDevice:
        deadline = time.monotonic() + timeout
        last_error = None
        while time.monotonic() < deadline:
            try:
                if identity is None:
                    port = self.requested_port or device_dev.discover_port()
                else:
                    candidates = identity.present_devices()
                    if not candidates:
                        last_error = (
                            "target identity did not appear "
                            "(serial={!r}, location={!r}, original path={!r})"
                            .format(
                                identity.serial_number,
                                identity.location,
                                identity.device,
                            )
                        )
                        time.sleep(
                            min(0.25, max(0.0, deadline - time.monotonic()))
                        )
                        continue
                    port = (
                        identity.device
                        if identity.device in candidates
                        else candidates[0]
                    )
                device = AcceptanceDevice(
                    port=port,
                    baudrate=self.args.baud,
                    out_dir=self.args.out_dir,
                )
                self.port = port
                print("[accept] connected: {} @ {} baud".format(port, self.args.baud))
                return device
            except (device_dev.PortNotFound, UserError) as exc:
                last_error = exc
                time.sleep(min(0.25, max(0.0, deadline - time.monotonic())))
        raise UserError(
            "could not connect to the firmware within {:.1f}s: {}"
            .format(timeout, last_error or "serial port did not appear")
        )

    def _reconnect(self, timeout: float | None = None):
        if self.device is not None:
            self.device.close()
            self.device = None
        self.device = self._connect(
            self.args.reconnect_timeout if timeout is None else timeout,
            self.port_identity,
        )
        self.mode = None

    def _request_status(self, label: str, timeout: float) -> Status:
        checkpoint = self.device.checkpoint()
        self.device.echo_send("status")
        _, line = self.device.wait_line(
            lambda candidate: candidate.strip().startswith(STATUS_PREFIX),
            checkpoint,
            timeout,
            "the requested status response for {}".format(label),
        )
        return validate_status(line, label)

    def _scan_reboot_lines(
        self,
        device: AcceptanceDevice,
        after: int,
        label: str,
        reboot_sequence: int | None,
        launcher_sequence: int | None,
        through: int | None = None,
    ) -> tuple[int, int | None, int | None]:
        cursor = after
        description = "reboot evidence after {}".format(label)
        for sequence, line in device.lines_since(after):
            if through is not None and sequence > through:
                break
            device._raise_if_command_rejected(line, description)
            cursor = sequence
            stripped = line.strip()
            if stripped == "@DEV REBOOTING":
                reboot_sequence = sequence
                launcher_sequence = None
            elif stripped.startswith("@DEV MODE "):
                if stripped not in MODE_MARKERS.values():
                    raise UserError(
                        "{} emitted unknown mode marker {!r}".format(label, stripped)
                    )
                if reboot_sequence is not None:
                    if stripped != MODE_LAUNCHER:
                        raise UserError(
                            "{} emitted {!r} after @DEV REBOOTING; expected {!r}"
                            .format(label, stripped, MODE_LAUNCHER)
                        )
                    launcher_sequence = sequence
        return cursor, reboot_sequence, launcher_sequence

    def _finish_reconnected_launcher(self, label: str, deadline: float):
        self._reconnect(max(0.001, deadline - time.monotonic()))
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise UserError(
                    "{} re-enumerated, but launcher mode was not observed within "
                    "{:.1f}s".format(label, self.args.reconnect_timeout)
                )

            try:
                status = self._request_status(
                    label,
                    min(STATUS_RETRY_SECONDS, remaining),
                )
            except AcceptanceTimeout:
                continue
            except DeviceGone:
                self._reconnect(max(0.001, deadline - time.monotonic()))
                continue

            if status.mode != "launcher":
                raise UserError(
                    "{} returned in {} mode; expected launcher"
                    .format(label, status.mode)
                )
            observed_markers = {
                candidate.strip()
                for _, candidate in self.device.lines_since(0)
                if candidate.strip() in MODE_MARKERS.values()
            }
            unexpected = observed_markers - {MODE_LAUNCHER}
            if unexpected:
                raise UserError(
                    "{} emitted running MODE marker(s) {} while reconnect status "
                    "reported launcher".format(
                        label, ", ".join(sorted(unexpected))
                    )
                )
            self.mode = status.mode
            print(
                "[accept] {} re-enumerated in launcher mode"
                .format(label)
            )
            return

    def _observe_rebooted_launcher(
        self,
        label: str,
        command_checkpoint: int,
        baseline_uptime_ms: int,
    ):
        deadline = time.monotonic() + self.args.reconnect_timeout
        device = self.device
        if device is None:
            raise UserError(
                "{} cannot prove a reboot because the serial link was already closed"
                .format(label)
            )

        cursor = command_checkpoint
        reboot_sequence = None
        launcher_sequence = None
        while True:
            cursor, reboot_sequence, launcher_sequence = self._scan_reboot_lines(
                device,
                cursor,
                label,
                reboot_sequence,
                launcher_sequence,
            )

            transport_lost = (
                not device.reader.is_alive()
                or (
                    self._identity_was_enumerated
                    and not self.port_identity.present_devices()
                )
            )
            if transport_lost:
                if device._stop.is_set():
                    raise UserError(
                        "{} cannot prove a reboot because the host closed the serial "
                        "link before device-generated transport loss was observed"
                        .format(label)
                    )

                # Stop and join the sole reader before the final scan so a marker
                # appended during transport teardown cannot race reconnect.
                device.close()
                cursor, reboot_sequence, launcher_sequence = self._scan_reboot_lines(
                    device,
                    cursor,
                    label,
                    reboot_sequence,
                    launcher_sequence,
                )
                self.device = None
                if reboot_sequence is None:
                    raise UserError(
                        "{} lost transport before the required post-command "
                        "@DEV REBOOTING marker was observed".format(label)
                    )
                print(
                    "[accept] {} device-generated transport loss observed"
                    .format(label)
                )
                self._finish_reconnected_launcher(label, deadline)
                return

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                device.scan_command_failures(
                    cursor,
                    "reboot evidence after {}".format(label),
                )
                if reboot_sequence is None:
                    missing = "@DEV REBOOTING followed by a fresh launcher marker"
                elif launcher_sequence is None:
                    missing = "a fresh launcher marker after @DEV REBOOTING"
                else:
                    missing = "a validated freshly booted launcher STATUS"
                raise UserError(
                    "{} produced neither transport loss nor {} within {:.1f}s"
                    .format(label, missing, self.args.reconnect_timeout)
                )

            if reboot_sequence is None or launcher_sequence is None:
                time.sleep(min(PORT_POLL_SECONDS, remaining))
                continue

            status_checkpoint = self.device.checkpoint()
            self.device.echo_send("status")
            try:
                status_sequence, status_line = self.device.wait_line(
                    lambda candidate: candidate.strip().startswith(STATUS_PREFIX),
                    status_checkpoint,
                    min(STATUS_RETRY_SECONDS, remaining),
                    "the requested post-boot status response for {}".format(label),
                )
            except AcceptanceTimeout:
                continue
            except DeviceGone:
                continue

            cursor, reboot_sequence, launcher_sequence = self._scan_reboot_lines(
                device,
                cursor,
                label,
                reboot_sequence,
                launcher_sequence,
                status_sequence,
            )
            if reboot_sequence is None or launcher_sequence is None:
                continue

            status = validate_status(status_line, label)
            if status.mode != "launcher":
                raise UserError(
                    "{} returned in {} mode after reboot; expected launcher"
                    .format(label, status.mode)
                )
            post_uptime_ms = status.uptime_ms
            fresh_limit_ms = math.ceil(self.args.reconnect_timeout * 1000.0)
            if post_uptime_ms >= baseline_uptime_ms:
                raise UserError(
                    "{} post-reboot uptime {}ms was not lower than the pre-command "
                    "launcher uptime {}ms"
                    .format(label, post_uptime_ms, baseline_uptime_ms)
                )
            if post_uptime_ms > fresh_limit_ms:
                raise UserError(
                    "{} post-reboot uptime {}ms exceeded the {:.1f}s fresh-boot "
                    "bound".format(
                        label,
                        post_uptime_ms,
                        self.args.reconnect_timeout,
                    )
                )

            transport_lost = (
                not device.reader.is_alive()
                or (
                    self._identity_was_enumerated
                    and not self.port_identity.present_devices()
                )
            )
            if transport_lost:
                continue
            self.mode = status.mode
            print(
                "[accept] {} restarted in place in launcher mode "
                "(uptime {}ms < baseline {}ms)"
                .format(label, post_uptime_ms, baseline_uptime_ms)
            )
            return

    def _aged_reboot_baseline(
        self,
        label: str,
        allowed_modes: tuple[str, ...],
    ) -> int:
        minimum_uptime_ms = 3000
        deadline = time.monotonic() + self.args.timeout
        last_mode = None
        last_uptime_ms = None
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                observed = (
                    "no validated STATUS"
                    if last_uptime_ms is None
                    else "mode={} uptime_ms={}".format(last_mode, last_uptime_ms)
                )
                raise UserError(
                    "{} did not reach the {}ms pre-reboot uptime floor within "
                    "{:.1f}s; last observed {}"
                    .format(
                        label,
                        minimum_uptime_ms,
                        self.args.timeout,
                        observed,
                    )
                )

            try:
                status = self._request_status(
                    label,
                    min(STATUS_RETRY_SECONDS, remaining),
                )
            except AcceptanceTimeout:
                continue
            if status.mode not in allowed_modes:
                raise UserError(
                    "{} returned in {} mode; expected one of {}"
                    .format(label, status.mode, ", ".join(allowed_modes))
                )

            self.mode = status.mode
            last_mode = status.mode
            last_uptime_ms = status.uptime_ms
            if status.uptime_ms >= minimum_uptime_ms:
                print(
                    "[accept] {} confirmed {} mode at {}ms uptime"
                    .format(label, status.mode, status.uptime_ms)
                )
                return status.uptime_ms

            remaining = deadline - time.monotonic()
            if remaining > 0:
                time.sleep(
                    min(
                        STATUS_RETRY_SECONDS,
                        (minimum_uptime_ms - status.uptime_ms) / 1000.0,
                        remaining,
                    )
                )

    def _probe_mode(
        self,
        expected: str,
        label: str,
        timeout: float | None = None,
    ) -> int:
        if expected not in MODE_NAMES:
            raise UserError(
                "{} requested an unsupported mode probe: {}"
                .format(label, expected)
            )

        status = self._request_status(
            label,
            self.args.timeout if timeout is None else timeout,
        )
        if status.mode != expected:
            raise UserError(
                "{} returned in {} mode; expected {}"
                .format(label, status.mode, expected)
            )
        self.mode = status.mode
        print("[accept] {} confirmed {} mode".format(label, status.mode))
        return status.uptime_ms


    def _send_wait_exact(self, command: str, marker: str):
        checkpoint = self.device.checkpoint()
        self.device.echo_send(command)
        return self.device.wait_exact(marker, checkpoint, self.args.timeout)

    def _send_wait_prefix(self, command: str, marker: str):
        checkpoint = self.device.checkpoint()
        self.device.echo_send(command)
        return self.device.wait_prefix(marker, checkpoint, self.args.timeout)

    def _transition(self, command: str, marker: str):
        sequence, _ = self._send_wait_exact(command, marker)
        try:
            self.mode = next(
                mode for mode, known_marker in MODE_MARKERS.items()
                if known_marker == marker
            )
        except StopIteration as exc:
            raise UserError("unsupported transition marker: {}".format(marker)) from exc
        return sequence

    def _boot_to_launcher(self):
        baseline_uptime_ms = self._aged_reboot_baseline(
            "short BOOT pre-command status",
            MODE_NAMES,
        )
        checkpoint = self.device.checkpoint()
        self.device.echo_send("input boot 150")
        try:
            self._observe_rebooted_launcher(
                "short BOOT input",
                checkpoint,
                baseline_uptime_ms,
            )
        except UserError as exc:
            raise UserError(
                "short BOOT input did not prove a reboot into launcher mode: {}"
                .format(exc)
            ) from exc

    def _legacy_reboot_to_launcher(self):
        baseline_uptime_ms = self._aged_reboot_baseline(
            "legacy reboot pre-command status",
            ("launcher",),
        )
        checkpoint = self.device.checkpoint()
        self.device.echo_send("reboot")
        try:
            self.device.wait_exact("@DEV REBOOTING", checkpoint, self.args.timeout)
        except DeviceGone as exc:
            raise UserError(
                "legacy reboot command disconnected before the required "
                "@DEV REBOOTING response was observed"
            ) from exc
        except AcceptanceTimeout as exc:
            raise UserError(
                "legacy reboot command did not emit the required "
                "@DEV REBOOTING response: {}".format(exc)
            ) from exc

        try:
            self._observe_rebooted_launcher(
                "legacy reboot command",
                checkpoint,
                baseline_uptime_ms,
            )
        except UserError as exc:
            raise UserError(
                "legacy reboot command did not prove a reboot into launcher mode: {}"
                .format(exc)
            ) from exc


    def _ensure_launcher(self):
        if self.mode == "launcher":
            return
        if self.mode in RUNNING_MODES:
            self._transition("input pwr 120", MODE_LAUNCHER)
            return
        self._boot_to_launcher()

    def _ensure_fluid(self):
        self._ensure_launcher()
        self._transition("input plus", MODE_FLUID)

    def _capture(self, label: str) -> FrameEvent:
        frame = self.device.capture(self.args.capture_timeout)
        self._assert_frame_shape(frame, label)
        path = save_frame(self.device, frame, self.args.out_dir)
        print(
            "[accept] {} signature={} path={}".format(
                label, frame_signature(frame)[:16], path
            )
        )
        return frame

    @staticmethod
    def _assert_frame_shape(frame: FrameEvent, label: str):
        if (frame.width, frame.height, len(frame.data)) != (
            PANEL_WIDTH,
            PANEL_HEIGHT,
            PANEL_BYTES,
        ):
            raise UserError(
                "{} capture violated the display contract: got {}x{} / {} bytes; "
                "expected 240x240 RGB565BE / 115200 bytes"
                .format(label, frame.width, frame.height, len(frame.data))
            )

    @staticmethod
    def _assert_distinct(left: FrameEvent, right: FrameEvent, labels: str):
        if frame_signature(left) == frame_signature(right):
            raise UserError(
                "{} produced identical framebuffer signatures; launcher and Fluid "
                "must render observably different content"
                .format(labels)
            )

    def _wait_epoch(self, after: int, predicate, description: str) -> Telemetry:
        sequence, line = self.device.wait_telemetry(
            predicate,
            after,
            self.args.telemetry_timeout,
            description,
        )
        sample = parse_telemetry(sequence, line)
        if sample is None:
            raise UserError("internal telemetry observation error")
        return sample

    def _health_baseline(self, after: int, label: str) -> Telemetry:
        sequence, line = self.device.wait_telemetry(
            lambda sample: (
                sample.missed is not None
                and sample.nonfinite is not None
            ),
            after,
            self.args.telemetry_timeout,
            (
                "one Running-mode telemetry line containing both cumulative "
                "missed and nonfinite counters for {}"
            ).format(label),
        )
        baseline = parse_telemetry(sequence, line)
        if (
            baseline is None
            or baseline.missed is None
            or baseline.nonfinite is None
        ):
            raise UserError(
                "{} health baseline did not contain both cumulative missed and "
                "nonfinite counters: {}".format(label, line.strip())
            )
        print(
            "[accept] {} health baseline: missed={} nonfinite={}"
            .format(label, baseline.missed, baseline.nonfinite)
        )
        return baseline

    def _assert_health(
        self,
        baseline: Telemetry,
        action_end: int,
        label: str,
    ):
        post_sequence, post_line = self.device.wait_telemetry(
            lambda sample: (
                sample.missed is not None
                and sample.nonfinite is not None
            ),
            action_end,
            self.args.telemetry_timeout,
            (
                "a post-action Running-mode telemetry line containing both "
                "cumulative missed and nonfinite counters for {}"
            ).format(label),
        )
        post_sample = parse_telemetry(post_sequence, post_line)
        if (
            post_sample is None
            or post_sample.missed is None
            or post_sample.nonfinite is None
        ):
            raise UserError(
                "{} post-action health sample did not contain both cumulative "
                "missed and nonfinite counters: {}".format(
                    label, post_line.strip()
                )
            )

        observed = [baseline]
        observed.extend(
            sample
            for sample in self.device.telemetry_since(baseline.sequence)
            if sample.sequence <= post_sequence
        )
        for sample in observed:
            if sample.missed is None or sample.nonfinite is None:
                raise UserError(
                    "{} telemetry line {} omitted a cumulative missed or "
                    "nonfinite counter between the health baseline and required "
                    "post-action sample: {}".format(
                        label, sample.sequence, sample.line.strip()
                    )
                )
            for field in ("missed", "nonfinite"):
                initial = getattr(baseline, field)
                value = getattr(sample, field)
                if value != initial:
                    raise UserError(
                        "{} changed cumulative {} telemetry from baseline {} to {} "
                        "at line {}; every observed value through the required "
                        "post-action Running sample must remain exactly equal to "
                        "baseline".format(
                            label,
                            field,
                            initial,
                            value,
                            sample.sequence,
                        )
                    )
        print(
            "[accept] {} health stable: missed={} nonfinite={}"
            .format(label, baseline.missed, baseline.nonfinite)
        )


    def check_launcher(self):
        self._boot_to_launcher()
        self._send_wait_exact("ping", "@DEV PONG")
        print("[accept] launcher check confirmed legacy ping response")
        self._probe_mode("launcher", "launcher check")
        self.launcher_frame = self._capture("launcher")

    def check_touch(self):
        self._ensure_launcher()
        self._probe_mode("launcher", "touch check precondition")

        def parse_touch_launch(stripped: str) -> tuple[int, int]:
            match = TOUCH_LAUNCH_LOG.search(stripped)
            if match is None:
                raise UserError(
                    "touch check emitted a malformed or non-ESP touch-launch log; "
                    "expected an ESP log under the fluid_demo tag containing exact "
                    "payload 'touch launch x=<uint> y=<uint>': {}"
                    .format(stripped)
                )

            x, y = (int(value) for value in match.groups())
            if not (56 <= x < PANEL_WIDTH and 56 <= y < 184):
                raise UserError(
                    "touch check logged coordinates x={} y={} outside the launcher "
                    "target x=[56,240), y=[56,184); out-of-panel and HOME touches "
                    "must not launch".format(x, y)
                )
            return x, y

        def run_touch_round(round_number: int) -> FrameEvent:
            label = "touch round {}".format(round_number)
            checkpoint = self.device.checkpoint()
            print(
                "[accept] ACTION: {} tap and release the centered Fluid Box entry "
                "on the touchscreen"
                .format("first" if round_number == 1 else "second")
            )

            touch_seen = False
            touch_coordinates = None

            def observe_touch_transition(line: str) -> bool:
                nonlocal touch_seen, touch_coordinates
                stripped = line.strip()

                if stripped == MODE_FLUID:
                    if not touch_seen:
                        raise UserError(
                            "{} reached fluid_box without a fresh "
                            "'touch launch x=<uint> y=<uint>' log after the "
                            "operator checkpoint".format(label)
                        )
                    return True

                if stripped == MODE_LAUNCHER:
                    if touch_seen:
                        raise UserError(
                            "{} emitted a launcher mode marker after the accepted "
                            "touch log instead of completing the Fluid launch"
                            .format(label)
                        )
                    return False

                if stripped.startswith("@DEV MODE "):
                    raise UserError(
                        "{} emitted unexpected mode marker {!r}"
                        .format(label, stripped)
                    )

                if "touch launch" not in stripped:
                    return False

                coordinates = parse_touch_launch(stripped)
                if touch_seen:
                    raise UserError(
                        "{} emitted more than one fresh touch-launch log before "
                        "completing the Fluid launch; one tap must produce one "
                        "logical event".format(label)
                    )
                touch_seen = True
                touch_coordinates = coordinates
                return False

            try:
                fluid_sequence, _ = self.device.wait_line(
                    observe_touch_transition,
                    checkpoint,
                    self.args.touch_timeout,
                    "an ordered touch-launch log followed by {!r} for {}"
                    .format(MODE_FLUID, label),
                )
            except AcceptanceTimeout as exc:
                if not touch_seen:
                    raise UserError(
                        "{} timed out after {:.1f}s without a fresh accepted "
                        "touch-launch log followed by {!r}"
                        .format(label, self.args.touch_timeout, MODE_FLUID)
                    ) from exc
                x, y = touch_coordinates
                raise UserError(
                    "{} logged x={} y={} but did not subsequently emit {!r} "
                    "within {:.1f}s; the touch did not launch Fluid"
                    .format(label, x, y, MODE_FLUID, self.args.touch_timeout)
                ) from exc
            except DeviceGone as exc:
                raise UserError(
                    "{} lost the serial transport before the ordered touch launch "
                    "completed; disconnect/reconnect is not accepted".format(label)
                ) from exc

            x, y = touch_coordinates
            self.mode = "fluid_box"
            print(
                "[accept] {} launch accepted at x={} y={}"
                .format(label, x, y)
            )
            try:
                fluid_frame = self._capture(
                    "fluid-after-touch-{}".format(round_number)
                )
            except DeviceGone as exc:
                raise UserError(
                    "{} lost the serial transport during the Fluid capture; "
                    "disconnect/reconnect is not accepted".format(label)
                ) from exc

            cleanup_checkpoint = self.device.checkpoint()
            pre_cleanup_description = "the Fluid capture before {} cleanup".format(
                label
            )
            for sequence, line in self.device.lines_since(fluid_sequence):
                if sequence > cleanup_checkpoint:
                    break
                self.device._raise_if_command_rejected(
                    line,
                    pre_cleanup_description,
                )
                stripped = line.strip()
                if stripped == MODE_LAUNCHER:
                    raise UserError(
                        "{} returned to Launcher after the accepted launch and "
                        "before the deliberate cleanup Home".format(label)
                    )
                if "touch launch" in stripped:
                    raise UserError(
                        "{} emitted another fresh touch-launch log after the "
                        "accepted launch and before deliberate cleanup: {}"
                        .format(label, stripped)
                    )
                if stripped.startswith("@DEV MODE ") and stripped != MODE_FLUID:
                    raise UserError(
                        "{} emitted unexpected mode marker {!r} before deliberate "
                        "cleanup".format(label, stripped)
                    )

            self.device.echo_send("input pwr 120")

            def observe_cleanup_home(line: str) -> bool:
                stripped = line.strip()
                if stripped == MODE_LAUNCHER:
                    return True
                if "touch launch" in stripped:
                    raise UserError(
                        "{} emitted another fresh touch-launch log while waiting "
                        "for deliberate cleanup Home: {}".format(label, stripped)
                    )
                if stripped == MODE_FLUID:
                    raise UserError(
                        "{} emitted fluid_box again while waiting for deliberate "
                        "cleanup Home".format(label)
                    )
                if stripped.startswith("@DEV MODE "):
                    raise UserError(
                        "{} emitted unexpected mode marker {!r} while waiting for "
                        "deliberate cleanup Home".format(label, stripped)
                    )
                return False

            try:
                home_sequence, _ = self.device.wait_line(
                    observe_cleanup_home,
                    cleanup_checkpoint,
                    self.args.timeout,
                    "the exact cleanup Home marker {!r} for {}"
                    .format(MODE_LAUNCHER, label),
                )
            except DeviceGone as exc:
                raise UserError(
                    "{} lost the serial transport during cleanup Home; "
                    "disconnect/reconnect is not accepted".format(label)
                ) from exc

            self.mode = "launcher"

            def reject_post_home_line(line: str):
                stripped = line.strip()
                if "touch launch" in stripped:
                    raise UserError(
                        "{} emitted a fresh touch-launch log after cleanup Home; "
                        "the tap was not observably released: {}"
                        .format(label, stripped)
                    )
                if stripped == MODE_FLUID:
                    raise UserError(
                        "{} re-entered fluid_box after cleanup Home; the tap was "
                        "not observably released".format(label)
                    )
                if stripped.startswith("@DEV MODE ") and stripped != MODE_LAUNCHER:
                    raise UserError(
                        "{} emitted unexpected mode marker {!r} after cleanup Home"
                        .format(label, stripped)
                    )

            try:
                self.device.wait_line(
                    lambda line: (reject_post_home_line(line), False)[1],
                    home_sequence,
                    TOUCH_RELEASE_QUIET_SECONDS,
                    "the bounded touch-release quiet window for {}".format(label),
                )
            except AcceptanceTimeout:
                pass
            except DeviceGone as exc:
                raise UserError(
                    "{} lost the serial transport during the post-Home "
                    "touch-release window; disconnect/reconnect is not accepted"
                    .format(label)
                ) from exc

            quiet_end = self.device.checkpoint()
            post_home_description = "the touch release window after {} cleanup".format(
                label
            )

            def scan_post_home(through: int):
                for sequence, line in self.device.lines_since(home_sequence):
                    if sequence > through:
                        break
                    self.device._raise_if_command_rejected(
                        line,
                        post_home_description,
                    )
                    reject_post_home_line(line)

            scan_post_home(quiet_end)
            try:
                status = self._request_status(
                    "{} post-release cleanup".format(label),
                    self.args.timeout,
                )
            except DeviceGone as exc:
                raise UserError(
                    "{} lost the serial transport before the final launcher "
                    "STATUS; disconnect/reconnect is not accepted".format(label)
                ) from exc

            scan_post_home(self.device.checkpoint())
            if status.mode != "launcher":
                raise UserError(
                    "{} final STATUS reported mode={}; expected launcher"
                    .format(label, status.mode)
                )
            self.mode = "launcher"
            print(
                "[accept] {} release stable in launcher for {:.2f}s with a fresh "
                "strict STATUS".format(label, TOUCH_RELEASE_QUIET_SECONDS)
            )
            return fluid_frame

        run_touch_round(1)
        self.fluid_frame = run_touch_round(2)

    def check_apps(self):
        self._boot_to_launcher()
        self._probe_mode("launcher", "apps check launcher")
        self._send_wait_exact("reset", "reset: no running app")

        apps = (
            (1, "Task Maze", "tilt_maze", MODE_MAZE),
            (2, "Avalanche", "ragdoll_avalanche", MODE_AVALANCHE),
            (3, "Cube", "orient_cube", MODE_CUBE),
            (4, "Level", "attitude", MODE_LEVEL),
        )
        for selection, label, mode, marker in apps:
            checkpoint = self.device.checkpoint()
            print(
                "[accept] ACTION: swipe left once to select {}".format(label)
            )
            expected_selection = (
                "swipe left - launcher selection {}: {}"
                .format(selection, label)
            )
            self.device.wait_line(
                lambda line, expected=expected_selection: expected in line,
                checkpoint,
                self.args.touch_timeout,
                "a physical left swipe selecting {}".format(label),
            )

            self._transition("input plus", marker)
            self._probe_mode(mode, "{} launch".format(label))
            baseline_checkpoint = self.device.checkpoint()
            baseline = self._wait_epoch(
                baseline_checkpoint,
                lambda sample: sample.epoch is not None,
                "{} epoch after launch".format(label),
            )

            reset_checkpoint = self.device.checkpoint()
            self._send_wait_exact("reset", "@DEV RESET_REQUESTED")
            reset = self._wait_epoch(
                reset_checkpoint,
                lambda sample, initial=baseline.epoch: (
                    sample.epoch is not None and sample.epoch != initial
                ),
                "{} epoch after reset".format(label),
            )
            if reset.epoch != baseline.epoch + 1:
                raise UserError(
                    "{} reset changed epoch from {} to {}; expected one increment"
                    .format(label, baseline.epoch, reset.epoch)
                )

            self._capture("{}-after-reset".format(mode))
            self._transition("input pwr 120", MODE_LAUNCHER)
            self._probe_mode("launcher", "{} home".format(label))

        checkpoint = self.device.checkpoint()
        self.device.echo_send("input swipe-left")
        expected_selection = "swipe left - launcher selection 0: Fluid Box"
        self.device.wait_line(
            lambda line: expected_selection in line,
            checkpoint,
            self.args.timeout,
            "launcher selection restored to Fluid Box after app checks",
        )


    def check_once(self):
        self._ensure_launcher()

        self._transition("input plus", MODE_FLUID)
        self._probe_mode("fluid_box", "launch/home health warm-up")
        health = self._health_baseline(
            self.device.checkpoint(), "launch/home"
        )
        self._transition("input pwr 120", MODE_LAUNCHER)
        launcher = self.launcher_frame or self._capture("launcher-before-once")

        self._transition("input plus", MODE_FLUID)
        self._probe_mode("fluid_box", "launch")
        fluid = self._capture("fluid-after-launch")
        self._assert_distinct(launcher, fluid, "launch")
        action_end = self.device.checkpoint()
        self._assert_health(health, action_end, "launch/home")

        self._transition("input pwr 120", MODE_LAUNCHER)
        returned = self._capture("launcher-after-home")
        self._assert_distinct(fluid, returned, "home")
        self._probe_mode("launcher", "home")
        self.launcher_frame = returned
        self.fluid_frame = fluid

    def check_cycles(self):
        self._ensure_launcher()
        baseline_epoch = None

        self._transition("input plus", MODE_FLUID)
        self._probe_mode("fluid_box", "transition-cycle health warm-up")
        health = self._health_baseline(
            self.device.checkpoint(),
            "{} transition cycles".format(self.args.cycles),
        )
        self._transition("input pwr 120", MODE_LAUNCHER)

        for cycle in range(1, self.args.cycles + 1):
            marker_sequence = self._transition("input plus", MODE_FLUID)
            if cycle == 1:
                baseline_epoch = self._wait_epoch(
                    marker_sequence,
                    lambda sample: sample.epoch is not None,
                    "reset_epoch telemetry on the first transition cycle",
                )
            if cycle == self.args.cycles:
                final_epoch = self._wait_epoch(
                    marker_sequence,
                    lambda sample: sample.epoch is not None,
                    "reset_epoch telemetry on the final transition cycle",
                )
                if final_epoch.epoch != baseline_epoch.epoch:
                    raise UserError(
                        "{} plain relaunch cycles changed reset_epoch from {} to {}"
                        .format(self.args.cycles, baseline_epoch.epoch, final_epoch.epoch)
                    )
                action_end = self.device.checkpoint()
                self._assert_health(health, action_end, "transition cycles")
            self._transition("input pwr 120", MODE_LAUNCHER)
            if cycle == 1 or cycle % 10 == 0 or cycle == self.args.cycles:
                print("[accept] transition cycle {}/{}".format(cycle, self.args.cycles))

    def check_freeze(self):
        self._ensure_fluid()
        entry_sequence = self.device.checkpoint()
        initial = self._wait_epoch(
            entry_sequence,
            lambda sample: sample.epoch is not None,
            "Fluid reset_epoch telemetry",
        )
        health = self._health_baseline(entry_sequence, "freeze/resume")

        self._send_wait_prefix("motion 0 0 6 0", "@DEV MOTION ")
        reset_checkpoint = self.device.checkpoint()
        self._send_wait_exact("reset", "@DEV RESET_REQUESTED")
        reset_sample = self._wait_epoch(
            reset_checkpoint,
            lambda sample: sample.epoch is not None and sample.epoch != initial.epoch,
            "reset_epoch increment after explicit reset",
        )
        if reset_sample.epoch != initial.epoch + 1:
            raise UserError(
                "explicit reset changed reset_epoch from {} to {}; expected exactly one increment"
                .format(initial.epoch, reset_sample.epoch)
            )
        time.sleep(self.args.reset_settle_seconds)
        reset_frame = self._capture("fluid-after-explicit-reset")

        self._send_wait_prefix("motion -6 0 0 0", "@DEV MOTION ")
        time.sleep(self.args.drive_seconds)
        calibration_frames = [
            self._capture("fluid-drift-calibration-{}".format(index))
            for index in range(1, FREEZE_DRIFT_CALIBRATION_FRAMES + 1)
        ]
        evolved = calibration_frames[-1]
        if frame_signature(reset_frame) == frame_signature(evolved):
            raise UserError(
                "deterministic motion did not change the Fluid framebuffer; "
                "freeze/resume cannot be observed"
            )

        calibration_drifts = [
            mean_frame_distance(left, right)
            for left, right in zip(calibration_frames, calibration_frames[1:])
        ]
        normal_drift_min = min(calibration_drifts)
        normal_drift_max = max(calibration_drifts)
        freeze_drift_tolerance = normal_drift_max
        print(
            "[accept] normal capture drift: min={:.6f} max={:.6f} "
            "absolute resume tolerance={:.6f}"
            .format(
                normal_drift_min,
                normal_drift_max,
                freeze_drift_tolerance,
            )
        )

        self._transition("input pwr 120", MODE_LAUNCHER)
        launcher = self._capture("launcher-during-freeze")
        self._assert_distinct(launcher, evolved, "home during freeze")
        time.sleep(self.args.freeze_seconds)

        relaunch_marker = self._transition("input plus", MODE_FLUID)
        resumed = self._capture("fluid-after-resume")
        resumed_sample = self._wait_epoch(
            relaunch_marker,
            lambda sample: sample.epoch is not None,
            "reset_epoch telemetry after plain relaunch",
        )
        if resumed_sample.epoch != reset_sample.epoch:
            raise UserError(
                "plain relaunch changed reset_epoch from {} to {}; Fluid was reset "
                "instead of resumed"
                .format(reset_sample.epoch, resumed_sample.epoch)
            )

        distance_to_frozen = mean_frame_distance(evolved, resumed)
        distance_to_reset = mean_frame_distance(reset_frame, resumed)
        print(
            "[accept] resume distances: frozen={:.6f} reset={:.6f} "
            "absolute tolerance={:.6f}"
            .format(
                distance_to_frozen,
                distance_to_reset,
                freeze_drift_tolerance,
            )
        )
        self._send_wait_exact("release", "@DEV MOTION_RELEASED")
        if distance_to_frozen > freeze_drift_tolerance:
            raise UserError(
                "resumed Fluid drifted {:.6f} from the pre-home state, exceeding "
                "the calibrated absolute capture-drift tolerance {:.6f}"
                .format(distance_to_frozen, freeze_drift_tolerance)
            )
        if distance_to_frozen >= distance_to_reset:
            raise UserError(
                "resumed Fluid is not closer to the pre-home state than to the "
                "reset state (frozen={:.6f}, reset={:.6f})"
                .format(distance_to_frozen, distance_to_reset)
            )

        plus_checkpoint = self.device.checkpoint()
        self.device.echo_send("input plus")
        plus_sample = self._wait_epoch(
            plus_checkpoint,
            lambda sample: sample.epoch is not None and sample.epoch != resumed_sample.epoch,
            "reset_epoch increment after PLUS in Fluid",
        )
        if plus_sample.epoch != resumed_sample.epoch + 1:
            raise UserError(
                "PLUS in Fluid changed reset_epoch from {} to {}; expected one increment"
                .format(resumed_sample.epoch, plus_sample.epoch)
            )

        action_end = self.device.checkpoint()
        self._assert_health(health, action_end, "freeze/resume")
        self._transition("input pwr 120", MODE_LAUNCHER)
        self.launcher_frame = launcher
        self.fluid_frame = resumed

    def check_reboot(self):
        self._boot_to_launcher()
        self._legacy_reboot_to_launcher()
        frame = self._capture("launcher-after-reboot-reconnect")
        if self.fluid_frame is not None:
            self._assert_distinct(frame, self.fluid_frame, "reboot final launcher")
        self.launcher_frame = frame

    def check_soak(self):
        if self.mode != "fluid_box":
            self._ensure_launcher()
            soak_entry = self._transition("input plus", MODE_FLUID)
        else:
            soak_entry = self.device.checkpoint()

        label = "bounded soak"

        def reject_forbidden_line(line: str):
            self.device._raise_if_command_rejected(line, label)
            stripped = line.strip()
            if stripped == MODE_LAUNCHER:
                raise UserError(
                    "{} returned to Launcher before deliberate cleanup Home"
                    .format(label)
                )
            if stripped == "@DEV REBOOTING":
                raise UserError(
                    "{} rebooted before deliberate cleanup Home".format(label)
                )
            if stripped == "@DEV POWEROFF":
                raise UserError(
                    "{} powered off before deliberate cleanup Home".format(label)
                )
            if stripped == MODE_FLUID:
                raise UserError(
                    "{} emitted a duplicate fluid_box mode marker; Fluid "
                    "transitioned or re-entered after the accepted soak entry"
                    .format(label)
                )
            if stripped.startswith("@DEV MODE"):
                raise UserError(
                    "{} emitted malformed or unknown mode marker {!r}"
                    .format(label, stripped)
                )

        def observe_complete_telemetry(line: str) -> bool:
            reject_forbidden_line(line)
            sample = parse_telemetry(0, line)
            return (
                sample is not None
                and sample.missed is not None
                and sample.nonfinite is not None
            )

        def wait_complete_telemetry(
            after: int,
            timeout: float,
            description: str,
        ) -> tuple[int, str]:
            return self.device.wait_line(
                observe_complete_telemetry,
                after,
                timeout,
                description,
            )

        def validate_telemetry_health(sequence: int, line: str):
            sample = parse_telemetry(sequence, line)
            if sample is None:
                return
            if sample.missed is None or sample.nonfinite is None:
                raise UserError(
                    "{} telemetry line {} omitted a cumulative missed or "
                    "nonfinite counter after the health baseline: {}"
                    .format(label, sequence, line.strip())
                )
            for field in ("missed", "nonfinite"):
                initial = getattr(health, field)
                value = getattr(sample, field)
                if value != initial:
                    raise UserError(
                        "{} changed cumulative {} telemetry from baseline {} to "
                        "{} at line {}; every observed value through cleanup Home "
                        "must remain exactly equal to baseline"
                        .format(label, field, initial, value, sequence)
                    )

        def scan_through(after: int, through: int) -> int:
            cursor = after
            for sequence, line in self.device.lines_since(after):
                if sequence > through:
                    break
                reject_forbidden_line(line)
                if sequence > health.sequence:
                    validate_telemetry_health(sequence, line)
                cursor = sequence
            return cursor

        entry_checkpoint = soak_entry
        baseline_sequence, _ = wait_complete_telemetry(
            entry_checkpoint,
            self.args.telemetry_timeout,
            (
                "one passive Running-mode telemetry line containing both "
                "cumulative missed and nonfinite counters for {}"
            ).format(label),
        )
        health = self._health_baseline(entry_checkpoint, label)
        cursor = scan_through(entry_checkpoint, baseline_sequence)

        start_frame = self._capture("fluid-soak-start")
        cursor = scan_through(cursor, self.device.checkpoint())
        deadline = time.monotonic() + self.args.soak_seconds

        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            wait_timeout = min(self.args.telemetry_timeout, remaining)
            try:
                telemetry_sequence, _ = wait_complete_telemetry(
                    cursor,
                    wait_timeout,
                    (
                        "continuous complete Running-mode telemetry during the "
                        "{} passive interval"
                    ).format(label),
                )
            except AcceptanceTimeout:
                cursor = scan_through(cursor, self.device.checkpoint())
                if time.monotonic() >= deadline:
                    break
                raise
            cursor = scan_through(cursor, telemetry_sequence)

        end_frame = self._capture("fluid-soak-end")
        self._assert_frame_shape(start_frame, "fluid-soak-start")
        self._assert_frame_shape(end_frame, "fluid-soak-end")
        action_end = self.device.checkpoint()
        cursor = scan_through(cursor, action_end)

        post_sequence, _ = wait_complete_telemetry(
            action_end,
            self.args.telemetry_timeout,
            (
                "a post-end-capture complete Running-mode telemetry line for {}"
            ).format(label),
        )
        cursor = scan_through(cursor, post_sequence)
        self._assert_health(health, action_end, label)

        home_checkpoint = self.device.checkpoint()
        cursor = scan_through(cursor, home_checkpoint)
        self.device.echo_send("input pwr 120")

        def accept_cleanup_launcher(line: str) -> bool:
            self.device._raise_if_command_rejected(
                line, "the deliberate cleanup Home for {}".format(label)
            )
            stripped = line.strip()
            if stripped == MODE_LAUNCHER:
                return True
            if stripped == "@DEV REBOOTING":
                raise UserError(
                    "{} rebooted while waiting for deliberate cleanup Home"
                    .format(label)
                )
            if stripped == "@DEV POWEROFF":
                raise UserError(
                    "{} powered off while waiting for deliberate cleanup Home"
                    .format(label)
                )
            if stripped == MODE_FLUID:
                raise UserError(
                    "{} emitted fluid_box while waiting for deliberate cleanup "
                    "Home".format(label)
                )
            if stripped.startswith("@DEV MODE"):
                raise UserError(
                    "{} emitted malformed or unknown mode marker {!r} while "
                    "waiting for deliberate cleanup Home".format(label, stripped)
                )
            validate_telemetry_health(0, line)
            return False

        self.device.wait_line(
            accept_cleanup_launcher,
            home_checkpoint,
            self.args.timeout,
            "the exact cleanup Home marker {!r} for {}".format(
                MODE_LAUNCHER, label
            ),
        )
        self.mode = "launcher"

    def check_power_off(self):
        self._ensure_fluid()
        minimum_absence = self.args.power_off_min_absence
        if minimum_absence >= self.args.power_off_timeout:
            raise UserError(
                "--power-off-min-absence must be less than --power-off-timeout"
            )
        checkpoint = self.device.checkpoint()
        self.device.echo_send("input pwr 3000")
        deadline = time.monotonic() + self.args.power_off_timeout
        poweroff_marker_seen = False
        marker_seen_at = None
        absent_since = None

        def reject_protocol_activity(device):
            nonlocal poweroff_marker_seen, marker_seen_at
            for _, line in device.lines_since(checkpoint):
                device._raise_if_command_rejected(
                    line,
                    "battery-latch release after long PWR input",
                )
                stripped = line.strip()
                if stripped == "@DEV POWEROFF":
                    poweroff_marker_seen = True
                    if marker_seen_at is None:
                        marker_seen_at = time.monotonic()
                elif stripped == MODE_LAUNCHER:
                    raise UserError(
                        "long PWR input emitted launcher mode; the hold was also "
                        "misclassified as a short press"
                    )

        def close_and_scan_device():
            device = self.device
            device.close()
            reject_protocol_activity(device)
            self.device = None

        while time.monotonic() < deadline:
            if self.device is not None:
                reject_protocol_activity(self.device)
                transport_lost = (
                    not self.device.reader.is_alive()
                    or (
                        self._identity_was_enumerated
                        and not self.port_identity.present_devices()
                    )
                )
                if transport_lost:
                    close_and_scan_device()

            if self.device is None:
                present = self.port_identity.present_devices()
                now = time.monotonic()
                if not present:
                    if absent_since is None:
                        absent_since = now
                elif absent_since is not None:
                    raise UserError(
                        "serial device re-enumerated as {} after {:.2f}s absent "
                        "following long PWR input; the hold rebooted the board "
                        "instead of releasing only the battery latch"
                        .format(", ".join(present), now - absent_since)
                    )
            elif (
                marker_seen_at is not None
                and time.monotonic() - marker_seen_at
                >= POWER_OFF_POST_MARKER_RELEASE_SECONDS
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                try:
                    status = self._request_status(
                        "post-release persistent-USB proof",
                        min(STATUS_RETRY_SECONDS, remaining),
                    )
                except AcceptanceTimeout as exc:
                    reject_protocol_activity(self.device)
                    raise UserError(
                        "long PWR input kept USB connected but did not return one "
                        "fresh STATUS after the synthetic release/debounce window"
                    ) from exc
                except DeviceGone:
                    close_and_scan_device()
                    continue

                reject_protocol_activity(self.device)
                transport_lost = (
                    not self.device.reader.is_alive()
                    or (
                        self._identity_was_enumerated
                        and not self.port_identity.present_devices()
                    )
                )
                if transport_lost:
                    close_and_scan_device()
                    continue
                if status.mode != "fluid_box":
                    raise UserError(
                        "long PWR post-release STATUS reported mode={}; expected "
                        "fluid_box with no short-PWR home transition"
                        .format(status.mode)
                    )
                if status.battery_hold != 0:
                    raise UserError(
                        "long PWR post-release STATUS reported battery_hold={}; "
                        "expected 0 after releasing the battery latch"
                        .format(status.battery_hold)
                    )
                self.mode = status.mode
                print(
                    "[accept] power-off released the battery latch "
                    "(battery_hold=0) while USB remained connected in fluid_box"
                )
                return

            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))

        if self.device is not None:
            reject_protocol_activity(self.device)
            if self.device.reader.is_alive():
                if not poweroff_marker_seen:
                    raise UserError(
                        "long PWR input did not emit the required post-command "
                        "@DEV POWEROFF marker"
                    )
                raise UserError(
                    "long PWR input kept USB connected but the timeout ended before "
                    "a fresh post-release STATUS proved battery_hold=0 and "
                    "mode=fluid_box"
                )
            close_and_scan_device()

        present = self.port_identity.present_devices()
        observed_at = time.monotonic()
        if present:
            if absent_since is not None:
                raise UserError(
                    "serial device re-enumerated as {} after {:.2f}s absent "
                    "following long PWR input; the hold rebooted the board instead "
                    "of releasing only the battery latch"
                    .format(", ".join(present), observed_at - absent_since)
                )
            raise UserError(
                "serial reader stopped, but the target device never remained "
                "absent during the {:.1f}s power-off window"
                .format(self.args.power_off_timeout)
            )
        if absent_since is None:
            absent_since = observed_at
        continuous_absence = observed_at - absent_since
        if continuous_absence < minimum_absence:
            raise UserError(
                "target was continuously absent for only {:.2f}s before the "
                "power-off deadline; require at least {:.2f}s to distinguish "
                "power-off from a late reboot"
                .format(continuous_absence, minimum_absence)
            )
        if not poweroff_marker_seen:
            raise UserError(
                "long PWR input did not emit the required post-command "
                "@DEV POWEROFF marker before sustained transport absence"
            )
        print(
            "[accept] power-off observed @DEV POWEROFF, then kept the USB serial "
            "transport continuously absent for {:.2f}s without a short event or "
            "re-enumeration; no persistent-USB STATUS proof was needed"
            .format(continuous_absence)
        )


def build_arg_parser():
    parser = argparse.ArgumentParser(
        prog="firmware_shell_acceptance.py",
        description=(
            "Run host-observable firmware-shell acceptance checks over the existing "
            "device_dev serial reader and @FB parser. With no CHECK names, runs the "
            "core launcher/transition/freeze/reboot checks."
        ),
    )
    parser.add_argument(
        "checks",
        nargs="*",
        choices=CHECK_CHOICES + ("all",),
        metavar="CHECK",
        help=(
            "subset: launcher once cycles freeze reboot soak touch apps all "
            "(touch and apps are interactive and excluded from all)"
        ),
    )
    parser.add_argument("--port", default=None, metavar="DEV")
    parser.add_argument("--baud", type=positive_int, default=device_dev.DEFAULT_BAUD)
    parser.add_argument("--out-dir", default=device_dev.DEFAULT_OUT_DIR, metavar="DIR")
    parser.add_argument("--cycles", type=positive_int, default=DEFAULT_CYCLES, metavar="N")
    parser.add_argument("--timeout", type=positive_finite, default=DEFAULT_TIMEOUT, metavar="SEC")
    parser.add_argument(
        "--capture-timeout",
        type=positive_finite,
        default=device_dev.DEFAULT_CAPTURE_TIMEOUT,
        metavar="SEC",
    )
    parser.add_argument(
        "--telemetry-timeout",
        type=positive_finite,
        default=DEFAULT_TELEMETRY_TIMEOUT,
        metavar="SEC",
    )
    parser.add_argument(
        "--reconnect-timeout",
        type=positive_finite,
        default=DEFAULT_RECONNECT_TIMEOUT,
        metavar="SEC",
    )
    parser.add_argument(
        "--touch-timeout",
        type=positive_finite,
        default=DEFAULT_TOUCH_TIMEOUT,
        metavar="SEC",
        help=(
            "bounded operator tap-and-release observation window for each of two "
            "opt-in touch rounds; each post-Home release quiet is fixed at "
            "{:.2f}s (default %(default)s)"
            .format(TOUCH_RELEASE_QUIET_SECONDS)
        ),
    )
    parser.add_argument(
        "--freeze-seconds",
        type=positive_finite,
        default=2.0,
        metavar="SEC",
        help="bounded time spent at home before Fluid relaunch (default %(default)s)",
    )
    parser.add_argument(
        "--reset-settle-seconds",
        type=positive_finite,
        default=0.5,
        metavar="SEC",
    )
    parser.add_argument(
        "--drive-seconds",
        type=positive_finite,
        default=1.5,
        metavar="SEC",
    )
    parser.add_argument(
        "--soak-seconds",
        type=positive_finite,
        default=DEFAULT_SOAK_SECONDS,
        metavar="SEC",
        help="bounded Fluid soak duration when the soak check is selected (default %(default)s)",
    )
    parser.add_argument(
        "--power-off",
        action="store_true",
        help=(
            "run the destructive long-PWR battery-latch release check last; "
            "USB-powered targets may remain serially connected"
        ),
    )
    parser.add_argument(
        "--power-off-timeout",
        type=positive_finite,
        default=10.0,
        metavar="SEC",
        help=(
            "bounded window for sustained USB disappearance or a fresh "
            "post-release STATUS proof (default %(default)s)"
        ),
    )
    parser.add_argument(
        "--power-off-min-absence",
        type=positive_finite,
        default=DEFAULT_POWER_OFF_MIN_ABSENCE,
        metavar="SEC",
        help=(
            "minimum continuous target absence required by the USB-disappearance "
            "branch (default %(default)s)"
        ),
    )
    return parser


def selected_checks(args) -> tuple[str, ...]:
    requested = tuple(args.checks) if args.checks else CORE_CHECKS
    if "all" in requested:
        requested = ALL_CHECKS
    ordered = tuple(check for check in CHECK_CHOICES if check in requested)
    if not ordered:
        raise UserError("select at least one acceptance check")
    return ordered


def main(argv=None) -> int:
    args = build_arg_parser().parse_args(argv)
    if device_dev.serial is None:
        print(
            "firmware_shell_acceptance: error: pyserial is required; activate the "
            "project environment or install pyserial",
            file=sys.stderr,
        )
        return 2

    harness = None
    try:
        checks = selected_checks(args)
        harness = FirmwareShellAcceptance(args)
        for check in checks:
            print("[accept] START {}".format(check))
            getattr(harness, "check_{}".format(check))()
            print("[accept] PASS {}".format(check))
        if args.power_off:
            print("[accept] START power-off")
            harness.check_power_off()
            print("[accept] PASS power-off")
        print("[accept] PASS firmware shell acceptance")
        return 0
    except UserError as exc:
        print("firmware_shell_acceptance: error: {}".format(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("firmware_shell_acceptance: interrupted", file=sys.stderr)
        return 130
    finally:
        if harness is not None:
            harness.close()


if __name__ == "__main__":
    sys.exit(main())
