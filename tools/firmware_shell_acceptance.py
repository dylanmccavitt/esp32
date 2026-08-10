#!/usr/bin/env python3
"""Hardware acceptance bridge for the development-only firmware shell protocol.

This executable deliberately exercises the board through the same serial protocol
used by tools/device_dev.py.  It is expected to fail against firmware that does
not yet implement the ``input`` commands and completed-transition MODE markers.
"""

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
DEFAULT_POWER_OFF_MIN_ABSENCE = 2.0
STATUS_RETRY_SECONDS = 1.0
FREEZE_DRIFT_CALIBRATION_FRAMES = 3
PORT_POLL_SECONDS = 0.05

MODE_LAUNCHER = "@DEV MODE launcher"
MODE_FLUID = "@DEV MODE fluid_box"
CORE_CHECKS = ("launcher", "once", "cycles", "freeze", "reboot")
ALL_CHECKS = CORE_CHECKS + ("soak",)
COMMAND_FAILURE_MARKERS = (
    "unrecognized command",
    "unknown command",
    "command not found",
    "invalid command",
)
TELEMETRY_FIELD = re.compile(r"\b(reset_epoch|epoch|missed|nonfinite)=(\d+)\b")
STATUS_PREFIX = "@DEV STATUS "
STATUS_VALUE_PATTERNS = {
    "uptime_ms": re.compile(r"\d+"),
    "override": re.compile(r"[01]"),
    "accel": re.compile(r"-?\d+\.\d{3},-?\d+\.\d{3},-?\d+\.\d{3}"),
    "capture_ready": re.compile(r"[01]"),
    "mode": re.compile(r"launcher|fluid_box"),
}
STATUS_VALUE_DESCRIPTIONS = {
    "uptime_ms": "an unsigned decimal integer",
    "override": "0 or 1",
    "accel": "three comma-separated fixed-point values with three decimals",
    "capture_ready": "0 or 1",
    "mode": "launcher or fluid_box",
}


class AcceptanceTimeout(UserError):
    """A bounded acceptance observation did not arrive."""


@dataclasses.dataclass(frozen=True)
class Telemetry:
    sequence: int
    epoch: int | None
    missed: int | None
    nonfinite: int | None
    line: str


@dataclasses.dataclass(frozen=True)
class HealthBaseline:
    missed: Telemetry | None
    nonfinite: Telemetry | None


@dataclasses.dataclass(frozen=True)
class SerialPortIdentity:
    device: str
    serial_number: str | None
    vid: int | None
    pid: int | None
    location: str | None

    @classmethod
    def capture(cls, device: str) -> "SerialPortIdentity":
        if list_ports is not None:
            for info in list_ports.comports():
                if info.device == device:
                    return cls(
                        device=device,
                        serial_number=info.serial_number,
                        vid=info.vid,
                        pid=info.pid,
                        location=info.location,
                    )
        return cls(device, None, None, None, None)

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
            has_enumerated_metadata = any(
                value is not None
                for value in (
                    self.serial_number,
                    self.vid,
                    self.pid,
                    self.location,
                )
            )
            if has_enumerated_metadata:
                return ()
        if os.path.exists(self.device):
            return (self.device,)
        return ()


class AcceptanceDevice(Device):
    """Device observer that records non-frame lines from Device.emit().

    ReaderThread continues to own serial reads and FrameParser continues to own
    @FB decoding.  This subclass only observes Device's existing emit funnel so
    acceptance waits cannot race a second serial consumer.
    """

    def __init__(self, *args, **kwargs):
        self._line_condition = threading.Condition()
        self._line_sequence = 0
        self._lines = collections.deque(maxlen=8192)
        self._telemetry = collections.deque()
        super().__init__(*args, **kwargs)

    def emit(self, text: bytes | str):
        decoded = text.decode("utf-8", "replace") if isinstance(text, bytes) else text
        with self._line_condition:
            for raw in decoded.splitlines():
                line = raw.rstrip("\r")
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
                pending = [(seq, line) for seq, line in self._lines if seq > cursor]
                if pending:
                    cursor = pending[-1][0]
                for sequence, line in pending:
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
            return [(seq, line) for seq, line in self._lines if seq > after]

    def scan_command_failures(self, after: int, description: str) -> int:
        """Reject command failures observed after a checkpoint and return a cursor."""
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


def validate_status(line: str, label: str) -> str:
    """Validate the preserved STATUS contract and return its single mode."""
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

    return observed["mode"][0]


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


def frame_distance(left: FrameEvent, right: FrameEvent) -> float:
    """Mean normalized RGB565 channel distance, in the range 0..1."""
    if (left.width, left.height, len(left.data)) != (
        right.width,
        right.height,
        len(right.data),
    ):
        raise UserError("cannot compare framebuffer signatures with different shapes")

    total = 0
    for offset in range(0, len(left.data), 2):
        lhs = (left.data[offset] << 8) | left.data[offset + 1]
        rhs = (right.data[offset] << 8) | right.data[offset + 1]
        total += abs((lhs >> 11) - (rhs >> 11))
        total += abs(((lhs >> 5) & 0x3F) - ((rhs >> 5) & 0x3F))
        total += abs((lhs & 0x1F) - (rhs & 0x1F))
    return total / ((len(left.data) // 2) * (31 + 63 + 31))


class FirmwareShellAcceptance:
    def __init__(self, args):
        self.args = args
        self.requested_port = args.port
        self.port = args.port
        self.dev = self._connect(args.reconnect_timeout)
        self.port_identity = SerialPortIdentity.capture(self.port)
        self._identity_was_enumerated = bool(
            self.port_identity.present_devices()
        )
        self.mode = None
        self.launcher_frame = None
        self.fluid_frame = None

    def close(self):
        if self.dev is not None:
            self.dev.close()
            self.dev = None

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
                dev = AcceptanceDevice(
                    port=port,
                    baudrate=self.args.baud,
                    out_dir=self.args.out_dir,
                )
                self.port = port
                print("[accept] connected: {} @ {} baud".format(port, self.args.baud))
                return dev
            except (device_dev.PortNotFound, UserError) as exc:
                last_error = exc
                time.sleep(min(0.25, max(0.0, deadline - time.monotonic())))
        raise UserError(
            "could not connect to the firmware within {:.1f}s: {}"
            .format(timeout, last_error or "serial port did not appear")
        )

    def _reconnect(self, timeout: float | None = None):
        if self.dev is not None:
            self.dev.close()
            self.dev = None
        self.dev = self._connect(
            self.args.reconnect_timeout if timeout is None else timeout,
            self.port_identity,
        )
        self.mode = None

    def _wait_for_transport_loss(
        self,
        label: str,
        deadline: float,
        command_checkpoint: int,
    ):
        """Observe device-generated link loss without closing the live handle."""
        dev = self.dev
        if dev is None:
            raise UserError(
                "{} cannot prove a reboot because the serial link was already closed"
                .format(label)
            )

        failure_cursor = command_checkpoint
        failure_description = "device-generated transport loss after {}".format(label)

        while True:
            failure_cursor = dev.scan_command_failures(
                failure_cursor,
                failure_description,
            )
            if not dev.reader.is_alive():
                # The reader cannot append more lines once it has stopped.  Scan once
                # more to close the race between the first scan and reader shutdown.
                dev.scan_command_failures(failure_cursor, failure_description)
                if dev._stop.is_set():
                    raise UserError(
                        "{} cannot prove a reboot because the host closed the serial "
                        "link before device-generated transport loss was observed"
                        .format(label)
                    )
                print(
                    "[accept] {} device-generated transport loss observed"
                    .format(label)
                )
                return

            if (
                self._identity_was_enumerated
                and not self.port_identity.present_devices()
            ):
                dev.scan_command_failures(failure_cursor, failure_description)
                print(
                    "[accept] {} target disappeared from serial enumeration"
                    .format(label)
                )
                return

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                dev.scan_command_failures(failure_cursor, failure_description)
                raise UserError(
                    "{} did not produce observable device-generated transport loss "
                    "within {:.1f}s; refusing a host-induced close/reconnect"
                    .format(label, self.args.reconnect_timeout)
                )
            time.sleep(min(PORT_POLL_SECONDS, remaining))

    def _observe_reconnected_launcher(
        self,
        label: str,
        command_checkpoint: int,
    ):
        """Require transport loss, reconnect by identity, then verify launcher."""
        deadline = time.monotonic() + self.args.reconnect_timeout
        self._wait_for_transport_loss(label, deadline, command_checkpoint)
        self._reconnect(max(0.001, deadline - time.monotonic()))
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise UserError(
                    "{} re-enumerated, but launcher mode was not observed within "
                    "{:.1f}s".format(label, self.args.reconnect_timeout)
                )

            try:
                checkpoint = self.dev.checkpoint()
                self.dev.echo_send("status")
                _, line = self.dev.wait_line(
                    lambda candidate: candidate.strip().startswith(STATUS_PREFIX),
                    checkpoint,
                    min(STATUS_RETRY_SECONDS, remaining),
                    "the requested status response for {}".format(label),
                )
            except AcceptanceTimeout:
                continue
            except DeviceGone:
                self._reconnect(max(0.001, deadline - time.monotonic()))
                continue

            actual = validate_status(line, label)
            if actual != "launcher":
                raise UserError(
                    "{} returned in {} mode; expected launcher"
                    .format(label, actual)
                )
            observed_markers = {
                candidate.strip()
                for _, candidate in self.dev.lines_since(0)
                if candidate.strip() in (MODE_LAUNCHER, MODE_FLUID)
            }
            if MODE_FLUID in observed_markers:
                raise UserError(
                    "{} emitted a fluid_box MODE marker while reconnect status "
                    "reported launcher".format(label)
                )
            self.mode = actual
            print(
                "[accept] {} re-enumerated in launcher mode"
                .format(label)
            )
            return

    def _probe_mode(
        self,
        expected: str,
        label: str,
        timeout: float | None = None,
    ):
        if expected not in ("launcher", "fluid_box"):
            raise UserError(
                "{} requested an unsupported mode probe: {}"
                .format(label, expected)
            )

        checkpoint = self.dev.checkpoint()
        self.dev.echo_send("status")
        _, line = self.dev.wait_line(
            lambda candidate: candidate.strip().startswith(STATUS_PREFIX),
            checkpoint,
            self.args.timeout if timeout is None else timeout,
            "the requested status response for {}".format(label),
        )

        actual = validate_status(line, label)

        if actual != expected:
            raise UserError(
                "{} returned in {} mode; expected {}"
                .format(label, actual, expected)
            )
        self.mode = actual
        print("[accept] {} confirmed {} mode".format(label, actual))


    def _send_wait_exact(self, command: str, marker: str):
        checkpoint = self.dev.checkpoint()
        self.dev.echo_send(command)
        return self.dev.wait_exact(marker, checkpoint, self.args.timeout)

    def _send_wait_prefix(self, command: str, marker: str):
        checkpoint = self.dev.checkpoint()
        self.dev.echo_send(command)
        return self.dev.wait_prefix(marker, checkpoint, self.args.timeout)

    def _transition(self, command: str, marker: str):
        sequence, _ = self._send_wait_exact(command, marker)
        self.mode = "launcher" if marker == MODE_LAUNCHER else "fluid_box"
        return sequence

    def _boot_to_launcher(self):
        checkpoint = self.dev.checkpoint()
        self.dev.echo_send("input boot 150")
        try:
            self._observe_reconnected_launcher("short BOOT input", checkpoint)
        except UserError as exc:
            raise UserError(
                "short BOOT input did not reboot and re-enumerate in launcher "
                "mode: {}".format(exc)
            ) from exc

    def _legacy_reboot_to_launcher(self):
        checkpoint = self.dev.checkpoint()
        self.dev.echo_send("reboot")
        try:
            self.dev.wait_exact("@DEV REBOOTING", checkpoint, self.args.timeout)
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
            self._observe_reconnected_launcher("legacy reboot command", checkpoint)
        except UserError as exc:
            raise UserError(
                "legacy reboot command did not re-enumerate in launcher mode: {}"
                .format(exc)
            ) from exc


    def _ensure_launcher(self):
        if self.mode == "launcher":
            return
        if self.mode == "fluid_box":
            self._transition("input pwr 120", MODE_LAUNCHER)
            return
        self._boot_to_launcher()

    def _ensure_fluid(self):
        self._ensure_launcher()
        self._transition("input plus", MODE_FLUID)

    def _capture(self, label: str) -> FrameEvent:
        frame = self.dev.capture(self.args.capture_timeout)
        self._assert_frame_shape(frame, label)
        path = save_frame(self.dev, frame, self.args.out_dir)
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
        _, line = self.dev.wait_telemetry(
            predicate,
            after,
            self.args.telemetry_timeout,
            description,
        )
        sample = parse_telemetry(0, line)
        if sample is None:
            raise UserError("internal telemetry observation error")
        return sample

    def _health_baseline(self, after: int, label: str) -> HealthBaseline | None:
        observed = {}
        for field in ("missed", "nonfinite"):
            try:
                sequence, line = self.dev.wait_telemetry(
                    lambda sample, field=field: getattr(sample, field) is not None,
                    after,
                    self.args.telemetry_timeout,
                    "{} telemetry for {}".format(field, label),
                )
            except AcceptanceTimeout:
                print(
                    "[accept] {}: {} telemetry not observable; skipped"
                    .format(label, field)
                )
                continue
            observed[field] = parse_telemetry(sequence, line)

        if not observed:
            return None
        return HealthBaseline(
            missed=observed.get("missed"),
            nonfinite=observed.get("nonfinite"),
        )

    def _assert_health(
        self,
        baseline: HealthBaseline | None,
        action_end: int,
        label: str,
    ):
        if baseline is None:
            return

        stable = {}
        for field in ("missed", "nonfinite"):
            initial_sample = getattr(baseline, field)
            if initial_sample is None:
                continue
            initial = getattr(initial_sample, field)
            post_sequence, _ = self.dev.wait_telemetry(
                lambda sample, field=field: getattr(sample, field) is not None,
                action_end,
                self.args.telemetry_timeout,
                "post-action {} telemetry for {}".format(field, label),
            )
            observed = [
                value
                for sample in self.dev.telemetry_since(initial_sample.sequence)
                if sample.sequence <= post_sequence
                and (value := getattr(sample, field)) is not None
            ]
            changed = next(
                (value for value in observed if value != initial),
                None,
            )
            if changed is not None:
                raise UserError(
                    "{} changed cumulative {} telemetry from baseline {} to {}; "
                    "every observed value must remain exactly equal to baseline"
                    .format(label, field, initial, changed)
                )
            stable[field] = initial
        print(
            "[accept] {} health stable: missed={} nonfinite={}"
            .format(label, stable.get("missed"), stable.get("nonfinite"))
        )


    def check_launcher(self):
        self._boot_to_launcher()
        self._send_wait_exact("ping", "@DEV PONG")
        print("[accept] launcher check confirmed legacy ping response")
        self._probe_mode("launcher", "launcher check")
        self.launcher_frame = self._capture("launcher")

    def check_once(self):
        self._ensure_launcher()
        launcher = self.launcher_frame or self._capture("launcher-before-once")
        health = self._health_baseline(
            self.dev.checkpoint(), "launch/home"
        )
        self._transition("input plus", MODE_FLUID)
        self._probe_mode("fluid_box", "launch")
        fluid = self._capture("fluid-after-launch")
        self._assert_distinct(launcher, fluid, "launch")
        self._transition("input pwr 120", MODE_LAUNCHER)
        returned = self._capture("launcher-after-home")
        self._assert_distinct(fluid, returned, "home")
        self._probe_mode("launcher", "home")
        action_end = self.dev.checkpoint()
        self._assert_health(health, action_end, "launch/home")
        self.launcher_frame = returned
        self.fluid_frame = fluid

    def check_cycles(self):
        self._ensure_launcher()
        baseline_epoch = None
        health = self._health_baseline(
            self.dev.checkpoint(),
            "{} transition cycles".format(self.args.cycles),
        )
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
            self._transition("input pwr 120", MODE_LAUNCHER)
            if cycle == 1 or cycle % 10 == 0 or cycle == self.args.cycles:
                print("[accept] transition cycle {}/{}".format(cycle, self.args.cycles))

        action_end = self.dev.checkpoint()
        self._assert_health(health, action_end, "transition cycles")

    def check_freeze(self):
        self._ensure_fluid()
        entry_sequence = self.dev.checkpoint()
        initial = self._wait_epoch(
            entry_sequence,
            lambda sample: sample.epoch is not None,
            "Fluid reset_epoch telemetry",
        )
        health = self._health_baseline(entry_sequence, "freeze/resume")

        self._send_wait_prefix("motion 0 0 6 0", "@DEV MOTION ")
        reset_checkpoint = self.dev.checkpoint()
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
            frame_distance(left, right)
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

        distance_to_frozen = frame_distance(evolved, resumed)
        distance_to_reset = frame_distance(reset_frame, resumed)
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

        plus_checkpoint = self.dev.checkpoint()
        self.dev.echo_send("input plus")
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

        action_end = self.dev.checkpoint()
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
            self._ensure_fluid()
        health = self._health_baseline(
            self.dev.checkpoint(), "bounded soak"
        )
        start_frame = self._capture("fluid-soak-start")
        deadline = time.monotonic() + self.args.soak_seconds

        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            self._probe_mode(
                "fluid_box",
                "bounded soak",
                min(self.args.timeout, remaining),
            )
            remaining = deadline - time.monotonic()
            if remaining > 0:
                time.sleep(min(1.0, remaining))

        end_frame = self._capture("fluid-soak-end")
        self._assert_frame_shape(start_frame, "fluid-soak-start")
        self._assert_frame_shape(end_frame, "fluid-soak-end")
        action_end = self.dev.checkpoint()
        self._assert_health(health, action_end, "bounded soak")
        self._transition("input pwr 120", MODE_LAUNCHER)

    def check_power_off(self):
        self._ensure_fluid()
        minimum_absence = self.args.power_off_min_absence
        if minimum_absence >= self.args.power_off_timeout:
            raise UserError(
                "--power-off-min-absence must be less than --power-off-timeout"
            )
        checkpoint = self.dev.checkpoint()
        self.dev.echo_send("input pwr 3000")
        deadline = time.monotonic() + self.args.power_off_timeout
        ping_sent = False
        absent_since = None

        def reject_protocol_activity(dev):
            for _, line in dev.lines_since(checkpoint):
                dev._raise_if_command_rejected(
                    line,
                    "device-generated transport loss after long PWR input",
                )
                stripped = line.strip()
                if stripped == MODE_LAUNCHER:
                    raise UserError(
                        "long PWR input emitted launcher mode; the hold was also "
                        "misclassified as a short press"
                    )
                if stripped == "@DEV PONG":
                    raise UserError(
                        "device still answered ping after long PWR power-off input"
                    )

        def close_and_scan_device():
            dev = self.dev
            dev.close()
            reject_protocol_activity(dev)
            self.dev = None

        while time.monotonic() < deadline:
            if self.dev is not None:
                reject_protocol_activity(self.dev)
                if not self.dev.reader.is_alive():
                    close_and_scan_device()
                elif not ping_sent and time.monotonic() + 1.0 >= deadline:
                    try:
                        self.dev.echo_send("ping")
                        ping_sent = True
                    except DeviceGone:
                        close_and_scan_device()

            if self.dev is None:
                present = self.port_identity.present_devices()
                now = time.monotonic()
                if not present:
                    if absent_since is None:
                        absent_since = now
                elif absent_since is not None:
                    raise UserError(
                        "serial device re-enumerated as {} after {:.2f}s absent "
                        "following long PWR input; the hold rebooted the board "
                        "instead of powering it off"
                        .format(", ".join(present), now - absent_since)
                    )
            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))

        if self.dev is not None:
            reject_protocol_activity(self.dev)
            if self.dev.reader.is_alive():
                raise UserError(
                    "device remained serially connected after {:.1f}s following "
                    "long PWR input; an unanswered ping is not positive power-off "
                    "evidence".format(self.args.power_off_timeout)
                )
            close_and_scan_device()

        present = self.port_identity.present_devices()
        observed_at = time.monotonic()
        if present:
            if absent_since is not None:
                raise UserError(
                    "serial device re-enumerated as {} after {:.2f}s absent "
                    "following long PWR input; the hold rebooted the board instead "
                    "of powering it off"
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
        print(
            "[accept] power-off kept the serial device continuously absent for "
            "{:.2f}s without a short event or re-enumeration"
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
        choices=ALL_CHECKS + ("all",),
        metavar="CHECK",
        help="subset: launcher once cycles freeze reboot soak all",
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
        help="run the destructive long-PWR check last; the board must be powered back on manually",
    )
    parser.add_argument(
        "--power-off-timeout",
        type=positive_finite,
        default=10.0,
        metavar="SEC",
    )
    parser.add_argument(
        "--power-off-min-absence",
        type=positive_finite,
        default=DEFAULT_POWER_OFF_MIN_ABSENCE,
        metavar="SEC",
        help=(
            "minimum continuous target absence required before power-off passes "
            "(default %(default)s)"
        ),
    )
    return parser


def selected_checks(args) -> tuple[str, ...]:
    requested = tuple(args.checks) if args.checks else CORE_CHECKS
    if "all" in requested:
        requested = ALL_CHECKS
    ordered = tuple(check for check in ALL_CHECKS if check in requested)
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
