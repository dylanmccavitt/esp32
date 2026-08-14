#!/usr/bin/env python3
"""Drive the firmware shell and decode ``@FB`` screenshots over serial."""

from __future__ import annotations

import argparse
import base64
import binascii
import collections
import glob
import os
import queue
import struct
import sys
import threading
import time
import zlib

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None

__all__ = [
    "Device",
    "DeviceGone",
    "FrameEvent",
    "FrameParser",
    "POSES",
    "ProtocolError",
    "CaptureTimeout",
    "UserError",
    "discover_port",
    "make_capture_path",
    "main",
    "pose_command",
    "rgb565be_to_png",
    "save_frame",
]

FRAME_FORMAT = "RGB565BE"
CMD_EOL = "\n"
MAX_FRAME_DIM = 4096
MAX_FRAME_BYTES = 64 * 1024 * 1024
MAX_LINE_BYTES = 4 * 1024 * 1024
PNG_FILTER_NONE = 0

POSES = collections.OrderedDict((
    ("left", (-6, 0, 0)),
    ("right", (6, 0, 0)),
    ("up", (0, 6, 0)),
    ("down", (0, -6, 0)),
    ("front", (0, 0, -6)),
    ("back", (0, 0, 6)),
))

READ_CHUNK = 4096
READ_TIMEOUT = 0.25
WRITE_TIMEOUT = 2.0
POST_MOTION_SETTLE_S = 0.8

DEFAULT_BAUD = 115200
DEFAULT_OUT_DIR = os.path.join("build", "device-captures")
DEFAULT_CAPTURE_TIMEOUT = 45.0
DEFAULT_SEND_DRAIN = 2.0


class UserError(Exception):
    pass


class DeviceGone(UserError):
    pass


class CaptureTimeout(UserError):
    pass


class ProtocolError(UserError):
    pass


def _png_chunk(tag: bytes, data: bytes) -> bytes:
    return (struct.pack(">I", len(data)) + tag + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))


def rgb565be_to_png(data: bytes | bytearray, width: int, height: int) -> bytes:
    expected_bytes = width * height * 2
    if len(data) != expected_bytes:
        raise ValueError(
            "RGB565BE payload is {} bytes; expected exactly {} ({:d}x{:d} pixels, "
            "2 bytes each)".format(len(data), expected_bytes, width, height))
    stride = width * 3
    scanlines = bytearray(height * (1 + stride))
    output_offset = 0
    for row in range(height):
        scanlines[output_offset] = PNG_FILTER_NONE
        output_offset += 1
        row_base = row * width * 2
        for column in range(width):
            packed_pixel = struct.unpack_from(">H", data, row_base + column * 2)[0]
            red_5bit = (packed_pixel >> 11) & 0x1F
            green_6bit = (packed_pixel >> 5) & 0x3F
            blue_5bit = packed_pixel & 0x1F
            scanlines[output_offset] = (red_5bit << 3) | (red_5bit >> 2)
            scanlines[output_offset + 1] = (green_6bit << 2) | (green_6bit >> 4)
            scanlines[output_offset + 2] = (blue_5bit << 3) | (blue_5bit >> 2)
            output_offset += 3
    image_header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n"
            + _png_chunk(b"IHDR", image_header)
            + _png_chunk(b"IDAT", zlib.compress(bytes(scanlines), 9))
            + _png_chunk(b"IEND", b""))


FrameEvent = collections.namedtuple("FrameEvent", "seq width height data")


class FrameParser:

    def __init__(self):
        self.reset()

    def reset(self):
        self._active = False
        self.seq = None
        self.width = 0
        self.height = 0
        self.expected = 0
        self._data = bytearray()


    def feed_line(self, line: str):
        if line.startswith("@FB BEGIN "):
            return self._begin(line[len("@FB BEGIN "):])
        if line.startswith("@FB DATA "):
            return self._data_line(line[len("@FB DATA "):])
        if line.startswith("@FB END "):
            return self._end(line[len("@FB END "):])
        if line.startswith("@FB ERROR "):
            rest = line[len("@FB ERROR "):]
            tokens = rest.split(maxsplit=1)
            try:
                seq = int(tokens[0]) if len(tokens) == 2 else None
            except ValueError:
                seq = None
            if seq is None:
                self.reset()
                return [("error", "malformed @FB ERROR: {!r}".format(rest))]
            if self._active and self.seq == seq:
                self.reset()
            return [(
                "error",
                (seq, "firmware framebuffer error: {}".format(tokens[1])),
            )]
        return [("warn", "unrecognized @FB line {!r}; expected "
                         "BEGIN/DATA/END/ERROR".format(line))]


    def _begin(self, rest: str):
        events = []
        if self._active:
            events.append(
                ("warn", "new frame announced while frame seq={} was still "
                         "incomplete; previous frame discarded".format(self.seq)))
            self.reset()
        tokens = rest.split()
        if len(tokens) not in (4, 5):
            return [("error", "malformed @FB BEGIN: {!r} (expected "
                              "\"<seq> <w> <h> <fmt> [bytes]\")".format(rest))]
        try:
            seq = int(tokens[0])
            width = int(tokens[1])
            height = int(tokens[2])
        except ValueError:
            return [("error", "malformed @FB BEGIN numbers: {!r}".format(rest))]
        if seq < 0 or not (1 <= width <= MAX_FRAME_DIM) or not (1 <= height <= MAX_FRAME_DIM):
            return [("error", "@FB BEGIN dimensions out of range: seq={} "
                              "{}x{} (width/height must be 1..{})"
                              .format(seq, width, height, MAX_FRAME_DIM))]
        fmt = tokens[3]
        if fmt != FRAME_FORMAT:
            return [("error", "unsupported pixel format {!r}; only {!r} is "
                              "implemented".format(fmt, FRAME_FORMAT))]
        expected = width * height * 2
        if expected > MAX_FRAME_BYTES:
            return [("error", "@FB BEGIN frame would need {} bytes, exceeding "
                              "the {} byte safety limit".format(expected, MAX_FRAME_BYTES))]
        if len(tokens) == 5:
            try:
                announced = int(tokens[4])
            except ValueError:
                return [("error", "malformed @FB BEGIN byte count: {!r}"
                                  .format(tokens[4]))]
            if announced != expected:
                return [("error", "@FB BEGIN announced {} bytes but {}x{} "
                                  "RGB565BE requires exactly {}"
                                  .format(announced, width, height, expected))]
        self._active = True
        self.seq = seq
        self.width = width
        self.height = height
        self.expected = expected
        self._data = bytearray()
        return events

    def _data_line(self, rest: str):
        tokens = rest.split()
        if len(tokens) != 2:
            message = ("malformed @FB DATA: {!r} (expected "
                       "\"<seq> <base64>\")".format(rest))
            return self._abort(message) if self._active else [("error", message)]
        try:
            seq = int(tokens[0])
        except ValueError:
            message = "malformed @FB DATA sequence: {!r}".format(tokens[0])
            return self._abort(message) if self._active else [("error", message)]
        if not self._active:
            return [("warn", "@FB DATA for seq={} arrived with no frame in "
                             "progress; ignored".format(seq))]
        if seq != self.seq:
            return self._abort("DATA seq={} does not match in-progress frame "
                               "seq={}".format(seq, self.seq))
        try:
            chunk = base64.b64decode(tokens[1], validate=True)
        except (binascii.Error, ValueError):
            return self._abort("DATA seq={} carries invalid base64".format(seq))
        self._data += chunk
        if len(self._data) > self.expected:
            return self._abort("DATA seq={} exceeded the expected {} bytes "
                               "({}x{} RGB565BE)"
                               .format(seq, self.expected, self.width, self.height))
        return []

    def _end(self, rest: str):
        tokens = rest.split()
        if len(tokens) != 1:
            message = "malformed @FB END: {!r} (expected \"<seq>\")".format(rest)
            return self._abort(message) if self._active else [("error", message)]
        try:
            seq = int(tokens[0])
        except ValueError:
            message = "malformed @FB END sequence: {!r}".format(tokens[0])
            return self._abort(message) if self._active else [("error", message)]
        if not self._active:
            return [("warn", "@FB END for seq={} arrived with no frame in "
                             "progress; ignored".format(seq))]
        if seq != self.seq:
            return self._abort("END seq={} does not match in-progress frame "
                               "seq={}".format(seq, self.seq))
        if len(self._data) != self.expected:
            return self._abort("frame seq={} ended with {} bytes, but @FB BEGIN "
                               "promised exactly {} bytes ({}x{} RGB565BE)"
                               .format(seq, len(self._data), self.expected,
                                       self.width, self.height))
        frame = FrameEvent(self.seq, self.width, self.height, bytes(self._data))
        self.reset()
        return [("frame", frame)]


    def _abort(self, message: str):
        sequence = self.seq
        self.reset()
        return [("error", (sequence, "protocol corruption: " + message +
                           "; frame discarded"))]


def discover_port():
    patterns = ("/dev/cu.usbmodem*", "/dev/tty.usbmodem*",
                "/dev/ttyACM*", "/dev/ttyUSB*")
    for pattern in patterns:
        for match in sorted(glob.glob(pattern)):
            return match
    raise PortNotFound(
        "no serial device found; scanned patterns: "
        + ", ".join(patterns)
        + ". Is the ESP32-S3 plugged in with its native USB (USB Serial/JTAG)? "
          "Pass --port /dev/... to override.")


class PortNotFound(UserError):
    pass


class ReaderThread(threading.Thread):

    def __init__(self, device):
        super().__init__(name="device-dev-reader", daemon=True)
        self._device = device

    def run(self):
        device = self._device
        serial_port = device.serial_port
        pending_bytes = b""
        try:
            while not device._stop.is_set():
                try:
                    chunk = serial_port.read(READ_CHUNK)
                except serial.SerialException as exc:  # type: ignore[union-attr]
                    if not device._stop.is_set():
                        device.post_error(
                            "serial read failed: {} (device disconnected or "
                            "another process took the port?)".format(exc))
                    break
                if not chunk:
                    continue
                pending_bytes += chunk
                while True:
                    newline_offset = pending_bytes.find(b"\n")
                    if newline_offset < 0:
                        break
                    line = pending_bytes[:newline_offset]
                    pending_bytes = pending_bytes[newline_offset + 1:]
                    self._process_line(line)
                if len(pending_bytes) > MAX_LINE_BYTES:
                    device.emit(b"[device_dev] input line exceeded the safety limit; "
                             b"discarding buffered bytes\n")
                    pending_bytes = b""
        finally:
            if pending_bytes:
                device.emit(pending_bytes)

    def _process_line(self, raw_line: bytes):
        device = self._device
        line = raw_line.decode("utf-8", "replace").rstrip("\r")
        if line.startswith("@FB "):
            for kind, payload in device.parser.feed_line(line):
                if kind == "frame":
                    device.post_frame(payload)
                elif kind == "error":
                    if isinstance(payload, tuple):
                        error_sequence, message = payload
                        device.post_error(message, error_sequence)
                    else:
                        device.post_error("protocol corruption: {}".format(payload))
                else:
                    device.print_err("protocol warning: {}".format(payload))
        else:
            device.emit(line + "\n")


class Device:

    def __init__(self, port=None, baudrate=DEFAULT_BAUD, out_dir=None):
        self.port = port
        self.baudrate = baudrate
        self.out_dir = out_dir
        self.cmd_eol = CMD_EOL
        self.parser = FrameParser()
        self._print_lock = threading.Lock()
        self._capture_events = queue.Queue(maxsize=8)
        self._stop = threading.Event()
        self._waiters = 0
        self._waiter_lock = threading.Lock()
        self._capture_lock = threading.Lock()
        self._request_seq = (time.monotonic_ns() & 0xFFFFFFFF) or 1
        self._dropped_frames = 0
        self._last_drop_warn = 0.0
        try:
            self.serial_port = serial.Serial(  # type: ignore[union-attr]
                port=None,
                baudrate=self.baudrate,
                timeout=READ_TIMEOUT,
                write_timeout=WRITE_TIMEOUT,
                dsrdtr=False,
                rtscts=False,
            )
            self.serial_port.rts = True  # Espressif reset lines are active-low.
            self.serial_port.dtr = True
            self.serial_port.port = self.port
            self.serial_port.open()
            self.serial_port.rts = False
            self.serial_port.dtr = False
        except serial.SerialException as exc:  # type: ignore[union-attr]
            raise UserError(
                "could not open serial port {!r} at {} baud: {}"
                .format(self.port, self.baudrate, exc)
                + " (is the cable plugged in, the driver present, and the "
                  "port free of other tools such as idf.py monitor?)"
            ) from exc
        self.reader = ReaderThread(self)
        self.reader.start()


    def emit(self, text: bytes | str):
        with self._print_lock:
            if isinstance(text, bytes):
                text = text.decode("utf-8", "replace")
            sys.stdout.write(text)
            sys.stdout.flush()

    def print_err(self, message: str):
        with self._print_lock:
            print("[device_dev] {}".format(message), file=sys.stderr)

    def echo_send(self, command: str):
        self.send(command)
        with self._print_lock:
            print("[sent] {}".format(command))


    def send(self, command: str):
        if not self.reader.is_alive():
            raise DeviceGone("serial reader is not running; the device is "
                             "unplugged or the port failed")
        payload = ("\x15" + command.rstrip("\r\n") + self.cmd_eol).encode("utf-8")
        try:
            self.serial_port.write(payload)
        except (serial.SerialException,   # type: ignore[union-attr]
                serial.SerialTimeoutException) as exc:  # type: ignore[union-attr]
            raise DeviceGone("write to {!r} failed: {}".format(self.port, exc)) from exc

    def post_frame(self, frame: FrameEvent):
        with self._waiter_lock:
            waiting = self._waiters > 0
        if not waiting:
            self._dropped_frames += 1
            now = time.monotonic()
            if now - self._last_drop_warn >= 10.0:
                self._last_drop_warn = now
                self.print_err("discarded unsolicited framebuffer seq={}; use "
                               "'screenshot' instead of sending raw 'fb'"
                               .format(frame.seq))
            return
        try:
            self._capture_events.put_nowait(("frame", frame))
        except queue.Full:
            self._dropped_frames += 1
            self.print_err("frame seq={} dropped because the capture event "
                           "queue is full".format(frame.seq))

    def post_error(self, message: str, sequence=None):
        self.print_err(message)
        with self._waiter_lock:
            waiting = self._waiters > 0
        if waiting:
            try:
                self._capture_events.put_nowait(("error", (sequence, message)))
            except queue.Full:
                pass


    def capture(self, timeout: float = DEFAULT_CAPTURE_TIMEOUT) -> FrameEvent:
        with self._capture_lock:
            sequence = self._request_seq
            self._request_seq = (sequence + 1) & 0xFFFFFFFF
            if self._request_seq == 0:
                self._request_seq = 1
            while True:
                try:
                    self._capture_events.get_nowait()
                except queue.Empty:
                    break
            with self._waiter_lock:
                self._waiters += 1
            try:
                self.send("fb {}".format(sequence))
                return self._wait_frame(timeout, sequence)
            finally:
                with self._waiter_lock:
                    self._waiters -= 1

    def _wait_frame(self, timeout: float, expected_sequence: int) -> FrameEvent:
        if timeout <= 0:
            timeout = DEFAULT_CAPTURE_TIMEOUT
        deadline = time.monotonic() + timeout
        while True:
            if self._stop.is_set():
                raise DeviceGone("serial link is shutting down")
            if not self.reader.is_alive():
                raise DeviceGone("serial reader stopped while waiting for a "
                                 "frame (device unplugged or port error)")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise CaptureTimeout(
                    "no complete @FB frame for request {} arrived within {:.1f} "
                    "s. Increase --timeout if firmware logged @FB BEGIN but not "
                    "@FB END; otherwise check --port/--baud."
                    .format(expected_sequence, timeout))
            try:
                kind, payload = self._capture_events.get(timeout=remaining)
            except queue.Empty:
                continue
            if kind == "frame":
                if payload.seq == expected_sequence:
                    return payload
                self.print_err("discarded stale framebuffer seq={} while "
                               "waiting for seq={}".format(payload.seq, expected_sequence))
                continue
            error_sequence, message = payload
            if error_sequence is not None and error_sequence != expected_sequence:
                self.print_err("ignored stale framebuffer error seq={} while "
                               "waiting for seq={}: {}"
                               .format(error_sequence, expected_sequence, message))
                continue
            raise ProtocolError(message)

    def close(self):
        self._stop.set()
        if getattr(self, "reader", None) is not None and self.reader.is_alive():
            self.reader.join(timeout=3.0)
        try:
            self.serial_port.close()
        except Exception:
            pass
        if getattr(self, "reader", None) is not None and self.reader.is_alive():
            self.reader.join(timeout=1.0)


def make_capture_path(out_dir: str, sequence: int) -> str:
    timestamp = time.strftime("%Y%m%d-%H%M%S", time.gmtime())
    path_stem = os.path.join(out_dir, "fb_{:04d}_{}".format(sequence, timestamp))
    path = path_stem + ".png"
    suffix_index = 1
    while os.path.exists(path):
        path = "{}_{}.png".format(path_stem, suffix_index)
        suffix_index += 1
    return path


def save_frame(device: Device, frame: FrameEvent, out_dir: str, path: str | None = None) -> str:
    try:
        png_bytes = rgb565be_to_png(frame.data, frame.width, frame.height)
    except ValueError as exc:
        raise ProtocolError(str(exc)) from exc
    try:
        if path is None:
            os.makedirs(out_dir, exist_ok=True)
            path = make_capture_path(out_dir, frame.seq)
        else:
            os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
        with open(path, "wb") as handle:
            handle.write(png_bytes)
    except OSError as exc:
        target = path if path is not None else out_dir
        raise UserError("could not create/write capture at {!r}: {}"
                        .format(target, exc)) from exc
    absolute_path = os.path.abspath(path)
    with device._print_lock:
        print("[capture] saved: {} (seq={}, {}x{} RGB565BE -> PNG)"
              .format(absolute_path, frame.seq, frame.width, frame.height))
    return absolute_path


def pose_command(name: str, duration_ms=0) -> str:
    pose_name = str(name).strip().lower()
    if pose_name not in POSES:
        raise ValueError("unknown pose {!r}; choose from: {}"
                         .format(name, ", ".join(sorted(POSES))))
    gravity_x, gravity_y, gravity_z = POSES[pose_name]
    command = "motion {} {} {}".format(gravity_x, gravity_y, gravity_z)
    if duration_ms:
        try:
            duration_ms = int(duration_ms)
        except (TypeError, ValueError):
            raise ValueError("pose duration must be an integer number of "
                             "milliseconds, got {!r}".format(duration_ms))
        if duration_ms < 0:
            raise ValueError("pose duration must be >= 0, got {}".format(duration_ms))
        command += " {}".format(duration_ms)
    return command


def parse_pose_name(text: str) -> str:
    pose_name = text.strip().lower()
    if pose_name not in POSES:
        raise argparse.ArgumentTypeError(
            "unknown pose {!r}; choose from: {}".format(
                text, ", ".join(sorted(POSES))))
    return pose_name


def build_arg_parser():
    parser = argparse.ArgumentParser(
        prog="device_dev.py",
        description="Host half of the ESP32-S3 fluid-demo serial link: opens "
                    "USB Serial/JTAG with reset lines inactive, streams firmware "
                    "logs, and parses the @FB screenshot protocol into RGB PNG "
                    "files.",
        epilog=(
            "poses (motion gravity vectors, g):\n"
            "  left=(-6,0,0)  right=(6,0,0)  up=(0,6,0)\n"
            "  down=(0,-6,0)  front=(0,0,-6) back=(0,0,6)\n\n"
            "firmware commands (pass-through):\n"
            "  ping, status, motion <ax> <ay> <az> [duration_ms],\n"
            "  release, reset, reboot, yaw <rad>, axes <sx> <sy> <sz>,\n"
            "  gain <0..4> (Level only), tau <0.05..2> (use 'screenshot' for framebuffers)\n\n"
            "each saved capture is printed as:  [capture] saved: <path>\n"
            "dependencies: Python 3 standard library + pyserial (in `eim run`)."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--port", default=None, metavar="DEV",
                        help="serial device; default: first /dev/cu.usbmodem* "
                             "(or tty.usbmodem*/ttyACM*/ttyUSB*)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, metavar="BAUD",
                        help="baud rate, informational for USB CDC "
                             "(default %(default)s)")
    parser.add_argument("--out-dir", default=DEFAULT_OUT_DIR, metavar="DIR",
                        help="directory for auto-named capture PNGs, created "
                             "on demand (default: %(default)s)")
    subparsers = parser.add_subparsers(dest="command", metavar="COMMAND", required=True)

    session_parser = subparsers.add_parser(
        "session",
        help="persistent reader plus stdin commands (logs keep streaming)")
    session_parser.add_argument("--timeout", type=float, default=DEFAULT_CAPTURE_TIMEOUT,
                           metavar="SEC",
                           help="seconds to wait for a screenshot frame "
                                "(default %(default)s)")
    session_parser.set_defaults(handler=cmd_session)

    monitor_parser = subparsers.add_parser(
        "monitor",
        help="print firmware logs until Ctrl-C")
    monitor_parser.set_defaults(handler=cmd_monitor)

    screenshot_parser = subparsers.add_parser(
        "screenshot",
        help="send 'fb', save the next complete frame as PNG")
    screenshot_parser.add_argument("output", nargs="?", default=None, metavar="[OUTPUT]",
                        help="PNG path; default: auto-named under --out-dir")
    screenshot_parser.add_argument("--timeout", type=float, default=DEFAULT_CAPTURE_TIMEOUT,
                        metavar="SEC",
                        help="seconds to wait for the frame (default %(default)s)")
    screenshot_parser.set_defaults(handler=cmd_screenshot)

    send_parser = subparsers.add_parser(
        "send",
        help="send one firmware protocol command (words joined by spaces)")
    send_parser.add_argument("command", nargs="+", metavar="command",
                        help="command text, e.g. 'send ping' or "
                             "'send motion 0 6 0 500'")
    send_parser.add_argument("--timeout", type=float, default=DEFAULT_SEND_DRAIN,
                        metavar="SEC",
                        help="seconds to keep draining/printing logs after "
                             "sending (default %(default)s)")
    send_parser.set_defaults(handler=cmd_send)

    drive_parser = subparsers.add_parser(
        "drive",
        help="apply a simulated gravity pose via 'motion'")
    drive_parser.add_argument("pose", type=parse_pose_name, metavar="POSE",
                         help="pose name: " + ", ".join(sorted(POSES)))
    drive_parser.add_argument("--duration", type=int, default=0, metavar="MS",
                         help="motion duration in ms; 0 (default) sustains the "
                              "pose until 'release' or reboot")
    drive_parser.add_argument("--timeout", type=float, default=DEFAULT_CAPTURE_TIMEOUT,
                         metavar="SEC",
                         help="seconds to wait for the screenshot frame "
                              "(default %(default)s)")
    drive_parser.add_argument("--screenshot", nargs="?", const="", default=None,
                         metavar="[PATH]",
                         help="after a short settle, send 'fb' and save the "
                              "next frame; optional PATH (default: auto-named "
                              "under --out-dir)")
    drive_parser.set_defaults(handler=cmd_drive)
    return parser


def cmd_monitor(device: Device, args):
    device.emit("[device_dev] monitoring {} @ {} baud; logs below. Ctrl-C to stop.\n"
             .format(device.port, device.baudrate))
    while True:
        if not device.reader.is_alive():
            raise DeviceGone("serial reader stopped unexpectedly (device "
                             "unplugged or port error)")
        if device._stop.wait(0.5):
            break
    return 0


def cmd_screenshot(device: Device, args):
    frame = device.capture(args.timeout)
    save_frame(device, frame, args.out_dir, args.output)
    return 0


def cmd_send(device: Device, args):
    if args.command[0] == "fb":
        raise UserError("raw 'fb' is request-scoped; use the 'screenshot' command")
    command = " ".join(args.command)
    device.echo_send(command)
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        if not device.reader.is_alive():
            raise DeviceGone("serial reader stopped unexpectedly (device "
                             "unplugged or port error)")
        time.sleep(0.05)
    return 0


def cmd_drive(device: Device, args):
    try:
        command = pose_command(args.pose, args.duration)
    except ValueError as exc:
        raise UserError(str(exc)) from exc
    device.echo_send(command)
    if args.screenshot is not None:
        time.sleep(POST_MOTION_SETTLE_S)
        path = args.screenshot or None
        frame = device.capture(args.timeout)
        save_frame(device, frame, args.out_dir, path)
    return 0


def cmd_session(device: Device, args):
    device.emit(
        "[device_dev] connected to {} @ {} baud; logs stream below.\n"
        "  commands: 'screenshot [path]', 'pose <name> [duration_ms]',\n"
        "  or any other line is sent to the board verbatim (ping, status,\n"
        "  motion, release, reset, reboot, yaw, axes, gain, tau). Ctrl-C exits.\n"
        .format(device.port, device.baudrate))

    stdin_events = queue.Queue()

    def read_stdin():
        while True:
            try:
                line = sys.stdin.readline()
            except OSError as exc:
                stdin_events.put(("error", str(exc)))
                return
            if line == "":
                stdin_events.put(("eof", None))
                return
            stdin_events.put(("line", line))

    threading.Thread(target=read_stdin, name="device-dev-stdin", daemon=True).start()
    screenshot_count = 0
    prompt_visible = False
    try:
        while True:
            if not device.reader.is_alive():
                raise DeviceGone("serial reader stopped unexpectedly (device "
                                 "unplugged or port error)")
            if not prompt_visible:
                device.emit("> ")
                prompt_visible = True
            try:
                event, line = stdin_events.get(timeout=0.1)
            except queue.Empty:
                continue
            prompt_visible = False
            if event == "eof":
                device.print_err("stdin closed; ending session.")
                break
            if event == "error":
                raise UserError("stdin read failed: {}".format(line))

            text = line.strip()
            if not text:
                continue
            command_parts = text.split()
            try:
                if command_parts[0] == "screenshot":
                    if len(command_parts) > 2:
                        device.print_err("usage: screenshot [path]")
                        continue
                    path = command_parts[1] if len(command_parts) == 2 else None
                    frame = device.capture(args.timeout)
                    save_frame(device, frame, args.out_dir, path)
                    screenshot_count += 1
                elif command_parts[0] == "pose":
                    if not (2 <= len(command_parts) <= 3):
                        device.print_err("usage: pose <name> [duration_ms]; names: "
                                      + ", ".join(sorted(POSES)))
                        continue
                    try:
                        command = pose_command(command_parts[1],
                                               command_parts[2] if len(command_parts) == 3 else 0)
                    except ValueError as exc:
                        device.print_err(str(exc))
                        continue
                    device.echo_send(command)
                elif command_parts[0] == "fb":
                    device.print_err("raw 'fb' is request-scoped; use 'screenshot [path]'")
                else:
                    device.echo_send(text)
            except DeviceGone:
                raise
            except UserError as exc:
                device.print_err(str(exc))
    except KeyboardInterrupt:
        device.print_err("interrupted; ending session.")
    finally:
        with device._print_lock:
            print("[device_dev] session ended; {} screenshot(s) saved.".format(screenshot_count))
    return 0


def _die(message: str, code: int = 2) -> None:
    print("device_dev: error: {}".format(message), file=sys.stderr)
    sys.exit(code)


def main(argv=None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    if serial is None:
        _die("pyserial is required but not installed. Activate the `eim run` "
             "environment (pyserial is available there) or `pip install "
             "pyserial`.")
    try:
        port = args.port or discover_port()
        device = Device(port=port, baudrate=args.baud, out_dir=args.out_dir)
    except (PortNotFound, UserError) as exc:
        _die(str(exc))
    try:
        return args.handler(device, args)
    except UserError as exc:
        _die(str(exc))
    except KeyboardInterrupt:
        with device._print_lock:
            print("\n[device_dev] interrupted; closing serial port.",
                  file=sys.stderr)
    finally:
        device.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
