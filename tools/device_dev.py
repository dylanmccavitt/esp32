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
MAX_LINE_BYTES = 4 * 1024 * 1024  # a single @FB line must not exceed this

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
    """Any error that should be reported to the user with a clean message."""


class DeviceGone(UserError):
    """The serial link died (unplug, port error, shutdown in progress)."""


class CaptureTimeout(UserError):
    """No complete frame arrived within the requested window."""


class ProtocolError(UserError):
    """The frame currently being awaited was corrupted on the wire."""


def _png_chunk(tag: bytes, data: bytes) -> bytes:
    return (struct.pack(">I", len(data)) + tag + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))


def rgb565be_to_png(data: bytes | bytearray, width: int, height: int) -> bytes:
    """Convert an RGB565BE pixel buffer into a valid RGB8 PNG (bytes)."""
    expected = width * height * 2
    if len(data) != expected:
        raise ValueError(
            "RGB565BE payload is {} bytes; expected exactly {} ({:d}x{:d} pixels, "
            "2 bytes each)".format(len(data), expected, width, height))
    stride = width * 3
    raw = bytearray(height * (1 + stride))
    out = 0
    for y in range(height):
        raw[out] = 0                          # filter type 0 (None) per row
        out += 1
        row_base = y * width * 2
        for x in range(width):
            v = struct.unpack_from(">H", data, row_base + x * 2)[0]
            r5 = (v >> 11) & 0x1F
            g6 = (v >> 5) & 0x3F
            b5 = v & 0x1F
            # bit-replication expansion to 8 bits/channel
            raw[out] = (r5 << 3) | (r5 >> 2)
            raw[out + 1] = (g6 << 2) | (g6 >> 4)
            raw[out + 2] = (b5 << 3) | (b5 >> 2)
            out += 3
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n"
            + _png_chunk(b"IHDR", ihdr)
            + _png_chunk(b"IDAT", zlib.compress(bytes(raw), 9))
            + _png_chunk(b"IEND", b""))


FrameEvent = collections.namedtuple("FrameEvent", "seq width height data")


class FrameParser:
    """Consume protocol lines; return ``('frame'|'error'|'warn', payload)``.

    The parser is deliberately dumb about log lines: callers only feed it
    lines that start with ``@FB ``. It enforces sequence continuity and the
    exact ``width*height*2`` byte length promised by ``@FB BEGIN``.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self._active = False
        self.seq = None
        self.width = 0
        self.height = 0
        self.expected = 0
        self._data = bytearray()

    # -- line dispatch ------------------------------------------------------

    def feed_line(self, line: str):
        """Return a list of ``(kind, payload)`` events for one protocol line."""
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

    # -- handlers ------------------------------------------------------------

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
        ev = FrameEvent(self.seq, self.width, self.height, bytes(self._data))
        self.reset()
        return [("frame", ev)]

    # -- helpers --------------------------------------------------------------

    def _abort(self, message: str):
        """Discard the active frame and report corruption against its sequence."""
        sequence = self.seq
        self.reset()
        return [("error", (sequence, "protocol corruption: " + message +
                           "; frame discarded"))]


def discover_port():
    """First matching device, or raise PortNotFound with an actionable hint."""
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
    """Drain the port, print logs, and feed complete lines to FrameParser.

    Buffering every partial line is intentional: serial reads may split even
    the ``@FB `` prefix, and emitting such a prefix early would corrupt a frame.
    """

    def __init__(self, device):
        super().__init__(name="device-dev-reader", daemon=True)
        self._dev = device

    def run(self):
        dev = self._dev
        ser = dev.ser
        buf = b""
        try:
            while not dev._stop.is_set():
                try:
                    chunk = ser.read(READ_CHUNK)
                except serial.SerialException as exc:  # type: ignore[union-attr]
                    if not dev._stop.is_set():
                        dev.post_error(
                            "serial read failed: {} (device disconnected or "
                            "another process took the port?)".format(exc))
                    break
                if not chunk:
                    continue
                buf += chunk
                while True:
                    nl = buf.find(b"\n")
                    if nl < 0:
                        break
                    line = buf[:nl]
                    buf = buf[nl + 1:]
                    self._line(line)
                if len(buf) > MAX_LINE_BYTES:
                    dev.emit(b"[device_dev] input line exceeded the safety limit; "
                             b"discarding buffered bytes\n")
                    buf = b""
        finally:
            if buf:
                dev.emit(buf)           # flush the last partial line

    def _line(self, raw: bytes):
        dev = self._dev
        line = raw.decode("utf-8", "replace").rstrip("\r")
        if line.startswith("@FB "):
            for kind, payload in dev.parser.feed_line(line):
                if kind == "frame":
                    dev.post_frame(payload)
                elif kind == "error":
                    if isinstance(payload, tuple):
                        error_seq, message = payload
                        dev.post_error(message, error_seq)
                    else:
                        dev.post_error("protocol corruption: {}".format(payload))
                else:
                    dev.print_err("protocol warning: {}".format(payload))
        else:
            dev.emit(line + "\n")


class Device:
    """A live serial link with a background reader and capture plumbing.

    DTR/RTS follow Espressif's no-reset open sequence: asserted before opening,
    then RTS deasserted before DTR so neither reset nor download mode fires.
    """

    def __init__(self, port=None, baudrate=DEFAULT_BAUD, out_dir=None):
        self.port = port
        self.baudrate = baudrate
        self.out_dir = out_dir
        self.cmd_eol = CMD_EOL
        self.parser = FrameParser()
        self._print_lock = threading.Lock()
        self._events = queue.Queue(maxsize=8)   # ('frame'|'error', payload)
        self._stop = threading.Event()
        self._waiters = 0
        self._waiter_lock = threading.Lock()
        self._capture_lock = threading.Lock()
        self._request_seq = (time.monotonic_ns() & 0xFFFFFFFF) or 1
        self._dropped_frames = 0
        self._last_drop_warn = 0.0
        try:
            self.ser = serial.Serial(  # type: ignore[union-attr]
                port=None,
                baudrate=self.baudrate,
                timeout=READ_TIMEOUT,
                write_timeout=WRITE_TIMEOUT,
                dsrdtr=False,
                rtscts=False,
            )
            self.ser.rts = True   # pyserial True is Espressif's active-low level
            self.ser.dtr = True
            self.ser.port = self.port
            self.ser.open()
            self.ser.rts = False
            self.ser.dtr = False
        except serial.SerialException as exc:  # type: ignore[union-attr]
            raise UserError(
                "could not open serial port {!r} at {} baud: {}"
                .format(self.port, self.baudrate, exc)
                + " (is the cable plugged in, the driver present, and the "
                  "port free of other tools such as idf.py monitor?)"
            ) from exc
        self.reader = ReaderThread(self)
        self.reader.start()

    # -- output helpers (shared lock keeps stdout lines from interleaving) ---

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
        """Send a command and echo it so responses are easy to follow."""
        self.send(command)
        with self._print_lock:
            print("[sent] {}".format(command))

    # -- protocol plumbing ----------------------------------------------------

    def send(self, command: str):
        if not self.reader.is_alive():
            raise DeviceGone("serial reader is not running; the device is "
                             "unplugged or the port failed")
        payload = ("\x15" + command.rstrip("\r\n") + self.cmd_eol).encode("utf-8")
        try:
            self.ser.write(payload)
        except (serial.SerialException,   # type: ignore[union-attr]
                serial.SerialTimeoutException) as exc:  # type: ignore[union-attr]
            raise DeviceGone("write to {!r} failed: {}".format(self.port, exc)) from exc

    def post_frame(self, ev: FrameEvent):
        with self._waiter_lock:
            waiting = self._waiters > 0
        if not waiting:
            self._dropped_frames += 1
            now = time.monotonic()
            if now - self._last_drop_warn >= 10.0:
                self._last_drop_warn = now
                self.print_err("discarded unsolicited framebuffer seq={}; use "
                               "'screenshot' instead of sending raw 'fb'"
                               .format(ev.seq))
            return
        try:
            self._events.put_nowait(("frame", ev))
        except queue.Full:
            self._dropped_frames += 1
            self.print_err("frame seq={} dropped because the capture event "
                           "queue is full".format(ev.seq))

    def post_error(self, message: str, seq=None):
        self.print_err(message)
        with self._waiter_lock:
            waiting = self._waiters > 0
        if waiting:
            try:
                self._events.put_nowait(("error", (seq, message)))
            except queue.Full:
                pass

    # -- capture ----------------------------------------------------------------

    def capture(self, timeout: float = DEFAULT_CAPTURE_TIMEOUT) -> FrameEvent:
        """Request one uniquely tagged frame and wait only for that response."""
        with self._capture_lock:
            sequence = self._request_seq
            self._request_seq = (sequence + 1) & 0xFFFFFFFF
            if self._request_seq == 0:
                self._request_seq = 1
            while True:
                try:
                    self._events.get_nowait()
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

    def _wait_frame(self, timeout: float, expected_seq: int) -> FrameEvent:
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
                    .format(expected_seq, timeout))
            try:
                kind, payload = self._events.get(timeout=remaining)
            except queue.Empty:
                continue
            if kind == "frame":
                if payload.seq == expected_seq:
                    return payload
                self.print_err("discarded stale framebuffer seq={} while "
                               "waiting for seq={}".format(payload.seq, expected_seq))
                continue
            error_seq, message = payload
            if error_seq is not None and error_seq != expected_seq:
                self.print_err("ignored stale framebuffer error seq={} while "
                               "waiting for seq={}: {}"
                               .format(error_seq, expected_seq, message))
                continue
            raise ProtocolError(message)

    def close(self):
        self._stop.set()
        if getattr(self, "reader", None) is not None and self.reader.is_alive():
            self.reader.join(timeout=3.0)
        try:
            self.ser.close()
        except Exception:
            pass
        if getattr(self, "reader", None) is not None and self.reader.is_alive():
            self.reader.join(timeout=1.0)   # reader is a daemon; best effort


def make_capture_path(out_dir: str, seq: int) -> str:
    """Return a collision-free auto-name inside an existing ``out_dir``."""
    stamp = time.strftime("%Y%m%d-%H%M%S", time.gmtime())
    base = os.path.join(out_dir, "fb_{:04d}_{}".format(seq, stamp))
    path = base + ".png"
    n = 1
    while os.path.exists(path):
        path = "{}_{}.png".format(base, n)
        n += 1
    return path


def save_frame(dev: Device, ev: FrameEvent, out_dir: str, path: str | None = None) -> str:
    """Encode ``ev`` as PNG, write it, and print the saved path clearly."""
    try:
        png = rgb565be_to_png(ev.data, ev.width, ev.height)
    except ValueError as exc:
        raise ProtocolError(str(exc)) from exc
    try:
        if path is None:
            os.makedirs(out_dir, exist_ok=True)
            path = make_capture_path(out_dir, ev.seq)
        else:
            os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
        with open(path, "wb") as handle:
            handle.write(png)
    except OSError as exc:
        target = path if path is not None else out_dir
        raise UserError("could not create/write capture at {!r}: {}"
                        .format(target, exc)) from exc
    absolute = os.path.abspath(path)
    with dev._print_lock:
        print("[capture] saved: {} (seq={}, {}x{} RGB565BE -> PNG)"
              .format(absolute, ev.seq, ev.width, ev.height))
    return absolute


def pose_command(name: str, duration_ms=0) -> str:
    """``motion <ax> <ay> <az> [duration_ms]`` for a named pose, or raise ValueError."""
    key = str(name).strip().lower()
    if key not in POSES:
        raise ValueError("unknown pose {!r}; choose from: {}"
                         .format(name, ", ".join(sorted(POSES))))
    gx, gy, gz = POSES[key]
    command = "motion {} {} {}".format(gx, gy, gz)
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


def _pose_type(text: str) -> str:
    key = text.strip().lower()
    if key not in POSES:
        raise argparse.ArgumentTypeError(
            "unknown pose {!r}; choose from: {}".format(
                text, ", ".join(sorted(POSES))))
    return key


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
    sub = parser.add_subparsers(dest="command", metavar="COMMAND", required=True)

    p_session = sub.add_parser(
        "session",
        help="persistent reader plus stdin commands (logs keep streaming)")
    p_session.add_argument("--timeout", type=float, default=DEFAULT_CAPTURE_TIMEOUT,
                           metavar="SEC",
                           help="seconds to wait for a screenshot frame "
                                "(default %(default)s)")
    p_session.set_defaults(handler=cmd_session)

    p_monitor = sub.add_parser(
        "monitor",
        help="print firmware logs until Ctrl-C")
    p_monitor.set_defaults(handler=cmd_monitor)

    p_shot = sub.add_parser(
        "screenshot",
        help="send 'fb', save the next complete frame as PNG")
    p_shot.add_argument("output", nargs="?", default=None, metavar="[OUTPUT]",
                        help="PNG path; default: auto-named under --out-dir")
    p_shot.add_argument("--timeout", type=float, default=DEFAULT_CAPTURE_TIMEOUT,
                        metavar="SEC",
                        help="seconds to wait for the frame (default %(default)s)")
    p_shot.set_defaults(handler=cmd_screenshot)

    p_send = sub.add_parser(
        "send",
        help="send one firmware protocol command (words joined by spaces)")
    p_send.add_argument("command", nargs="+", metavar="command",
                        help="command text, e.g. 'send ping' or "
                             "'send motion 0 6 0 500'")
    p_send.add_argument("--timeout", type=float, default=DEFAULT_SEND_DRAIN,
                        metavar="SEC",
                        help="seconds to keep draining/printing logs after "
                             "sending (default %(default)s)")
    p_send.set_defaults(handler=cmd_send)

    p_drive = sub.add_parser(
        "drive",
        help="apply a simulated gravity pose via 'motion'")
    p_drive.add_argument("pose", type=_pose_type, metavar="POSE",
                         help="pose name: " + ", ".join(sorted(POSES)))
    p_drive.add_argument("--duration", type=int, default=0, metavar="MS",
                         help="motion duration in ms; 0 (default) sustains the "
                              "pose until 'release' or reboot")
    p_drive.add_argument("--timeout", type=float, default=DEFAULT_CAPTURE_TIMEOUT,
                         metavar="SEC",
                         help="seconds to wait for the screenshot frame "
                              "(default %(default)s)")
    p_drive.add_argument("--screenshot", nargs="?", const="", default=None,
                         metavar="[PATH]",
                         help="after a short settle, send 'fb' and save the "
                              "next frame; optional PATH (default: auto-named "
                              "under --out-dir)")
    p_drive.set_defaults(handler=cmd_drive)
    return parser


def cmd_monitor(dev: Device, args):
    dev.emit("[device_dev] monitoring {} @ {} baud; logs below. Ctrl-C to stop.\n"
             .format(dev.port, dev.baudrate))
    while True:
        if not dev.reader.is_alive():
            raise DeviceGone("serial reader stopped unexpectedly (device "
                             "unplugged or port error)")
        if dev._stop.wait(0.5):
            break
    return 0


def cmd_screenshot(dev: Device, args):
    ev = dev.capture(args.timeout)
    save_frame(dev, ev, args.out_dir, args.output)
    return 0


def cmd_send(dev: Device, args):
    if args.command[0] == "fb":
        raise UserError("raw 'fb' is request-scoped; use the 'screenshot' command")
    command = " ".join(args.command)
    dev.echo_send(command)
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        if not dev.reader.is_alive():
            raise DeviceGone("serial reader stopped unexpectedly (device "
                             "unplugged or port error)")
        time.sleep(0.05)
    return 0


def cmd_drive(dev: Device, args):
    try:
        command = pose_command(args.pose, args.duration)
    except ValueError as exc:
        raise UserError(str(exc)) from exc
    dev.echo_send(command)
    if args.screenshot is not None:
        time.sleep(POST_MOTION_SETTLE_S)
        path = args.screenshot or None
        ev = dev.capture(args.timeout)
        save_frame(dev, ev, args.out_dir, path)
    return 0


def cmd_session(dev: Device, args):
    dev.emit(
        "[device_dev] connected to {} @ {} baud; logs stream below.\n"
        "  commands: 'screenshot [path]', 'pose <name> [duration_ms]',\n"
        "  or any other line is sent to the board verbatim (ping, status,\n"
        "  motion, release, reset, reboot, yaw, axes, gain, tau). Ctrl-C exits.\n"
        .format(dev.port, dev.baudrate))

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
    saved = 0
    prompt_visible = False
    try:
        while True:
            if not dev.reader.is_alive():
                raise DeviceGone("serial reader stopped unexpectedly (device "
                                 "unplugged or port error)")
            if not prompt_visible:
                dev.emit("> ")
                prompt_visible = True
            try:
                event, line = stdin_events.get(timeout=0.1)
            except queue.Empty:
                continue
            prompt_visible = False
            if event == "eof":
                dev.print_err("stdin closed; ending session.")
                break
            if event == "error":
                raise UserError("stdin read failed: {}".format(line))

            text = line.strip()
            if not text:
                continue
            parts = text.split()
            try:
                if parts[0] == "screenshot":
                    if len(parts) > 2:
                        dev.print_err("usage: screenshot [path]")
                        continue
                    path = parts[1] if len(parts) == 2 else None
                    ev = dev.capture(args.timeout)
                    save_frame(dev, ev, args.out_dir, path)
                    saved += 1
                elif parts[0] == "pose":
                    if not (2 <= len(parts) <= 3):
                        dev.print_err("usage: pose <name> [duration_ms]; names: "
                                      + ", ".join(sorted(POSES)))
                        continue
                    try:
                        command = pose_command(parts[1],
                                               parts[2] if len(parts) == 3 else 0)
                    except ValueError as exc:
                        dev.print_err(str(exc))
                        continue
                    dev.echo_send(command)
                elif parts[0] == "fb":
                    dev.print_err("raw 'fb' is request-scoped; use 'screenshot [path]'")
                else:
                    dev.echo_send(text)
            except DeviceGone:
                raise
            except UserError as exc:
                dev.print_err(str(exc))     # keep the session alive
    except KeyboardInterrupt:
        dev.print_err("interrupted; ending session.")
    finally:
        with dev._print_lock:
            print("[device_dev] session ended; {} screenshot(s) saved.".format(saved))
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
        dev = Device(port=port, baudrate=args.baud, out_dir=args.out_dir)
    except (PortNotFound, UserError) as exc:
        _die(str(exc))
    try:
        return args.handler(dev, args)
    except UserError as exc:
        _die(str(exc))
    except KeyboardInterrupt:
        with dev._print_lock:
            print("\n[device_dev] interrupted; closing serial port.",
                  file=sys.stderr)
    finally:
        dev.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
