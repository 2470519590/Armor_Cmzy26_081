"""Armor CAN parameter tuner and frame monitor using ControlCAN.dll."""

from __future__ import annotations

import argparse
import csv
import ctypes
import secrets
import sys
import time
from ctypes import POINTER, Structure, byref, c_ubyte, c_uint, c_uint16
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

VCI_USBCAN2 = 4
STATUS_OK = 1
DEFAULT_DEVICE_INDEX = 0
DEFAULT_TX_CHANNEL = 1
DEFAULT_RX_CHANNEL = 1
DEFAULT_TIMING0 = 0x00
DEFAULT_TIMING1 = 0x1C
RX_BATCH_SIZE = 2500
P04 = 0x04
PARAM_SET_OFFSET = 0x0B
PARAM_ACK_OFFSET = 0x0C
THRESHOLD_MIN = 1000
THRESHOLD_MAX = 67108864


# These layouts are copied from the supplied 64-bit vendor Python example.
class VCI_INIT_CONFIG(Structure):
    _fields_ = [
        ("AccCode", c_uint),
        ("AccMask", c_uint),
        ("Reserved", c_uint),
        ("Filter", c_ubyte),
        ("Timing0", c_ubyte),
        ("Timing1", c_ubyte),
        ("Mode", c_ubyte),
    ]


class VCI_CAN_OBJ(Structure):
    _fields_ = [
        ("ID", c_uint),
        ("TimeStamp", c_uint),
        ("TimeFlag", c_ubyte),
        ("SendType", c_ubyte),
        ("RemoteFlag", c_ubyte),
        ("ExternFlag", c_ubyte),
        ("DataLen", c_ubyte),
        ("Data", c_ubyte * 8),
        ("Reserved", c_ubyte * 3),
    ]


class VCI_CAN_OBJ_ARRAY(Structure):
    _fields_ = [("SIZE", c_uint16), ("STRUCT_ARRAY", POINTER(VCI_CAN_OBJ))]

    def __init__(self, count: int):
        storage = (VCI_CAN_OBJ * count)()
        self._storage = storage
        self.STRUCT_ARRAY = ctypes.cast(storage, POINTER(VCI_CAN_OBJ))
        self.SIZE = count
        self.ADDR = self.STRUCT_ARRAY[0]


@dataclass(frozen=True)
class CanFrame:
    can_id: int
    data: bytes
    timestamp: int


def node_base(node_id: int) -> int:
    if not 1 <= node_id <= 4:
        raise ValueError("NodeID must be in range 1..4")
    return 0x130 + (node_id - 1) * 0x10


def parameter_id(node_id: int, offset: int) -> int:
    return node_base(node_id) + offset


def build_threshold_payload(value: int, sequence: int) -> bytes:
    if not THRESHOLD_MIN <= value <= THRESHOLD_MAX:
        raise ValueError(f"thr_hit must be in range {THRESHOLD_MIN}..{THRESHOLD_MAX}")
    if not 0 <= sequence <= 0xFF:
        raise ValueError("sequence must be in range 0..255")
    return bytes([P04]) + value.to_bytes(4, "little") + bytes([0, sequence, 0])


def decode_ack(frame: CanFrame) -> dict[str, int] | None:
    """Decode a valid-shaped P04 ACK; unrelated frames return None."""
    if len(frame.data) != 8 or frame.data[0] != P04 or frame.data[7] != 0:
        return None
    if frame.can_id < 0x130 or (frame.can_id - 0x130) % 0x10 != PARAM_ACK_OFFSET:
        return None
    return {
        "node": (frame.can_id - 0x130) // 0x10 + 1,
        "result": frame.data[1],
        "applied": int.from_bytes(frame.data[2:6], "little"),
        "sequence": frame.data[6],
    }


class ControlCan:
    def __init__(self, dll_path: Path, device_type: int, device_index: int,
                 tx_channel: int, rx_channel: int, timing0: int, timing1: int):
        if sys.platform != "win32":
            raise RuntimeError("ControlCAN.dll requires 64-bit Windows Python")
        if not dll_path.is_file():
            raise FileNotFoundError(f"ControlCAN.dll not found: {dll_path}")
        try:
            self.dll = ctypes.WinDLL(str(dll_path))
        except OSError as exc:
            raise RuntimeError(f"failed to load ControlCAN.dll: {exc}") from exc
        self.device_type = device_type
        self.device_index = device_index
        self.tx_channel = tx_channel
        self.rx_channel = rx_channel
        self.rx_buffer = VCI_CAN_OBJ_ARRAY(RX_BATCH_SIZE)
        self._opened = False
        try:
            self._open_channels(timing0, timing1)
        except Exception:
            self.close()
            raise

    def _check(self, name: str, result: int) -> None:
        if result != STATUS_OK:
            raise RuntimeError(f"{name} failed (return code {result})")

    def _open_channels(self, timing0: int, timing1: int) -> None:
        result = int(self.dll.VCI_OpenDevice(self.device_type, self.device_index, 0))
        self._check("VCI_OpenDevice", result)
        self._opened = True
        config = VCI_INIT_CONFIG(0x80000008, 0xFFFFFFFF, 0, 0, timing0, timing1, 0)
        for channel in sorted({self.tx_channel, self.rx_channel}):
            self._check(
                f"VCI_InitCAN(channel={channel})",
                int(self.dll.VCI_InitCAN(self.device_type, self.device_index, channel, byref(config))),
            )
            self._check(
                f"VCI_StartCAN(channel={channel})",
                int(self.dll.VCI_StartCAN(self.device_type, self.device_index, channel)),
            )

    def transmit(self, can_id: int, data: bytes) -> None:
        if not 0 <= can_id <= 0x7FF:
            raise ValueError("CAN ID must be a standard 11-bit ID")
        if not 0 <= len(data) <= 8:
            raise ValueError("classic CAN payload must be 0..8 bytes")
        payload = (c_ubyte * 8)(*data, *([0] * (8 - len(data))))
        reserved = (c_ubyte * 3)(0, 0, 0)
        frame = VCI_CAN_OBJ(can_id, 0, 0, 1, 0, 0, len(data), payload, reserved)
        self._check(
            "VCI_Transmit",
            int(self.dll.VCI_Transmit(
                self.device_type, self.device_index, self.tx_channel, byref(frame), 1
            )),
        )

    def receive(self) -> list[CanFrame]:
        count = int(self.dll.VCI_Receive(
            self.device_type, self.device_index, self.rx_channel,
            byref(self.rx_buffer.ADDR), RX_BATCH_SIZE, 0
        ))
        if count <= 0:
            return []
        return [
            CanFrame(
                int(self.rx_buffer.STRUCT_ARRAY[index].ID),
                bytes(self.rx_buffer.STRUCT_ARRAY[index].Data[:int(self.rx_buffer.STRUCT_ARRAY[index].DataLen)]),
                int(self.rx_buffer.STRUCT_ARRAY[index].TimeStamp),
            )
            for index in range(min(count, RX_BATCH_SIZE))
        ]

    def close(self) -> None:
        if getattr(self, "_opened", False):
            try:
                self.dll.VCI_CloseDevice(self.device_type, self.device_index)
            finally:
                self._opened = False

    def __enter__(self) -> "ControlCan":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


def format_frame(frame: CanFrame, marker: str = "") -> str:
    data = frame.data.hex(" ").upper()
    decoded = decode_ack(frame)
    detail = ""
    if decoded is not None:
        status = "OK" if decoded["result"] == 0 else f"INVALID({decoded['result']})"
        detail = (f" ACK node={decoded['node']} P04={decoded['applied']}"
                  f" result={status} seq={decoded['sequence']}")
    return (f"{marker}TS={datetime.now(timezone.utc).isoformat()} "
            f"CAN_TS={frame.timestamp} ID=0x{frame.can_id:03X} "
            f"DLC={len(frame.data)} DATA={data}{detail}")


def make_csv(path: str | None):
    if path is None:
        return None, None
    handle = open(path, "a", newline="", encoding="utf-8")
    writer = csv.writer(handle)
    if handle.tell() == 0:
        writer.writerow(["host_time_utc", "device_timestamp", "can_id", "dlc", "data", "decoded_ack"])
    return handle, writer


def resolve_channels(args: argparse.Namespace) -> tuple[int, int]:
    tx = args.tx_channel if args.tx_channel is not None else args.channel
    rx = args.rx_channel if args.rx_channel is not None else args.channel
    return (DEFAULT_TX_CHANNEL if tx is None else tx, DEFAULT_RX_CHANNEL if rx is None else rx)


def add_common(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--dll", type=Path, default=Path(__file__).parent / "vendor" / "ControlCAN.dll")
    parser.add_argument("--device", type=int, default=VCI_USBCAN2, help="ControlCAN device type (default: 4 / USB-CAN2)")
    parser.add_argument("--device-index", type=int, default=DEFAULT_DEVICE_INDEX, help="device index (default: 0)")
    parser.add_argument("--channel", type=int, help="set both TX and RX channels")
    parser.add_argument("--tx-channel", type=int, help="transmit channel; overrides --channel")
    parser.add_argument("--rx-channel", type=int, help="receive channel; overrides --channel")
    parser.add_argument("--timing0", type=lambda value: int(value, 0), default=DEFAULT_TIMING0)
    parser.add_argument("--timing1", type=lambda value: int(value, 0), default=DEFAULT_TIMING1)


def receive_frames(can: ControlCan, deadline: float, csv_writer=None,
                   node_filter: int | None = None, print_all: bool = True) -> list[CanFrame]:
    received = []
    while time.monotonic() < deadline:
        frames = can.receive()
        if not frames:
            time.sleep(0.002)
            continue
        for frame in frames:
            decoded = decode_ack(frame)
            if csv_writer is not None:
                csv_writer.writerow([
                    datetime.now(timezone.utc).isoformat(), frame.timestamp,
                    f"0x{frame.can_id:03X}", len(frame.data), frame.data.hex(" ").upper(),
                    "" if decoded is None else str(decoded),
                ])
            matches_node = decoded is not None and decoded["node"] == node_filter
            if print_all or matches_node:
                print(format_frame(frame, "*** " if matches_node else ""), flush=True)
            received.append(frame)
    return received


def open_can(args: argparse.Namespace) -> ControlCan:
    tx, rx = resolve_channels(args)
    return ControlCan(args.dll, args.device, args.device_index, tx, rx, args.timing0, args.timing1)


def command_listen(args: argparse.Namespace) -> int:
    handle, writer = make_csv(args.csv)
    try:
        with open_can(args) as can:
            print("Listening. Press Ctrl+C to stop.")
            while True:
                receive_frames(can, time.monotonic() + 1.0, writer)
    except KeyboardInterrupt:
        return 0
    finally:
        if handle is not None:
            handle.close()


def command_monitor(args: argparse.Namespace) -> int:
    node_base(args.node)
    handle, writer = make_csv(args.csv)
    try:
        with open_can(args) as can:
            print(f"Monitoring NodeID {args.node}. Press Ctrl+C to stop.")
            while True:
                receive_frames(can, time.monotonic() + 1.0, writer, args.node, print_all=False)
    except KeyboardInterrupt:
        return 0
    finally:
        if handle is not None:
            handle.close()


def command_set_threshold(args: argparse.Namespace) -> int:
    sequence = secrets.randbelow(256)
    payload = build_threshold_payload(args.value, sequence)
    set_id = parameter_id(args.node, PARAM_SET_OFFSET)
    ack_id = set_id + 1
    with open_can(args) as can:
        print(f"TX ID=0x{set_id:03X} DATA={payload.hex(' ').upper()} (RAM only)")
        can.transmit(set_id, payload)
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            for frame in can.receive():
                print(format_frame(frame))
                ack = decode_ack(frame)
                if frame.can_id != ack_id or ack is None or ack["sequence"] != sequence:
                    continue
                if ack["result"] != 0:
                    print(f"ERROR: ACK rejected parameter (result={ack['result']})", file=sys.stderr)
                    return 3
                if ack["applied"] != args.value:
                    print(f"ERROR: ACK applied value {ack['applied']} != requested {args.value}", file=sys.stderr)
                    return 4
                print(f"ACK OK node={args.node} P04={ack['applied']} sequence={sequence}")
                return 0
            time.sleep(0.002)
    print(f"ERROR: timeout waiting for ACK ID=0x{ack_id:03X} sequence={sequence}", file=sys.stderr)
    return 2


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Armor P04 thr_hit CAN tuner and monitor")
    subparsers = parser.add_subparsers(dest="command", required=True)

    listen = subparsers.add_parser("listen", help="print and optionally log all received CAN frames")
    add_common(listen)
    listen.add_argument("--csv", help="append received frames to a CSV file")
    listen.set_defaults(handler=command_listen)

    monitor = subparsers.add_parser("monitor", help="highlight P04 ACK frames for one NodeID")
    add_common(monitor)
    monitor.add_argument("--node", type=int, required=True, help="armor NodeID 1..4")
    monitor.add_argument("--csv", help="append received frames to a CSV file")
    monitor.set_defaults(handler=command_monitor)

    tune = subparsers.add_parser("set-thr", aliases=["set-threshold"], help="write RAM-only P04 thr_hit and wait for ACK")
    add_common(tune)
    tune.add_argument("--node", type=int, required=True, help="armor NodeID 1..4")
    tune.add_argument("--value", type=int, required=True, help="thr_hit, 1000..67108864")
    tune.add_argument("--timeout", type=float, default=1.0, help="ACK timeout in seconds")
    tune.set_defaults(handler=command_set_threshold)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        return args.handler(args)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
