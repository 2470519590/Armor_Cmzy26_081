"""CAN firmware updater for STM32L432 armor boards using ControlCAN.dll."""

from __future__ import annotations

import argparse
import secrets
import sys
import time
import zlib
from pathlib import Path

from armor_can_tool import add_common, open_can

CMD = 0x100
DATA = 0x101
ACK = 0x102
STATUS = 0x103
DISCOVER = 1
SELECT = 2
ENTER = 3
BEGIN = 4
END = 5
INFO = 7
DATA_ACK = 9
DISCOVER_REPLY = 10
APP_ENUM_START = 0x120
APP_ENUM_ANNOUNCE = 0x121
L431_MAINT_CMD = 0x110
L431_MAINT_ACK = 0x111
L431_MAINT_MAGIC = 0xA5
L431_MAINT_ACK_MAGIC = 0x5A
SLOT_SIZE = 48 * 1024
# A smaller burst coexists better with an active L431/business bus.  The
# isolated USB-CAN case can still be changed locally for benchmarking.
DATA_WINDOW = 32
OFFLINE_GRACE_SECONDS = 3.0


def frames(can, seconds: float):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        batch = can.receive()
        if batch:
            yield from batch
        else:
            time.sleep(0.002)


def uid_crc(uid: bytes) -> int:
    crc = 0xFFFF
    for byte in uid:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def discover(can, session: int, seconds: float = 0.7) -> list[str]:
    can.transmit(CMD, bytes([DISCOVER, session, 0, 0, 0, 0, 0, 0]))
    parts: dict[int, dict[int, bytes]] = {}
    for frame in frames(can, seconds):
        if frame.can_id != STATUS or len(frame.data) != 8:
            continue
        d = frame.data
        if d[0] != DISCOVER_REPLY or d[1] != session or d[2] > 3:
            continue
        parts.setdefault(int.from_bytes(d[6:8], "little"), {})[d[2]] = bytes(d[3:6])
    found = []
    for crc, value in parts.items():
        if len(value) == 4:
            uid = b"".join(value[index] for index in range(4))
            if uid_crc(uid) == crc:
                found.append(uid.hex())
    return sorted(set(found))


def discover_targets(can, targets: set[str]) -> set[str]:
    """Re-probe only the fixed initial target set; ignore newly seen boards."""
    probe_session = secrets.randbelow(255) + 1
    return set(discover(can, probe_session)).intersection(targets)


def set_l431_silent(can, session: int, silent: bool, timeout: float = 2.0) -> None:
    command = 1 if silent else 0
    can.transmit(L431_MAINT_CMD, bytes([L431_MAINT_MAGIC, session, command, 0, 0, 0, 0, 0]))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for frame in can.receive():
            if frame.can_id != L431_MAINT_ACK or len(frame.data) != 8:
                continue
            d = frame.data
            if (d[0] == L431_MAINT_ACK_MAGIC and d[1] == session and
                    d[2] == command and d[3] == 0):
                return
        time.sleep(0.002)
    state = "silent" if silent else "normal"
    raise TimeoutError(f"no L431 {state} acknowledgement")


def restore_l431(can, session: int) -> None:
    last_error = None
    for _ in range(3):
        try:
            set_l431_silent(can, session, False, timeout=0.8)
            return
        except TimeoutError as exc:
            last_error = exc
            time.sleep(0.1)
    raise last_error


def verify_applications(can, uids: list[bytes], session: int, timeout: float = 2.0) -> None:
    """Prove the reset landed in the application, without requiring NodeID.

    The application responds to the existing armor enumeration start (0x120)
    with four UID fragments on 0x121.  The bootloader intentionally ignores
    0x120, so receiving and validating all fragments is a direct application
    liveness check and works with or without an L431 node.
    """
    wanted = {uid for uid in uids}
    found: set[bytes] = set()
    candidates: dict[int, dict[int, bytes]] = {}
    deadline = time.monotonic() + timeout
    next_probe = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_probe:
            can.transmit(APP_ENUM_START, bytes([session]))
            next_probe = now + 0.25
        for frame in can.receive():
            if frame.can_id != APP_ENUM_ANNOUNCE or len(frame.data) != 8:
                continue
            d = frame.data
            if d[0] != session or d[1] > 3:
                continue
            token = int.from_bytes(d[5:8], "little")
            candidates.setdefault(token, {})[d[1]] = bytes(d[2:5])
        for parts in candidates.values():
            if len(parts) == 4:
                candidate = b"".join(parts[index] for index in range(4))
                if candidate in wanted:
                    found.add(candidate)
        if found == wanted:
            return
        time.sleep(0.002)
    missing = ",".join(uid.hex() for uid in sorted(wanted - found))
    raise RuntimeError(f"application enumeration/UID verification failed; missing={missing}")


def await_ack(can, command: int, session: int, timeout: float = 2.0) -> bytes:
    seen = []
    for frame in frames(can, timeout):
        if frame.can_id in (ACK, STATUS) and len(frame.data) == 8:
            seen.append((frame.can_id, frame.data.hex()))
            if len(seen) > 6:
                seen.pop(0)
        if frame.can_id == ACK and len(frame.data) == 8 and frame.data[0] == command and frame.data[2] == session:
            if frame.data[1] != 0:
                raise RuntimeError(f"bootloader rejected command {command}: status {frame.data[1]}")
            return frame.data
    detail = "" if not seen else f"; recent={seen}"
    raise TimeoutError(f"no ACK for command {command}{detail}")


def await_data_window(can, session: int, target_offset: int, timeout: float = 2.0) -> tuple[bool, int]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for frame in can.receive():
            if frame.can_id != ACK or len(frame.data) != 8:
                continue
            d = frame.data
            if d[0] != DATA_ACK or d[2] != session:
                continue
            offset = int.from_bytes(d[3:7], "little")
            if d[1] != 0:
                if d[1] == 1:
                    return False, offset
                raise RuntimeError(f"bootloader rejected data window: status {d[1]}")
            if offset >= target_offset:
                return True, offset
        time.sleep(0.001)
    raise TimeoutError(f"no window ACK at offset {target_offset}")


def select(can, uid: bytes, session: int, expect_ack: bool) -> None:
    for part in range(3):
        can.transmit(CMD, bytes([SELECT, part, session]) + uid[part * 4 : part * 4 + 4] + b"\x00")
        time.sleep(0.005)
    if expect_ack:
        await_ack(can, SELECT, session)
    else:
        can.transmit(CMD, bytes([ENTER, session, 0, 0, 0, 0, 0, 0]))


def boot_info(can, session: int) -> tuple[int, int]:
    can.transmit(CMD, bytes([INFO, session, 0, 0, 0, 0, 0, 0]))
    for frame in frames(can, 0.7):
        if frame.can_id == STATUS and len(frame.data) == 8:
            d = frame.data
            if d[0] == INFO and d[1] == session:
                return d[2], d[3]
    raise TimeoutError("no bootloader INFO response")


def command(can, session: int, field: int, value: int, slot: int = 0) -> None:
    can.transmit(CMD, bytes([BEGIN, session, field]) + value.to_bytes(4, "little") + bytes([slot]))
    await_ack(can, BEGIN, session, timeout=30.0 if field == 2 else 2.0)


def update(can, uid: bytes, slot: int | None, image: bytes | None,
           session: int, image_dir: Path) -> int:
    select(can, uid, session, expect_ack=False)
    time.sleep(0.25)
    select(can, uid, session, expect_ack=True)
    if slot is None:
        pending, active = boot_info(can, session)
        if pending:
            raise RuntimeError("board has a pending trial image; power-cycle and retry")
        slot = 1 - active
    if image is None:
        slot_name = "A" if slot == 0 else "B"
        image = (image_dir / f"build_{slot_name}" /
                 f"Armor_Cmzy26_081_{slot_name}.bin").read_bytes()
    if not 8 <= len(image) <= SLOT_SIZE:
        raise ValueError("image must fit the 48 KB application slot")
    command(can, session, 0, len(image), slot)
    command(can, session, 1, zlib.crc32(image))
    command(can, session, 2, 0)
    total_packets = (len(image) + 5) // 6
    for window_start in range(0, total_packets, DATA_WINDOW):
        window_end = min(window_start + DATA_WINDOW, total_packets)
        target_offset = min(window_end * 6, len(image))
        for attempt in range(10):
            batch = []
            for packet in range(window_start, window_end):
                offset = packet * 6
                chunk = image[offset : offset + 6]
                payload = bytes([packet & 0xFF, packet >> 8]) + chunk.ljust(6, bytes([0xFF]))
                batch.append((DATA, payload))
            can.transmit_many(batch)
            try:
                acked, _received_offset = await_data_window(can, session, target_offset)
                if acked:
                    break
                print(f"window retry start={window_start} target={target_offset} status=invalid", flush=True)
                if attempt == 9:
                    raise RuntimeError(f"window retry limit reached at packet {window_start}")
            except TimeoutError:
                print(f"window retry start={window_start} target={target_offset} status=timeout", flush=True)
                if attempt == 9:
                    raise
        if target_offset % 2048 == 0 or target_offset == len(image):
            print(f"{target_offset}/{len(image)} bytes", flush=True)
    can.transmit(CMD, bytes([END, session, 0, 0, 0, 0, 0, 0]))
    await_ack(can, END, session, timeout=5.0)
    print("image verified; waiting for trial boot", flush=True)
    return len(image)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common(parser)
    parser.add_argument("--list", action="store_true", help="discover connected boards")
    parser.add_argument("--uid", help="12-byte UID; omit to update all discovered boards")
    parser.add_argument("--slot", choices=("A", "B"), help="target slot matching image link address")
    parser.add_argument("--image", type=Path, help="slot-specific application .bin; omit for automatic build_A/build_B selection")
    args = parser.parse_args()
    try:
        with open_can(args) as can:
            session = secrets.randbelow(255) + 1
            l431_controlled = False
            try:
                # A missing L431 is a supported topology.  Probe briefly;
                # only require the normal 2 s timeout when restoring a node
                # that already acknowledged the enter command.
                set_l431_silent(can, session, True, timeout=0.35)
                l431_controlled = True
                print("L431 silent=ON", flush=True)
            except TimeoutError:
                print("L431 not detected; continuing armor-only upgrade", flush=True)
            try:
                found = discover(can, session)
                for uid in found:
                    print(f"UID={uid}")
                if args.list:
                    return 0
                targets = [args.uid.lower()] if args.uid else found
                if not targets:
                    raise RuntimeError("no boards discovered")
                image_dir = Path(__file__).resolve().parents[1]
                if args.uid and args.uid.lower() not in found:
                    raise ValueError("target UID must be one of the discovered 12-byte UIDs")
                completed = []
                initial_targets = list(targets)
                pending = set(initial_targets)
                warned_offline: set[str] = set()
                offline_since: float | None = None
                while pending:
                    live = discover_targets(can, pending)
                    progressed = False
                    for target in initial_targets:
                        if target not in pending:
                            continue
                        if target not in live:
                            if target not in warned_offline:
                                print(f"warning: target offline; deferred UID={target}", flush=True)
                                warned_offline.add(target)
                            continue
                        uid = bytes.fromhex(target)
                        slot = None if args.slot is None else (0 if args.slot == "A" else 1)
                        image = args.image.read_bytes() if args.image else None
                        start = time.monotonic()
                        try:
                            transferred = update(can, uid, slot, image, session, image_dir)
                        except TimeoutError as exc:
                            print(f"warning: target disappeared during upgrade; deferred UID={target}; detail={exc}", flush=True)
                            warned_offline.add(target)
                            continue
                        elapsed = time.monotonic() - start
                        rate = transferred / 1024.0 / elapsed if elapsed > 0 else 0.0
                        completed.append((target, elapsed, rate))
                        pending.remove(target)
                        progressed = True
                        print(f"updated UID={target}; transfer complete; elapsed={elapsed:.2f}s; average={rate:.2f} KB/s", flush=True)
                    if not pending:
                        break
                    if live:
                        offline_since = None
                    elif offline_since is None:
                        offline_since = time.monotonic()
                    if offline_since is not None and time.monotonic() - offline_since >= OFFLINE_GRACE_SECONDS:
                        print(f"warning: offline grace expired; leaving={len(pending)}", flush=True)
                        break
                    if not progressed:
                        print(f"warning: no pending target online; waiting={len(pending)}", flush=True)
                        time.sleep(1.0)
                if completed:
                    verify_applications(can, [bytes.fromhex(target) for target, _, _ in completed], session)
                for target, elapsed, rate in completed:
                    print(f"application=verified UID={target}; elapsed={elapsed:.2f}s; average={rate:.2f} KB/s", flush=True)
                missing_targets = [target for target in initial_targets if target not in {item[0] for item in completed}]
                missing = len(missing_targets)
                print(f"summary: initial={len(initial_targets)} updated={len(completed)} missing={missing}", flush=True)
                if missing_targets:
                    print(f"warning: not updated UID={','.join(missing_targets)}", flush=True)
            finally:
                if l431_controlled:
                    restore_l431(can, session)
                    print("L431 silent=OFF", flush=True)
        return 0
    except (OSError, ValueError, RuntimeError, TimeoutError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
