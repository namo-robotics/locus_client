#!/usr/bin/env python3
"""Live viewer / config demo for Locus, a dual ZED-F9P + IMU receiver.

Shows one continuously-updating status line per sensor (GPS1, GPS2,
IMU) with measured update rates; device log messages and UBX ACKs
scroll above the status block.

Usage:
    python example.py /dev/ttyACM0                 # live view + NTRIP to GPS1
    python example.py /dev/ttyACM0 --no-ntrip      # live view only
    python example.py /dev/ttyACM0 --set-rate 10   # also set GPS1+GPS2 to 10 Hz
    python example.py /dev/ttyACM0 --ntrip PDA1    # different mountpoint

NTRIP is on by default (see NTRIP_DEFAULTS): RTCM corrections from the
caster are forwarded to GPS1, which then reports an absolute RTK
position (watch carr= in the GPS1 line) while still acting as moving
base for GPS2's relative heading. GPS1's position is reported back to
the caster as NMEA GGA every gga_interval seconds. Find other bases at
https://centipede.fr.

Requires pyserial. If pyubx2 is installed, UBX frames are decoded to
readable summaries; otherwise their class/id/length is printed.
"""

import argparse
import base64
import math
import queue
import shutil
import socket
import struct
import sys
import threading
import time

import serial

import dualgps

try:
    from pyubx2 import UBXReader
except ImportError:
    UBXReader = None

DRAW_INTERVAL = 0.05  # redraw at most 20x/s; data arrives much faster

NTRIP_CHUNK = 256          # firmware MUX_CMD_MAX per host->device frame
NTRIP_CHUNK_GAP_S = 0.005  # pace frames so the device USB ring never floods
NTRIP_RECONNECT_S = 5.0
NTRIP_STALL_S = 30.0       # no caster data for this long -> reconnect

NTRIP_DEFAULTS = {
    "host": "crtk.net",
    "port": 2101,
    "mountpoint": "PDA1",
    "user": "centipede",
    "password": "centipede",
    "version": "2.0",
    "user_agent": "NTRIP-Locus/1.0",
    "gga_interval": 15.0,   # seconds between GGA uploads (0 disables)
}


class _ChunkedDecoder:
    """Incremental HTTP/1.1 chunked transfer decoder (NTRIP 2.0 streams)."""

    def __init__(self):
        self._buf = b""
        self._need = None   # bytes needed for current chunk incl. CRLF
        self._size = 0

    def feed(self, data: bytes) -> bytes:
        self._buf += data
        out = b""
        while True:
            if self._need is None:
                nl = self._buf.find(b"\r\n")
                if nl < 0:
                    break
                try:
                    self._size = int(self._buf[:nl].split(b";")[0], 16)
                except ValueError:
                    raise RuntimeError("bad chunk header from caster")
                if self._size == 0:
                    raise RuntimeError("caster ended the chunked stream")
                self._buf = self._buf[nl + 2:]
                self._need = self._size + 2  # chunk data + trailing CRLF
            if len(self._buf) < self._need:
                break
            out += self._buf[:self._size]
            self._buf = self._buf[self._need:]
            self._need = None
        return out


def build_gga(lat: float, lon: float, alt_m: float, sats: int) -> bytes:
    """Minimal NMEA GGA sentence for caster position reporting."""
    t = time.gmtime()
    lat_d, lon_d = int(abs(lat)), int(abs(lon))
    lat_m = (abs(lat) - lat_d) * 60
    lon_m = (abs(lon) - lon_d) * 60
    body = (f"GPGGA,{t.tm_hour:02d}{t.tm_min:02d}{t.tm_sec:02d}.00,"
            f"{lat_d:02d}{lat_m:09.6f},{'N' if lat >= 0 else 'S'},"
            f"{lon_d:03d}{lon_m:09.6f},{'E' if lon >= 0 else 'W'},"
            f"1,{min(sats, 99):02d},1.0,{alt_m:.1f},M,0.0,M,,")
    ck = 0
    for c in body:
        ck ^= ord(c)
    return f"${body}*{ck:02X}\r\n".encode()


class _MountNotFound(Exception):
    """Caster answered with a SOURCETABLE: the mountpoint doesn't exist."""

    def __init__(self, table: str):
        super().__init__("mountpoint not found")
        self.table = table


class NtripClient(threading.Thread):
    """Streams RTCM from an NTRIP v1 caster to GPS1 via the USB mux.

    Runs as a daemon thread and is the only serial writer once started.
    Status messages go into `events` (drained by the main loop); received
    byte counts accumulate in `rx_bytes` under `lock`. `get_pos` is an
    optional callable returning the latest (lat, lon) or None, used to
    suggest nearby mountpoints when the configured one doesn't exist.
    """

    def __init__(self, ser, host, port, mount, user, password, get_pos=None,
                 version=NTRIP_DEFAULTS["version"],
                 user_agent=NTRIP_DEFAULTS["user_agent"],
                 gga_interval=NTRIP_DEFAULTS["gga_interval"]):
        super().__init__(daemon=True)
        self.ser = ser
        self.host = host
        self.port = port
        self.mount = mount
        self.auth = base64.b64encode(f"{user}:{password}".encode()).decode()
        self.get_pos = get_pos
        self.version = version
        self.user_agent = user_agent
        self.gga_interval = gga_interval
        self.events = queue.Queue()
        self.lock = threading.Lock()
        self.rx_bytes = 0

    def take_rx_bytes(self) -> int:
        with self.lock:
            n, self.rx_bytes = self.rx_bytes, 0
        return n

    def run(self):
        while True:
            try:
                self._stream()
            except _MountNotFound as exc:
                self.events.put(f"NTRIP: mountpoint '{self.mount}' not on "
                                f"{self.host}, giving up")
                self._suggest_mounts(exc.table)
                return
            except (OSError, RuntimeError) as exc:
                self.events.put(f"NTRIP: {exc}; retrying in "
                                f"{NTRIP_RECONNECT_S:.0f}s")
                time.sleep(NTRIP_RECONNECT_S)

    def _request(self) -> bytes:
        if self.version.startswith("2"):
            return (f"GET /{self.mount} HTTP/1.1\r\n"
                    f"Host: {self.host}\r\n"
                    f"Ntrip-Version: Ntrip/2.0\r\n"
                    f"User-Agent: {self.user_agent}\r\n"
                    f"Authorization: Basic {self.auth}\r\n"
                    f"Connection: close\r\n"
                    f"\r\n").encode()
        return (f"GET /{self.mount} HTTP/1.0\r\n"
                f"Host: {self.host}\r\n"
                f"User-Agent: {self.user_agent}\r\n"
                f"Authorization: Basic {self.auth}\r\n"
                f"\r\n").encode()

    @staticmethod
    def _read_head(sock):
        """Read the response head: (status, headers dict, leftover bytes).

        NTRIP v1 casters answer "ICY 200 OK" with no header block and
        stream immediately; HTTP-style answers carry headers up to a
        blank line.
        """
        buf = b""
        while b"\r\n" not in buf:
            data = sock.recv(1024)
            if not data:
                raise RuntimeError("caster closed during handshake")
            buf += data
        status, _, rest = buf.partition(b"\r\n")
        if status.startswith(b"ICY"):
            return status, {}, rest
        buf = rest
        while b"\r\n\r\n" not in buf:
            data = sock.recv(1024)
            if not data:
                raise RuntimeError("caster closed during handshake")
            buf += data
        head, _, rest = buf.partition(b"\r\n\r\n")
        headers = {}
        for line in head.decode(errors="replace").splitlines():
            key, sep, val = line.partition(":")
            if sep:
                headers[key.strip().lower()] = val.strip()
        return status, headers, rest

    def _stream(self):
        with socket.create_connection((self.host, self.port),
                                      timeout=10) as sock:
            sock.sendall(self._request())
            status, headers, rest = self._read_head(sock)

            if (b"SOURCETABLE" in status
                    or "sourcetable" in headers.get("content-type", "")):
                raise _MountNotFound(self._read_table(sock, rest))
            if b"401" in status or b"403" in status:
                raise RuntimeError("caster rejected credentials")
            if b"404" in status:
                raise _MountNotFound("")
            if b"200" not in status:
                raise RuntimeError(f"caster said: {status.decode(errors='replace')}")

            decoder = (_ChunkedDecoder()
                       if headers.get("transfer-encoding", "").lower() == "chunked"
                       else None)
            self.events.put(f"NTRIP: connected, streaming {self.mount} "
                            f"from {self.host} (v{self.version})")

            sock.settimeout(2)
            last_data = time.monotonic()
            last_gga = 0.0
            self._forward(decoder.feed(rest) if decoder else rest)
            while True:
                # GGA first so the caster gets a position right after
                # connecting, then again every gga_interval seconds
                if self.gga_interval and self.get_pos:
                    now = time.monotonic()
                    if now - last_gga >= self.gga_interval:
                        pos = self.get_pos()
                        if pos is not None:
                            lat, lon, alt, sats = pos
                            sock.sendall(build_gga(lat, lon, alt, sats))
                            last_gga = now

                try:
                    data = sock.recv(2048)
                    if not data:
                        raise RuntimeError("caster closed the stream")
                    self._forward(decoder.feed(data) if decoder else data)
                    last_data = time.monotonic()
                except socket.timeout:
                    if time.monotonic() - last_data > NTRIP_STALL_S:
                        raise RuntimeError("caster stopped sending data")

    @staticmethod
    def _read_table(sock, first: bytes) -> str:
        """Drain the rest of a SOURCETABLE response (caster closes after)."""
        table = first
        try:
            while len(table) < 1_000_000:
                data = sock.recv(8192)
                if not data:
                    break
                table += data
        except OSError:
            pass
        return table.decode(errors="replace")

    def _suggest_mounts(self, table: str, count: int = 3):
        """Log the nearest mountpoints to the current GPS1 position."""
        bases = []  # (mount, identifier, lat, lon)
        for line in table.splitlines():
            fields = line.split(";")
            if fields[0] != "STR" or len(fields) < 11:
                continue
            try:
                bases.append((fields[1], fields[2],
                              float(fields[9]), float(fields[10])))
            except ValueError:
                continue
        if not bases:
            return
        # The board usually has a fix within seconds; wait briefly for one
        # so we can rank by distance
        deadline = time.monotonic() + 20
        pos = self.get_pos() if self.get_pos else None
        while pos is None and time.monotonic() < deadline:
            time.sleep(0.5)
            pos = self.get_pos() if self.get_pos else None
        if pos is None:
            self.events.put(f"NTRIP: caster lists {len(bases)} bases "
                            f"(no GPS fix yet, can't rank by distance)")
            return
        lat0, lon0 = pos[0], pos[1]

        def dist_km(base):
            dx = (base[3] - lon0) * math.cos(math.radians(lat0)) * 111.32
            dy = (base[2] - lat0) * 110.57
            return math.hypot(dx, dy)

        nearest = sorted(bases, key=dist_km)[:count]
        pretty = ", ".join(f"{m} ({ident}, {dist_km((m, ident, la, lo)):.0f}km)"
                           for m, ident, la, lo in nearest)
        self.events.put(f"NTRIP: nearest bases: {pretty}")

    def _forward(self, data: bytes):
        for i in range(0, len(data), NTRIP_CHUNK):
            self.ser.write(dualgps.frame(dualgps.CH_GPS1_CMD,
                                         data[i:i + NTRIP_CHUNK]))
            time.sleep(NTRIP_CHUNK_GAP_S)
        with self.lock:
            self.rx_bytes += len(data)


def build_valset_rate(rate_hz: int) -> bytes:
    """UBX-CFG-VALSET (RAM layer) setting CFG-RATE-MEAS to 1000/rate_hz ms."""
    meas_ms = round(1000 / rate_hz)
    payload = bytes([0x00, 0x01, 0x00, 0x00])            # version, RAM layer
    payload += struct.pack("<IH", 0x30210001, meas_ms)   # CFG-RATE-MEAS key
    body = bytes([0x06, 0x8A]) + struct.pack("<H", len(payload)) + payload
    ck_a = ck_b = 0
    for b in body:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return b"\xb5\x62" + body + bytes([ck_a, ck_b])


def rtcm_type(payload: bytes) -> int:
    """RTCM3 message type from a complete frame (12 bits after header)."""
    if len(payload) < 5 or payload[0] != 0xD3:
        return 0
    return (payload[3] << 4) | (payload[4] >> 4)


def parse_ubx(payload: bytes):
    """Return a pyubx2 message, or None if undecodable/unavailable."""
    if UBXReader is None:
        return None
    try:
        return UBXReader.parse(payload)
    except Exception:
        return None


def describe_pvt(msg) -> str:
    carr = {0: "NONE", 1: "FLOAT", 2: "FIXED"}.get(msg.carrSoln, "?")
    return (f"PVT fix={msg.fixType} carr={carr} sats={msg.numSV} "
            f"lat={msg.lat:.7f} lon={msg.lon:.7f} "
            f"hMSL={msg.hMSL / 1000:.1f}m hAcc={msg.hAcc / 1000:.2f}m")


def describe_relposned(msg) -> str:
    # pyubx2 folds the high-precision components in already;
    # relPos* are cm, acc* are mm, heading is degrees
    carr = {0: "NONE", 1: "FLOAT", 2: "FIXED"}.get(msg.carrSoln, "?")
    # Surface the failure flags: with relPosValid=0 the receiver zeroes
    # every relPos field, so the reason matters more than the numbers
    problems = []
    if not getattr(msg, "relPosValid", 1):
        problems.append("INVALID")
    if not getattr(msg, "diffSoln", 1):
        problems.append("noDiff")   # epoch computed without corrections
    if not getattr(msg, "isMoving", 1):
        problems.append("static")   # rover not in moving-base mode
    if getattr(msg, "refPosMiss", 0):
        problems.append("refPosMiss")
    if getattr(msg, "refObsMiss", 0):
        problems.append("refObsMiss")
    if not getattr(msg, "relPosHeadingValid", 1):
        problems.append("hdg?")
    extra = (" [" + ",".join(problems) + "]") if problems else ""
    return (f"REL N={msg.relPosN / 100:.4f} E={msg.relPosE / 100:.4f} "
            f"D={msg.relPosD / 100:.4f} len={msg.relPosLength / 100:.4f}m "
            f"hdg={msg.relPosHeading:.2f} carr={carr}{extra}")


class LiveDisplay:
    """Three status lines redrawn in place; log text scrolls above them."""

    def __init__(self, labels):
        self.labels = labels
        self.status = {label: "waiting..." for label in labels}
        self._block_live = False  # status block currently on screen

    def _erase_block(self) -> str:
        if not self._block_live:
            return ""
        self._block_live = False
        return f"\x1b[{len(self.labels)}F\x1b[0J"  # cursor up N, clear below

    def log(self, text: str) -> None:
        sys.stdout.write(self._erase_block() + text + "\n")
        self.draw()

    def draw(self) -> None:
        width = shutil.get_terminal_size().columns
        out = self._erase_block()
        for label in self.labels:
            line = f"{label:5s} {self.status[label]}"
            out += line[:width] + "\n"
        sys.stdout.write(out)
        sys.stdout.flush()
        self._block_live = True


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("port", help="serial port, e.g. /dev/ttyACM0")
    ap.add_argument("--set-rate", type=int, metavar="HZ",
                    help="send CFG-VALSET to both GPS setting nav rate in Hz")
    ap.add_argument("--ntrip", metavar="MOUNTPOINT",
                    default=NTRIP_DEFAULTS["mountpoint"],
                    help="NTRIP mountpoint for GPS1 corrections "
                         "(default: %(default)s)")
    ap.add_argument("--no-ntrip", action="store_true",
                    help="run without NTRIP corrections")
    ap.add_argument("--ntrip-host", default=NTRIP_DEFAULTS["host"],
                    help="NTRIP caster host (default: %(default)s)")
    ap.add_argument("--ntrip-port", type=int, default=NTRIP_DEFAULTS["port"],
                    help="NTRIP caster port (default: %(default)s)")
    ap.add_argument("--ntrip-user", default=NTRIP_DEFAULTS["user"],
                    help="NTRIP username (default: %(default)s)")
    ap.add_argument("--ntrip-pass", default=NTRIP_DEFAULTS["password"],
                    help="NTRIP password (default: %(default)s)")
    ap.add_argument("--ntrip-version", default=NTRIP_DEFAULTS["version"],
                    choices=["1.0", "2.0"],
                    help="NTRIP protocol version (default: %(default)s)")
    args = ap.parse_args()

    ser = serial.Serial(args.port, timeout=0.1)
    framer = dualgps.MuxFramer()
    disp = LiveDisplay(["GPS1", "GPS2", "REL", "RTCM", "IMU"])

    if args.set_rate:
        ubx = build_valset_rate(args.set_rate)
        ser.write(dualgps.frame(dualgps.CH_GPS1_CMD, ubx))
        ser.write(dualgps.frame(dualgps.CH_GPS2_CMD, ubx))
        disp.log(f"sent CFG-VALSET rate={args.set_rate}Hz to GPS1+GPS2, "
                 f"NAKs will be shown")

    # Latest GPS1 (lat, lon, alt_m, sats), shared with the NTRIP thread
    gps1_pos = [None]

    # Started after any --set-rate write: the NTRIP thread must be the
    # only serial writer once it is running
    ntrip = None
    if args.no_ntrip:
        disp.log("NTRIP: disabled, GPS1 runs without RTK corrections")
    else:
        disp.log(f"NTRIP: enabled, corrections for GPS1 from "
                 f"{args.ntrip_host}:{args.ntrip_port}/{args.ntrip}")
        ntrip = NtripClient(ser, args.ntrip_host, args.ntrip_port,
                            args.ntrip, args.ntrip_user, args.ntrip_pass,
                            get_pos=lambda: gps1_pos[0],
                            version=args.ntrip_version)
        ntrip.start()

    # Latest data summary and per-second message counters for each line
    detail = {"GPS1": "waiting...", "GPS2": "waiting...",
              "REL": "waiting...", "IMU": "waiting..."}
    counts = {"GPS1": 0, "GPS2": 0, "REL": 0, "IMU": 0}
    rates = {"GPS1": 0.0, "GPS2": 0.0, "REL": 0.0, "IMU": 0.0}
    rtcm_bytes = 0
    rtcm_rate = 0.0
    ntrip_rate = 0.0
    # Per-RTCM-type message counts for the GPS1->GPS2 correction stream:
    # shows directly whether 4072.0 (moving-base reference) is present
    rtcm_counts = {}
    rtcm_type_rates = {}
    rate_t0 = time.monotonic()
    last_draw = 0.0
    dirty = False

    def handle_gps(gps: str, payload: bytes) -> None:
        nonlocal dirty
        msg = parse_ubx(payload)
        if msg is None:  # no pyubx2 (or bad frame): show raw class/id
            detail[gps] = (f"UBX cls=0x{payload[2]:02X} id=0x{payload[3]:02X} "
                           f"len={len(payload) - 8}")
            counts[gps] += 1
            dirty = True
            return
        if msg.identity == "NAV-PVT":
            counts[gps] += 1  # one PVT per nav epoch = effective nav rate
            if gps == "GPS1" and msg.fixType >= 2:
                gps1_pos[0] = (msg.lat, msg.lon, msg.hMSL / 1000, msg.numSV)
            detail[gps] = describe_pvt(msg)
            dirty = True
        elif msg.identity == "NAV-RELPOSNED":
            counts["REL"] += 1
            detail["REL"] = describe_relposned(msg)
            dirty = True
        elif msg.identity == "ACK-NAK":  # rejected config is worth seeing
            disp.log(f"{gps}: ACK-NAK for "
                     f"cls=0x{msg.clsID:02X} id=0x{msg.msgID:02X}")
        # other messages (ACK-ACK, MON-RF etc.) are dropped silently

    while True:
        try:
            data = ser.read(4096)
        except KeyboardInterrupt:
            break

        for chan, payload in framer.feed(data):
            if chan == dualgps.CH_GPS1_UBX:
                handle_gps("GPS1", payload)
            elif chan == dualgps.CH_GPS2_UBX:
                handle_gps("GPS2", payload)
            elif chan == dualgps.CH_GPS1_RTCM:
                rtcm_bytes += len(payload)
                mtype = rtcm_type(payload)
                rtcm_counts[mtype] = rtcm_counts.get(mtype, 0) + 1
            elif chan == dualgps.CH_IMU:
                s = dualgps.ImuSample.from_bytes(payload)
                detail["IMU"] = (f"a=({s.ax_g:+.3f},{s.ay_g:+.3f},"
                                 f"{s.az_g:+.3f})g w=({s.gx_dps:+.1f},"
                                 f"{s.gy_dps:+.1f},{s.gz_dps:+.1f})dps "
                                 f"{s.temp_c:.1f}C")
                counts["IMU"] += 1
                dirty = True
            elif chan == dualgps.CH_LOG:
                disp.log(f"LOG: {payload.decode(errors='replace')}")

        if ntrip is not None:
            try:
                while True:
                    disp.log(ntrip.events.get_nowait())
            except queue.Empty:
                pass

        now = time.monotonic()
        if now - rate_t0 >= 1.0:
            span = now - rate_t0
            for key in counts:
                rates[key] = counts[key] / span
                counts[key] = 0
            rtcm_rate = rtcm_bytes / span
            rtcm_bytes = 0
            rtcm_type_rates = {t: c / span for t, c in rtcm_counts.items()}
            rtcm_counts = {}
            if ntrip is not None:
                ntrip_rate = ntrip.take_rx_bytes() / span
            rate_t0 = now
            dirty = True

        if dirty and now - last_draw >= DRAW_INTERVAL:
            types = " ".join(f"{t}:{r:.1f}" for t, r in
                             sorted(rtcm_type_rates.items()))
            ntrip_part = (f"  NTRIP in {ntrip_rate / 1000:.1f}kB/s"
                          if ntrip is not None else "")
            disp.status["GPS1"] = f"{rates['GPS1']:5.1f}Hz  {detail['GPS1']}"
            disp.status["GPS2"] = f"{rates['GPS2']:5.1f}Hz  {detail['GPS2']}"
            disp.status["REL"] = f"{rates['REL']:5.1f}Hz  {detail['REL']}"
            disp.status["RTCM"] = (f"fwd {rtcm_rate / 1000:.1f}kB/s "
                                   f"[{types or 'none'}]Hz{ntrip_part}")
            disp.status["IMU"] = f"{rates['IMU']:5.1f}Hz  {detail['IMU']}"
            disp.draw()
            last_draw = now
            dirty = False

    print()  # leave the status block intact on exit


if __name__ == "__main__":
    sys.exit(main())
