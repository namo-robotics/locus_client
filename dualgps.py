"""Python client for the Locus dual ZED-F9P + IMU USB mux protocol.

Wire format (both directions), little-endian:

    0xFA 0xCE | chan(1) | len(2) | payload(len) | ckA ckB

The checksum is an 8-bit Fletcher (same algorithm as UBX) over
chan, len, and payload.

Device -> host channels carry raw UBX frames (parse with pyubx2),
raw RTCM3 frames, a packed IMU sample struct, or ASCII log text.
Host -> device channels carry raw bytes to forward to a GPS UART
(typically complete UBX frames such as CFG-VALSET).

This module has no dependencies; example.py uses pyserial and
optionally pyubx2.
"""

from dataclasses import dataclass
import struct

SYNC1 = 0xFA
SYNC2 = 0xCE

# Device -> host
CH_GPS1_UBX = 0x01   # UBX frame from GPS1 (moving base): PVT, ACKs, MON-RF...
CH_GPS1_RTCM = 0x02  # RTCM3 correction frame from GPS1
CH_GPS2_UBX = 0x03   # UBX frame from GPS2 (rover): PVT, RELPOSNED, ACKs...
CH_IMU = 0x04        # ImuSample struct
CH_LOG = 0x05        # ASCII text

# Host -> device
CH_GPS1_CMD = 0x81   # raw bytes forwarded to GPS1's UART
CH_GPS2_CMD = 0x83   # raw bytes forwarded to GPS2's UART

CHANNEL_NAMES = {
    CH_GPS1_UBX: "GPS1_UBX",
    CH_GPS1_RTCM: "GPS1_RTCM",
    CH_GPS2_UBX: "GPS2_UBX",
    CH_IMU: "IMU",
    CH_LOG: "LOG",
}

_MAX_PAYLOAD = 1030  # RTCM3 max frame; anything longer means lost sync


def _fletcher(chan: int, payload: bytes) -> tuple[int, int]:
    ck_a = ck_b = 0
    for b in bytes([chan, len(payload) & 0xFF, len(payload) >> 8]) + payload:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def frame(chan: int, payload: bytes) -> bytes:
    """Build a complete mux frame ready to write to the serial port."""
    ck_a, ck_b = _fletcher(chan, payload)
    return (bytes([SYNC1, SYNC2, chan, len(payload) & 0xFF, len(payload) >> 8])
            + payload + bytes([ck_a, ck_b]))


class MuxFramer:
    """Incremental deframer. Feed bytes, get (chan, payload) tuples out."""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self._buf.extend(data)
        frames = []
        while True:
            start = self._buf.find(bytes([SYNC1, SYNC2]))
            if start < 0:
                # keep a trailing 0xFA in case its 0xCE is still in flight
                del self._buf[:max(0, len(self._buf) - 1)]
                break
            del self._buf[:start]
            if len(self._buf) < 5:
                break
            chan = self._buf[2]
            length = self._buf[3] | (self._buf[4] << 8)
            if length > _MAX_PAYLOAD:
                del self._buf[:2]  # bogus header; resync past this sync pair
                continue
            if len(self._buf) < 5 + length + 2:
                break
            payload = bytes(self._buf[5:5 + length])
            ck_a, ck_b = self._buf[5 + length], self._buf[6 + length]
            del self._buf[:7 + length]
            if (ck_a, ck_b) == _fletcher(chan, payload):
                frames.append((chan, payload))
            # bad checksum: frame dropped, loop continues at next sync
        return frames


@dataclass
class ImuSample:
    """LSM6DSL sample. Constructed from the 18-byte CH_IMU payload."""

    tick_ms: int
    ax_g: float
    ay_g: float
    az_g: float
    gx_dps: float
    gy_dps: float
    gz_dps: float
    temp_c: float

    _STRUCT = struct.Struct("<Ihhhhhhh")  # tick, ax, ay, az, gx, gy, gz, temp
    ACCEL_G_PER_LSB = 0.061e-3   # +/-2 g range
    GYRO_DPS_PER_LSB = 17.5e-3   # +/-500 dps range

    @classmethod
    def from_bytes(cls, payload: bytes) -> "ImuSample":
        tick, ax, ay, az, gx, gy, gz, temp = cls._STRUCT.unpack(payload)
        return cls(
            tick_ms=tick,
            ax_g=ax * cls.ACCEL_G_PER_LSB,
            ay_g=ay * cls.ACCEL_G_PER_LSB,
            az_g=az * cls.ACCEL_G_PER_LSB,
            gx_dps=gx * cls.GYRO_DPS_PER_LSB,
            gy_dps=gy * cls.GYRO_DPS_PER_LSB,
            gz_dps=gz * cls.GYRO_DPS_PER_LSB,
            temp_c=25.0 + temp / 256.0,
        )
