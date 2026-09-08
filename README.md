# Locus1 — Python client

Python tools for **Locus1**, a dual ZED-F9P GNSS receiver by Namo Robotics.
Read both receivers and the IMU over USB, view live status, send receiver
configuration, and forward NTRIP corrections to GPS1.

[Product page](https://namo-robotics.github.io/dual_zedf9p_pcb/) ·
[Purchase and support](mailto:davidwbrwn@gmail.com?subject=Locus1%20inquiry)

## Quick start

Use Python 3.10 or newer. Connect Locus1 by USB, then:

```sh
git clone https://github.com/namo-robotics/locus_client.git
cd locus_client
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python example.py /dev/ttyACM0 --no-ntrip
```

On Windows, activate with `.venv\Scripts\activate` and use your serial port,
for example `COM3`. On macOS, use the device's `/dev/cu.usbmodem...` path.
Your account must have permission to open the serial device.

`--no-ntrip` starts the viewer without connecting to a correction service.
The viewer displays receiver status, relative position/heading messages,
RTCM traffic, IMU samples, and device logs when supplied by the firmware.
Press Ctrl+C to stop.

## Corrections and configuration

```sh
# Use a mountpoint appropriate for your location.
python example.py /dev/ttyACM0 --ntrip YOUR_MOUNTPOINT

# Request a 10 Hz navigation rate in receiver RAM, without NTRIP.
python example.py /dev/ttyACM0 --no-ntrip --set-rate 10

# List serial and NTRIP options.
python example.py --help
```

NTRIP is enabled unless `--no-ntrip` is given. The existing defaults are
`crtk.net:2101`, mountpoint `PDA1`, NTRIP 2.0, and the public Centipede
username/password `centipede`. Choose a mountpoint suitable for your location.
See [Centipede connection information](https://docs.centipede.fr/docs/centipede/3_connect_caster.html).

When NTRIP is enabled and a GPS1 fix is available, the viewer sends its
position to the caster as NMEA GGA every 15 seconds. Corrections are forwarded
to GPS1. RTK status and relative heading depend on receiver configuration,
antennas, reception, and available corrections. This client does not configure
an entire base/rover setup automatically.

## Library

`dualgps.py` is the dependency-free protocol module. The original import name
is preserved for existing integrations; `locus_client` is the repository name.
`example.py` requires `pyserial`; `pyubx2` adds readable UBX decoding and the
position decoding used for GGA reporting. The requirements file installs both.

```python
import dualgps

framer = dualgps.MuxFramer()
# Feed arbitrary chunks read from the serial device.
for channel, payload in framer.feed(received_bytes):
    if channel == dualgps.CH_IMU:
        sample = dualgps.ImuSample.from_bytes(payload)
        print(sample)

# Wrap a UBX configuration message to send to GPS1.
command = dualgps.frame(dualgps.CH_GPS1_CMD, ubx_message)
```

`received_bytes` and `ubx_message` above are application-supplied byte strings.
The module documents the USB multiplexing wire format and channel constants.
This is the Locus1 firmware protocol, not a generic direct-to-u-blox serial client.

## Development

```sh
python -m unittest discover -s tests -v
python example.py --help
```

CI runs protocol tests and checks CLI startup on Python 3.10 and 3.14, without
hardware or a live NTRIP connection. Real receiver operation requires a Locus1 board.

MIT licensed; see [LICENSE](LICENSE). This repository contains the Python client.
Hardware designs and embedded firmware are maintained separately.
