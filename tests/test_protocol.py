import struct
import unittest

import dualgps


class ProtocolTests(unittest.TestCase):
    # Independent wire fixture: channel 0x05, length 2, payload 'OK', checksum A/B.
    packet = bytes.fromhex('fa ce 05 02 00 4f 4b a1 0a')

    def test_encode_wire_fixture(self):
        self.assertEqual(dualgps.frame(dualgps.CH_LOG, b'OK'), self.packet)

    def test_fragmented_frame(self):
        for split in range(1, len(self.packet)):
            with self.subTest(split=split):
                framer = dualgps.MuxFramer()
                self.assertEqual(framer.feed(self.packet[:split]), [])
                self.assertEqual(framer.feed(self.packet[split:]), [(dualgps.CH_LOG, b'OK')])

    def test_noise_and_bad_checksum_recover(self):
        corrupt = self.packet[:-1] + b'\x00'
        self.assertEqual(dualgps.MuxFramer().feed(b'noise' + corrupt + self.packet),
                         [(dualgps.CH_LOG, b'OK')])

    def test_multiple_frames(self):
        self.assertEqual(dualgps.MuxFramer().feed(self.packet * 2),
                         [(dualgps.CH_LOG, b'OK')] * 2)

    def test_imu_units(self):
        sample = dualgps.ImuSample.from_bytes(struct.pack('<Ihhhhhhh', 42, 1000, -1000, 0, 100, -100, 0, 256))
        self.assertEqual(sample.tick_ms, 42)
        self.assertAlmostEqual(sample.ax_g, 0.061)
        self.assertAlmostEqual(sample.ay_g, -0.061)
        self.assertAlmostEqual(sample.gx_dps, 1.75)
        self.assertAlmostEqual(sample.gy_dps, -1.75)
        self.assertEqual(sample.temp_c, 26.0)


if __name__ == '__main__':
    unittest.main()
