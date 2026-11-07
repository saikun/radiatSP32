"""StreamParser のテスト。実行: python -m unittest discover tools"""

import unittest
import zlib

from camera_viewer import FORMAT_GRAY, HEADER, MAGIC, CorruptFrame, Frame, StreamParser


def make_frame(payload=b"\x01\x02\x03\x04", seq=7, width=2, height=2):
    head = HEADER.pack(MAGIC, FORMAT_GRAY, 0, width, height, seq, len(payload), zlib.crc32(payload), 0)
    return head[:-4] + zlib.crc32(head[:-4]).to_bytes(4, "little") + payload


class StreamParserTest(unittest.TestCase):
    def test_text_and_frames_are_separated(self):
        events = StreamParser().feed(b"[OK] a\r\n" + make_frame() + b"[OK] b\n")
        self.assertEqual(events[0], "[OK] a")
        self.assertEqual(events[1], Frame(FORMAT_GRAY, 2, 2, 7, b"\x01\x02\x03\x04"))
        self.assertEqual(events[2], "[OK] b")

    def test_split_at_every_byte(self):
        data = b"[OK] a\n" + make_frame() + make_frame(seq=8) + b"[OK] b\n"
        parser = StreamParser()
        events = []
        for i in range(len(data)):
            events += parser.feed(data[i : i + 1])
        self.assertEqual([getattr(e, "seq", e) for e in events], ["[OK] a", 7, 8, "[OK] b"])

    def test_log_inserted_into_payload_drops_only_that_frame(self):
        frame = make_frame(payload=bytes(range(100)), seq=1, width=10, height=10)
        broken = frame[:60] + b"E (123) cam_hal: FB-OVF\n" + frame[60:]
        events = StreamParser().feed(broken + make_frame(seq=2))
        self.assertIsInstance(events[0], CorruptFrame)
        self.assertEqual([e.seq for e in events if isinstance(e, Frame)], [2])

    def test_corrupt_header_does_not_wait_for_bogus_length(self):
        frame = bytearray(make_frame(seq=1))
        frame[15] ^= 0xFF  # payload_len を壊す
        events = StreamParser().feed(bytes(frame) + make_frame(seq=2))
        self.assertIsInstance(events[0], CorruptFrame)
        self.assertEqual([e.seq for e in events if isinstance(e, Frame)], [2])

    def test_truncated_frame_recovers_text_after_reset(self):
        parser = StreamParser()
        frame = make_frame(payload=bytes(100), width=10, height=10)
        self.assertEqual(parser.feed(frame[:50] + b"\n[BOOT] camera test firmware\n"), [])
        events = parser.drop_partial_frame()
        self.assertIsInstance(events[0], CorruptFrame)
        self.assertIn("[BOOT] camera test firmware", events)

    def test_log_after_corrupt_frame_is_kept(self):
        frame = make_frame(payload=b"\xff" * 100, seq=1, width=10, height=10)
        broken = frame[:60] + b"E (123) cam_hal: FB-OVF\n" + frame[60:]
        events = StreamParser().feed(broken + b"[OK] fmt=gray\n" + make_frame(seq=2))
        self.assertIn("E (123) cam_hal: FB-OVF", events)
        self.assertIn("[OK] fmt=gray", events)
        self.assertEqual([e.seq for e in events if isinstance(e, Frame)], [2])

    def test_binary_garbage_is_not_shown_as_text(self):
        self.assertEqual(StreamParser().feed(b"\xff\x00\xfe\x80\n[OK] a\n"), ["[OK] a"])


if __name__ == "__main__":
    unittest.main()
