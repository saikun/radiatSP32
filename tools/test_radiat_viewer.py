"""radiat_viewer のテスト。実行: python -m unittest discover tools"""

import unittest

from radiat_viewer import (SUM_BIN_WIDTH, Config, Hit, HitMap, Stat, energy_color, parse_line,
                           split_lines)


class ParseLineTest(unittest.TestCase):
    def test_hit(self):
        self.assertEqual(parse_line("[HIT] X=123, Y=45, Energy=67, Sum=210, T=123456"),
                         Hit(123, 45, 67, 210, 123456))

    def test_stat(self):
        self.assertEqual(
            parse_line("[STAT] CPS=3, CPM=180, Total=1234, FPS=25.0, Drop=2, Cal=1"),
            Stat(3, 180, 1234, 25.0, 2, True))

    def test_config(self):
        self.assertEqual(parse_line("[CFG] W=400, H=296, TH=25, GAIN=10, EXP=300"),
                         Config(400, 296, 25, 10, 300))

    def test_other_lines_stay_text(self):
        for text in ("[CAL] done mean=12.00 max=40 hot=3", "[HIT] X=1", "E (12) cam_hal: x"):
            self.assertEqual(parse_line(text), text)


class SplitLinesTest(unittest.TestCase):
    def test_partial_lines_are_kept(self):
        buffer = bytearray()
        self.assertEqual(split_lines(buffer, b"[OK] a\r\n[OK"), ["[OK] a"])
        self.assertEqual(split_lines(buffer, b"] b\n"), ["[OK] b"])

    def test_garbage_is_removed(self):
        self.assertEqual(split_lines(bytearray(), b"\xff\x00\n\xfe[OK] a\n"), ["[OK] a"])


class EnergyColorTest(unittest.TestCase):
    def test_stops(self):
        self.assertEqual(energy_color(0), (0, 0, 255))
        self.assertEqual(energy_color(255), (255, 255, 255))
        self.assertEqual(energy_color(300), (255, 255, 255))


class HitMapTest(unittest.TestCase):
    def test_accumulates_and_clears(self):
        hitmap = HitMap(400, 296)
        hitmap.add(Hit(10, 20, 60, 5000, 0))
        hitmap.add(Hit(10, 20, 255, 300, 0))
        hitmap.add(Hit(400, 0, 50, 50, 0))  # 範囲外は無視
        self.assertEqual(hitmap.hits, 2)
        self.assertEqual(hitmap.counts[20 * 400 + 10], 2)
        self.assertEqual(hitmap.energy_hist[60], 1)
        self.assertEqual(hitmap.energy_hist[255], 1)
        self.assertEqual(hitmap.sum_hist[-1], 1)  # 上限超えは最後のビン
        self.assertEqual(hitmap.sum_hist[300 // SUM_BIN_WIDTH], 1)
        for mode in ("points", "heatmap"):
            self.assertEqual(hitmap.render(mode).size, hitmap.view_size)
        hitmap.clear()
        self.assertEqual(hitmap.hits, 0)
        self.assertEqual(max(hitmap.counts), 0)


if __name__ == "__main__":
    unittest.main()
