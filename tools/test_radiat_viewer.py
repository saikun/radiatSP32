"""radiat_viewer のテスト。実行: python -m unittest discover tools"""

import pathlib
import tempfile
import unittest
from unittest import mock

import radiat_viewer
from radiat_viewer import (SUM_BIN_WIDTH, Config, Hit, HitMap, MaskList, Measurement, Stat,
                           average_cpm, energy_color, find_repeats, format_elapsed, parse_line,
                           split_lines)


class ParseLineTest(unittest.TestCase):
    def test_hit(self):
        self.assertEqual(parse_line("[HIT] X=123, Y=45, Energy=67, Sum=210, T=123456"),
                         Hit(123, 45, 67, 210, 123456))

    def test_stat(self):
        self.assertEqual(
            parse_line("[STAT] CPS=3, CPM=180, Total=1234, FPS=25.0, Drop=2, Cal=1"),
            Stat(3, 180, 1234, 25.0, 2, True))

    def test_stat_with_masked(self):
        self.assertEqual(
            parse_line("[STAT] CPS=3, CPM=180, Total=1234, FPS=25.0, Drop=0, Cal=0, Masked=7"),
            Stat(3, 180, 1234, 25.0, 0, False, 7))

    def test_mask_list(self):
        self.assertEqual(parse_line("[MASK] n=2 399,164 10,20"),
                         MaskList([(399, 164), (10, 20)]))
        self.assertEqual(parse_line("[MASK] n=0"), MaskList([]))

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


class FindRepeatsTest(unittest.TestCase):
    def test_noise_pixel_wobbling_between_neighbors(self):
        hits = [Hit(399, 164, 33, 0, 0), Hit(399, 165, 31, 0, 0), Hit(399, 164, 27, 0, 0),
                Hit(398, 164, 37, 0, 0), Hit(100, 100, 80, 0, 0), Hit(200, 50, 60, 0, 0),
                Hit(201, 51, 45, 0, 0)]
        self.assertEqual(find_repeats(hits), [(399, 164, 4, [27, 31, 33, 37]),
                                              (200, 50, 2, [45, 60])])

    def test_no_repeats(self):
        self.assertEqual(find_repeats([Hit(1, 1, 30, 0, 0), Hit(5, 5, 30, 0, 0)]), [])

    def test_render_with_masks(self):
        hitmap = HitMap(400, 296)
        image = hitmap.render("points", [(399, 164), (0, 0)])
        self.assertEqual(image.size, hitmap.view_size)
        self.assertNotEqual(image.getbbox(), None)  # 枠が描かれている


class MeasurementTest(unittest.TestCase):
    def test_format_elapsed(self):
        self.assertEqual(format_elapsed(0), "00:00")
        self.assertEqual(format_elapsed(299.9), "04:59")
        self.assertEqual(format_elapsed(3725), "1:02:05")

    def test_average_cpm(self):
        cpm, error = average_cpm(277, 300)
        self.assertAlmostEqual(cpm, 55.4)
        self.assertAlmostEqual(error, 277 ** 0.5 / 5)
        self.assertEqual(average_cpm(10, 0), (0.0, 0.0))

    def test_csv_is_split_per_measurement(self):
        with tempfile.TemporaryDirectory() as directory, \
                mock.patch.object(radiat_viewer, "LOG_DIR", pathlib.Path(directory)):
            first = Measurement(record=True)
            first.record(Hit(1, 2, 30, 40, 1000))
            first_path = first.stop_recording()
            second = Measurement(record=True)  # 同じ秒に作っても上書きしない
            second_path = second.stop_recording()
            self.assertNotEqual(first_path, second_path)
            lines = first_path.read_text().splitlines()
            self.assertEqual(lines[0], "pc_time,esp_ms,x,y,energy,sum")
            self.assertTrue(lines[1].endswith(",1000,1,2,30,40"))
            self.assertEqual(len(second_path.read_text().splitlines()), 1)

    def test_recording_can_be_toggled(self):
        with tempfile.TemporaryDirectory() as directory, \
                mock.patch.object(radiat_viewer, "LOG_DIR", pathlib.Path(directory)):
            measurement = Measurement(record=False)
            measurement.record(Hit(1, 2, 30, 40, 0))  # CSV off なら何もしない
            self.assertIsNone(measurement.set_recording(True))
            self.assertIsNotNone(measurement.recorder)
            path = measurement.set_recording(False)
            self.assertTrue(path.exists())
            self.assertIsNone(measurement.recorder)


if __name__ == "__main__":
    unittest.main()
