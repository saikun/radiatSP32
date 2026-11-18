"""放射線検出モード (pio run -e radiation) の結果を表示する GUI。

表示: ヒットマップ / エネルギーヒストグラム / CPS の推移 / ESP32 のログ
操作: 閾値・ゲイン・露光の変更、再キャリブレーション、リセット、CSV 記録

使い方: python tools/radiat_viewer.py
必要なパッケージ: pip install -r tools/requirements.txt
通信の仕様は src/radiation/main.cpp の先頭コメントを参照。
"""

import collections
import csv
import dataclasses
import io
import math
import pathlib
import queue
import re
import threading
import time

import PySimpleGUI as sg
import serial
from PIL import Image, ImageDraw, ImageFilter

from camera_viewer import list_ports, open_port

DEFAULT_BAUD = 921600
# 標準の計測設定 (src/radiation/main.cpp の起動時の値と合わせる)
DEFAULT_THRESHOLD = 25
DEFAULT_GAIN = 30
DEFAULT_EXPOSURE = 1200
DEFAULT_SENSOR_SIZE = (400, 296)  # src/radiation/main.cpp の kFrameSize (CIF)
MAP_VIEW_MAX = (640, 480)
CHART_SIZE = (380, 200)
HISTOGRAM_BINS = 256
SUM_BIN_WIDTH = 4  # Sum は最大 255*9 になるので 4 刻みで 0-1023 を表示 (超過分は最後のビン)
RATE_HISTORY_S = 120
MAP_REFRESH_S = 0.1
CHART_REFRESH_S = 0.5
MAX_LINE = 512
TOOLS_DIR = pathlib.Path(__file__).resolve().parent
LOG_DIR = TOOLS_DIR / "logs"
SNAPSHOT_DIR = TOOLS_DIR / "snapshots"

# src/radiation/main.cpp の Serial.printf と一致させること
HIT_RE = re.compile(r"\[HIT\] X=(\d+), Y=(\d+), Energy=(\d+), Sum=(\d+), T=(\d+)")
STAT_RE = re.compile(
    r"\[STAT\] CPS=(\d+), CPM=(\d+), Total=(\d+), FPS=([\d.]+), Drop=(\d+), Cal=([01])"
    r"(?:, Masked=(\d+))?"  # マスク機能より前のファームには無い
)
CFG_RE = re.compile(r"\[CFG\] W=(\d+), H=(\d+), TH=(\d+), GAIN=(\d+), EXP=(\d+)")
MASK_RE = re.compile(r"\[MASK\] n=\d+((?: \d+,\d+)*)$")
POINT_RE = re.compile(r"(\d+),(\d+)")
MASK_INPUT_RE = re.compile(r"^\s*(\d+)\s*,\s*(\d+)\s*$")
MASK_COLOR = (255, 64, 255)
NON_PRINTABLE = re.compile(r"[^\x20-\x7e]")


@dataclasses.dataclass
class Hit:
    x: int
    y: int
    energy: int
    sum: int
    t_ms: int


@dataclasses.dataclass
class Stat:
    cps: int
    cpm: int
    total: int
    fps: float
    drop: int
    calibrating: bool
    masked: int = 0  # ノイズ画素マスクに当たって捨てたイベント数


@dataclasses.dataclass
class Config:
    width: int
    height: int
    threshold: int
    gain: int
    exposure: int


@dataclasses.dataclass
class MaskList:
    points: list  # [(x, y), ...] 各点の周囲 3x3 が検出から外れる


def parse_line(text):
    """1 行を Hit / Stat / Config / MaskList に変換する。それ以外のログは文字列のまま返す。"""
    if match := HIT_RE.match(text):
        return Hit(*map(int, match.groups()))
    if match := STAT_RE.match(text):
        cps, cpm, total, fps, drop, cal, masked = match.groups()
        return Stat(int(cps), int(cpm), int(total), float(fps), int(drop), cal == "1",
                    int(masked or 0))
    if match := CFG_RE.match(text):
        return Config(*map(int, match.groups()))
    if match := MASK_RE.match(text):
        return MaskList([(int(x), int(y)) for x, y in POINT_RE.findall(match.group(1))])
    return text


def find_repeats(hits, min_count=2):
    """同じ画素 (3x3 以内) で繰り返し出たヒットを、ノイズ画素の候補として返す。

    戻り値は [(x, y, 回数, [エネルギー...]), ...] で回数の多い順。x, y はそのうち最も多く
    出た画素。偶然の重なりもあり得るので、背景測定で何度も出るものだけをマスクすること。
    """
    by_pixel = collections.defaultdict(list)
    for hit in hits:
        by_pixel[(hit.x, hit.y)].append(hit.energy)
    groups = []
    for (x, y) in by_pixel:
        energies = [e for (nx, ny), es in by_pixel.items()
                    if abs(nx - x) <= 1 and abs(ny - y) <= 1 for e in es]
        if len(energies) >= min_count:
            groups.append((len(energies), len(by_pixel[(x, y)]), x, y, energies))
    groups.sort(key=lambda g: (-g[0], -g[1]))
    result = []
    for count, _, x, y, energies in groups:
        if all(abs(x - rx) > 1 or abs(y - ry) > 1 for rx, ry, _, _ in result):
            result.append((x, y, count, sorted(energies)))
    return result


def split_lines(buffer, data):
    """buffer に data を足し、完成した行 (表示可能文字のみ) を返す。残りは buffer に残る。"""
    buffer.extend(data)
    *lines, rest = buffer.split(b"\n")
    buffer[:] = rest[-MAX_LINE:]
    result = []
    for raw in lines:
        # ボーレートの違う ROM ブートログなどの化けた文字は捨てる
        text = NON_PRINTABLE.sub("", raw.decode("ascii", errors="ignore")).strip()
        if text:
            result.append(text)
    return result


_COLOR_STOPS = [(0, 0, 255), (0, 255, 0), (255, 255, 0), (255, 0, 0), (255, 255, 255)]


def energy_color(energy):
    """0-255 を 青→緑→黄→赤→白 に対応させる (ファームウェアの energyColor と同じ)。"""
    scaled = max(0, min(255, energy)) * 4
    segment = 3 if scaled >= 1020 else scaled // 255
    t = scaled - segment * 255
    start, end = _COLOR_STOPS[segment], _COLOR_STOPS[segment + 1]
    return tuple(a + (b - a) * t // 255 for a, b in zip(start, end))


def hex_color(rgb):
    return "#{:02x}{:02x}{:02x}".format(*rgb)


# ヒートマップ用: 黒 → 青 → 緑 → 黄 → 赤 → 白
HEATMAP_PALETTE = [0, 0, 0] + [c for i in range(1, 256) for c in energy_color(i)]


class HitMap:
    """受信したヒットを蓄積し、ヒットマップ画像とヒストグラムを作る。"""

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.scale = min(MAP_VIEW_MAX[0] / width, MAP_VIEW_MAX[1] / height)
        self.view_size = (round(width * self.scale), round(height * self.scale))
        self.clear()

    def clear(self):
        self.counts = bytearray(self.width * self.height)  # 画素ごとのヒット数 (255 で飽和)
        self.points = Image.new("RGB", self.view_size)
        self._draw = ImageDraw.Draw(self.points)
        self.energy_hist = [0] * HISTOGRAM_BINS
        self.sum_hist = [0] * HISTOGRAM_BINS
        self.hits = 0
        self.history = []  # find_repeats 用

    def add(self, hit):
        if not (0 <= hit.x < self.width and 0 <= hit.y < self.height):
            return
        self.history.append(hit)
        index = hit.y * self.width + hit.x
        self.counts[index] = min(255, self.counts[index] + 1)
        self.energy_hist[min(hit.energy, HISTOGRAM_BINS - 1)] += 1
        self.sum_hist[min(hit.sum // SUM_BIN_WIDTH, HISTOGRAM_BINS - 1)] += 1
        self.hits += 1
        cx, cy = (hit.x + 0.5) * self.scale, (hit.y + 0.5) * self.scale
        radius = 2
        self._draw.ellipse((cx - radius, cy - radius, cx + radius, cy + radius),
                           fill=energy_color(hit.energy))

    def heatmap(self):
        """画素ごとのヒット数を対数スケールで色付けした画像。"""
        peak = max(self.counts) if self.hits else 0
        image = Image.frombytes("L", (self.width, self.height), bytes(self.counts))
        if peak > 0:
            lut = [0] + [max(1, round(255 * math.log1p(c) / math.log1p(peak))) for c in range(1, 256)]
            image = image.point(lut)
        # 1 画素の点でも見えるよう少し太らせてから拡大する
        image = image.filter(ImageFilter.MaxFilter(3)).resize(self.view_size, Image.NEAREST)
        image = image.convert("P")
        image.putpalette(HEATMAP_PALETTE)
        return image.convert("RGB")

    def render(self, mode, masks=()):
        """masks の各点の周囲 3x3 (検出から外れる範囲) を枠で示す。"""
        image = self.heatmap() if mode == "heatmap" else self.points.copy()
        draw = ImageDraw.Draw(image)
        for x, y in masks:
            margin = 3  # 1 画素の大きさによらず見えるように枠を少し広げる
            draw.rectangle(((x - 1) * self.scale - margin, (y - 1) * self.scale - margin,
                            (x + 2) * self.scale + margin, (y + 2) * self.scale + margin),
                           outline=MASK_COLOR, width=2)
        return image


def to_png(image):
    buffer = io.BytesIO()
    image.save(buffer, format="PNG", compress_level=1)
    return buffer.getvalue()


def draw_histogram(graph, counts, log_scale, x_label, bin_width):
    width, height = CHART_SIZE
    graph.erase()
    top = height - 22
    peak = max(counts)
    graph.draw_text(f"{x_label} histogram  (n={sum(counts)}, peak={peak})",
                    (width / 2, height - 10), color="white", font=("Consolas", 9))
    if peak == 0:
        graph.draw_text("no data", (width / 2, height / 2), color="gray")
        return
    bar = width / len(counts)
    for i, count in enumerate(counts):
        if count == 0:
            continue
        ratio = math.log1p(count) / math.log1p(peak) if log_scale else count / peak
        color = hex_color(energy_color(i * 255 // (len(counts) - 1)))
        graph.draw_rectangle((i * bar, max(1.0, ratio * top)), ((i + 1) * bar, 0),
                             fill_color=color, line_color=color, line_width=0)
    graph.draw_text("0", (8, 8), color="gray", font=("Consolas", 8))
    graph.draw_text(str(len(counts) * bin_width - 1), (width - 16, 8), color="gray",
                    font=("Consolas", 8))


def draw_rate(graph, history):
    """直近 RATE_HISTORY_S 秒の CPS (水色) と CPM/60 (橙) を描く。"""
    width, height = CHART_SIZE
    graph.erase()
    top = height - 22
    peak = max([max(s.cps, s.cpm / 60) for s in history], default=0)
    last = history[-1] if history else None
    title = f"CPS  (last {RATE_HISTORY_S}s, max {peak:.0f})"
    if last is not None:
        title += f"  CPM {last.cpm}"
    graph.draw_text(title, (width / 2, height - 10), color="white", font=("Consolas", 9))
    if peak == 0:
        return
    step = width / (RATE_HISTORY_S - 1)
    offset = RATE_HISTORY_S - len(history)
    for i, stat in enumerate(history):
        if stat.calibrating:  # キャリブレーション中の区間を暗く塗る
            x = (offset + i) * step
            graph.draw_rectangle((x - step / 2, top), (x + step / 2, 0), fill_color="#333300",
                                 line_width=0)
    for values, color in (([s.cps for s in history], "#4fc3f7"),
                          ([s.cpm / 60 for s in history], "#ffa726")):
        points = [((offset + i) * step, v / peak * top) for i, v in enumerate(values)]
        if len(points) >= 2:
            graph.draw_lines(points, color=color, width=2)


class Connection:
    def __init__(self, port, baud):
        self.events = queue.Queue()
        self._stop = threading.Event()
        self._device = open_port(port, baud)
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()

    def _read(self):
        buffer = bytearray()
        while not self._stop.is_set():
            try:
                data = self._device.read(self._device.in_waiting or 1)
            except (serial.SerialException, OSError) as error:
                self.events.put(("disconnected", str(error)))
                return
            for text in split_lines(buffer, data):
                self.events.put(("line", (text, parse_line(text))))

    def send(self, command):
        try:
            self._device.write(command.encode("ascii") + b"\n")
        except (serial.SerialException, OSError) as error:
            self.events.put(("disconnected", str(error)))

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1)
        self._device.close()


class CsvRecorder:
    def __init__(self):
        LOG_DIR.mkdir(exist_ok=True)
        stem = f"hits_{time.strftime('%Y%m%d_%H%M%S')}"
        self.path = LOG_DIR / f"{stem}.csv"
        suffix = 2
        while self.path.exists():  # 1 秒以内に Reset を続けて押した場合
            self.path = LOG_DIR / f"{stem}_{suffix}.csv"
            suffix += 1
        # 行バッファリングにして、ビューアが落ちても記録済みの行は残るようにする
        self._file = self.path.open("w", newline="", encoding="ascii", buffering=1)
        self._writer = csv.writer(self._file)
        self._writer.writerow(["pc_time", "esp_ms", "x", "y", "energy", "sum"])

    def write(self, hit):
        self._writer.writerow([f"{time.time():.3f}", hit.t_ms, hit.x, hit.y, hit.energy, hit.sum])

    def close(self):
        self._file.close()


def format_elapsed(seconds):
    minutes, sec = divmod(int(seconds), 60)
    hours, minutes = divmod(minutes, 60)
    return f"{hours}:{minutes:02d}:{sec:02d}" if hours else f"{minutes:02d}:{sec:02d}"


def average_cpm(total, seconds):
    """計測全体の平均 CPM と、その誤差 (√カウント から求めたゆらぎ)。"""
    if seconds <= 0:
        return 0.0, 0.0
    scale = 60 / seconds
    return total * scale, math.sqrt(total) * scale


class Measurement:
    """Reset (ESP32 の "[OK] reset") から始まる 1 回分の計測。経過時間と CSV を管理する。"""

    def __init__(self, record):
        self.start = time.monotonic()
        self.recorder = CsvRecorder() if record else None

    def elapsed(self):
        return time.monotonic() - self.start

    def record(self, hit):
        if self.recorder is not None:
            self.recorder.write(hit)

    def set_recording(self, enabled):
        """計測の途中で Record CSV を切り替えたとき。閉じたファイルがあればそのパスを返す。"""
        if enabled and self.recorder is None:
            self.recorder = CsvRecorder()
            return None
        if not enabled:
            return self.stop_recording()
        return None

    def stop_recording(self):
        if self.recorder is None:
            return None
        self.recorder.close()
        path, self.recorder = self.recorder.path, None
        return path


def build_window(view_size):
    ports = list_ports()
    spin = dict(size=(5, 1), enable_events=False)
    layout = [
        [
            sg.Text("Port"),
            sg.Combo(ports, default_value=ports[-1] if ports else "", key="-PORT-", size=(10, 1)),
            sg.Button("Refresh", key="-REFRESH-"),
            sg.Text("Baud"),
            sg.Combo([115200, 921600, 2000000], default_value=DEFAULT_BAUD, key="-BAUD-",
                     size=(9, 1)),
            sg.Button("Connect", key="-CONNECT-"),
        ],
        [
            sg.Text("Threshold"),
            sg.Spin(list(range(1, 256)), initial_value=DEFAULT_THRESHOLD, key="-TH-", **spin),
            sg.Text("Gain"),
            sg.Spin(list(range(0, 31)), initial_value=DEFAULT_GAIN, key="-GAIN-", **spin),
            sg.Text("Exposure"),
            sg.Spin(list(range(0, 1201, 10)), initial_value=DEFAULT_EXPOSURE, key="-EXP-", **spin),
            sg.Button("Apply", key="-APPLY-", disabled=True,
                      tooltip="ゲイン/露光を変えると ESP32 側で自動的に再キャリブレーションします"),
            sg.Button("Calibrate", key="-CAL-", disabled=True,
                      tooltip="センサーを遮光した状態で実行 (ダークフレームの取り直し)"),
            sg.Button("Reset", key="-RESET-",
                      tooltip="計測を最初から始める (カウンタ・Drop・表示を消去し、"
                              "経過時間と CSV ファイルを新しくする)"),
            sg.Button("Info", key="-INFO-", disabled=True),
        ],
        [
            sg.Text("Map"),
            sg.Combo(["points", "heatmap"], default_value="points", key="-MODE-",
                     enable_events=True, readonly=True, size=(8, 1)),
            sg.Text("Histogram"),
            sg.Combo(["energy", "sum"], default_value="energy", key="-HISTKIND-",
                     enable_events=True, readonly=True, size=(7, 1)),
            sg.Checkbox("log scale", default=True, key="-LOG_SCALE-", enable_events=True),
            sg.Checkbox("Record CSV", default=True, key="-REC-", enable_events=True,
                        tooltip="Reset ごとに tools/logs/ へ新しい CSV を作る"),
            sg.Button("Save image", key="-SAVE-"),
            sg.Checkbox("Show HIT/STAT lines", key="-VERBOSE-"),
        ],
        [sg.Text("", key="-MEASURE-", size=(90, 1), font=("Consolas", 13, "bold"),
                 text_color="#ffee58")],
        [
            sg.Column([[sg.Image(key="-MAP-", size=view_size, background_color="black")]],
                      vertical_alignment="top", pad=(0, 0)),
            sg.Column([
                [sg.Graph(CHART_SIZE, (0, 0), CHART_SIZE, key="-HIST-", background_color="black")],
                [sg.Graph(CHART_SIZE, (0, 0), CHART_SIZE, key="-RATE-", background_color="black")],
                [
                    sg.Text("Noise mask x,y"),
                    sg.Input("", key="-MASKXY-", size=(9, 1)),
                    sg.Button("Add", key="-MASKADD-", disabled=True,
                              tooltip="その画素の周囲 3x3 を検出から外す (ESP32 に保存される)"),
                    sg.Button("Remove", key="-MASKDEL-", disabled=True),
                ],
                [
                    sg.Button("Find repeats", key="-REPEATS-",
                              tooltip="この計測で同じ画素に 2 回以上出たヒットをログに出す"),
                    sg.Button("Clear masks", key="-MASKCLR-", disabled=True),
                ],
                [sg.Text("Masks: -", key="-MASKS-", size=(48, 2), font=("Consolas", 9))],
            ], vertical_alignment="top"),
        ],
        [sg.Text("Not connected", key="-STATUS-", size=(110, 1), font=("Consolas", 10))],
        [sg.Multiline(key="-LOG-", size=(130, 8), disabled=True, autoscroll=True,
                      font=("Consolas", 9))],
    ]
    return sg.Window("radiatSP32 radiation viewer", layout, finalize=True)


def set_connected(window, connected):
    window["-CONNECT-"].update("Disconnect" if connected else "Connect")
    for key in ("-APPLY-", "-CAL-", "-INFO-", "-MASKADD-", "-MASKDEL-", "-MASKCLR-"):
        window[key].update(disabled=not connected)


def main():
    hitmap = HitMap(*DEFAULT_SENSOR_SIZE)
    window = build_window(hitmap.view_size)
    connection = None
    measurement = None  # Reset するまでは None (経過時間・CSV なし)
    config = None
    last_stat = None
    masks = []  # ESP32 の [MASK] 行で更新される
    rate_history = collections.deque(maxlen=RATE_HISTORY_S)
    map_dirty = chart_dirty = True
    next_map = next_chart = 0.0
    measure_text = None

    def log(text):
        window["-LOG-"].print(text)

    def clear_view():
        nonlocal last_stat, map_dirty, chart_dirty
        hitmap.clear()
        rate_history.clear()
        last_stat = None
        map_dirty = chart_dirty = True

    def log_saved(path):
        if path is None:
            return
        total = last_stat.total if last_stat is not None else "?"
        drop = last_stat.drop if last_stat is not None else "?"
        masked = last_stat.masked if last_stat is not None else "?"
        log(f"[PC] saved {path.name} ({format_elapsed(measurement.elapsed())}, "
            f"Total {total}, PC hits {hitmap.hits}, Drop {drop}, Masked {masked})")

    def finish_measurement():
        nonlocal measurement
        if measurement is not None:
            log_saved(measurement.stop_recording())
            measurement = None

    def start_measurement(record):
        nonlocal measurement
        finish_measurement()
        clear_view()
        measurement = Measurement(record)
        if measurement.recorder is not None:
            log(f"[PC] measurement started, recording to {measurement.recorder.path.name}")
        else:
            log("[PC] measurement started (CSV off)")

    def disconnect():
        nonlocal connection
        finish_measurement()
        connection.close()
        connection = None
        set_connected(window, False)

    while True:
        event, values = window.read(timeout=30)
        if event == sg.WIN_CLOSED:
            break

        if event == "-REFRESH-":
            ports = list_ports()
            window["-PORT-"].update(values=ports, value=ports[-1] if ports else "")
        elif event == "-CONNECT-":
            if connection is not None:
                disconnect()
                window["-STATUS-"].update("Not connected")
            else:
                try:
                    connection = Connection(values["-PORT-"], int(values["-BAUD-"]))
                except (serial.SerialException, ValueError) as error:
                    log(f"[PC] open failed: {error}")
                else:
                    set_connected(window, True)
                    # ESP32 はリセットされないので、現在の設定を問い合わせる
                    connection.send("info")
        elif event == "-APPLY-" and connection is not None:
            # 変わった値だけ送る (ゲイン/露光を送ると再キャリブレーションが走るため)
            for key, command, current in (("-TH-", "t", config and config.threshold),
                                          ("-GAIN-", "g", config and config.gain),
                                          ("-EXP-", "e", config and config.exposure)):
                try:
                    value = int(values[key])
                except ValueError:
                    log(f"[PC] invalid value: {values[key]}")
                    continue
                if value != current:
                    connection.send(f"{command}={value}")
        elif event == "-CAL-" and connection is not None:
            connection.send("c")
        elif event == "-INFO-" and connection is not None:
            connection.send("info")
        elif event == "-RESET-":
            if connection is not None:
                # 計測は ESP32 から "[OK] reset" が返った時点で始める。
                # それより後に届いた行は、すべてリセット後のデータになる
                connection.send("r")
            else:
                clear_view()
        elif event in ("-MODE-", "-HISTKIND-", "-LOG_SCALE-"):
            map_dirty = chart_dirty = True
        elif event == "-REC-" and measurement is not None:
            log_saved(measurement.set_recording(values["-REC-"]))
            if measurement.recorder is not None:
                log(f"[PC] recording to {measurement.recorder.path.name}")
        elif event == "-SAVE-":
            SNAPSHOT_DIR.mkdir(exist_ok=True)
            path = SNAPSHOT_DIR / f"hitmap_{time.strftime('%Y%m%d_%H%M%S')}_{values['-MODE-']}.png"
            hitmap.render(values["-MODE-"], masks).save(path)
            log(f"[PC] saved {path}")
        elif event in ("-MASKADD-", "-MASKDEL-") and connection is not None:
            match = MASK_INPUT_RE.match(values["-MASKXY-"])
            if match is None:
                log(f"[PC] mask point must be 'x,y': {values['-MASKXY-']}")
            else:
                prefix = "m=" if event == "-MASKADD-" else "m-="
                connection.send(f"{prefix}{match.group(1)},{match.group(2)}")
        elif event == "-MASKCLR-" and connection is not None:
            if sg.popup_yes_no("ESP32 に保存されているマスクを全て削除しますか？",
                               title="Clear masks") == "Yes":
                connection.send("m=clear")
        elif event == "-REPEATS-":
            repeats = find_repeats(hitmap.history)
            if not repeats:
                log(f"[PC] no repeated pixels in {hitmap.hits} hits")
            for x, y, count, energies in repeats:
                log(f"[PC] repeat {x},{y} x{count} energy={energies}")
            if repeats:
                # いちばん多いものを入力欄に入れておく (Add で登録できる)
                window["-MASKXY-"].update(f"{repeats[0][0]},{repeats[0][1]}")

        if connection is not None:
            try:
                while True:
                    kind, value = connection.events.get_nowait()
                    if kind == "disconnected":
                        log(f"[PC] disconnected: {value}")
                        disconnect()
                        break
                    text, message = value
                    if isinstance(message, Hit):
                        hitmap.add(message)
                        if measurement is not None:
                            measurement.record(message)
                        map_dirty = chart_dirty = True
                        if values["-VERBOSE-"]:
                            log(text)
                    elif isinstance(message, Stat):
                        last_stat = message
                        rate_history.append(message)
                        chart_dirty = True
                        if values["-VERBOSE-"]:
                            log(text)
                    elif isinstance(message, Config):
                        log(text)
                        if config is None or (config.width, config.height) != (message.width,
                                                                               message.height):
                            hitmap = HitMap(message.width, message.height)
                            window["-MAP-"].update(size=hitmap.view_size)
                            map_dirty = chart_dirty = True
                        config = message
                        window["-TH-"].update(value=message.threshold)
                        window["-GAIN-"].update(value=message.gain)
                        window["-EXP-"].update(value=message.exposure)
                    elif isinstance(message, MaskList):
                        log(text)
                        masks = message.points
                        window["-MASKS-"].update(
                            "Masks: " + (" ".join(f"{x},{y}" for x, y in masks) or "none"))
                        map_dirty = True
                    elif text == "[OK] reset":
                        log(text)
                        start_measurement(values["-REC-"])
                    else:
                        log(text)
            except queue.Empty:
                pass

        now = time.monotonic()
        if map_dirty and now >= next_map:
            window["-MAP-"].update(data=to_png(hitmap.render(values["-MODE-"], masks)))
            map_dirty = False
            next_map = now + MAP_REFRESH_S
        if chart_dirty and now >= next_chart:
            if values["-HISTKIND-"] == "sum":
                draw_histogram(window["-HIST-"], hitmap.sum_hist, values["-LOG_SCALE-"], "Sum",
                               SUM_BIN_WIDTH)
            else:
                draw_histogram(window["-HIST-"], hitmap.energy_hist, values["-LOG_SCALE-"],
                               "Energy", 1)
            draw_rate(window["-RATE-"], rate_history)
            chart_dirty = False
            next_chart = now + CHART_REFRESH_S

            status = f"PC hits {hitmap.hits}"
            if last_stat is not None:
                s = last_stat
                status = (f"CPS {s.cps:>4} | CPM {s.cpm:>6} | Total {s.total:>7} | "
                          f"FPS {s.fps:4.1f} | Drop {s.drop} | Masked {s.masked} | " + status)
                if s.calibrating:
                    status += " | CALIBRATING"
            if config is not None:
                status += (f" | {config.width}x{config.height} TH={config.threshold} "
                           f"GAIN={config.gain} EXP={config.exposure}")
            window["-STATUS-"].update(status)

        # 計測時間の表示 (文字が変わったときだけ更新する)
        if measurement is not None:
            elapsed = measurement.elapsed()
            total = last_stat.total if last_stat is not None else 0
            cpm, error = average_cpm(total, elapsed)
            recording = (f"REC {measurement.recorder.path.name}" if measurement.recorder
                         else "CSV off")
            line = (f"Elapsed {format_elapsed(elapsed)} | Total {total} | "
                    f"Avg {cpm:.1f} ± {error:.1f} CPM | {recording}")
        elif connection is not None:
            line = "Reset を押すと計測を開始します"
        else:
            line = ""
        if line != measure_text:
            window["-MEASURE-"].update(line)
            measure_text = line

    # ウィンドウは閉じているのでログは出さずに CSV だけ閉じる
    if measurement is not None:
        measurement.stop_recording()
    if connection is not None:
        connection.close()
    window.close()


if __name__ == "__main__":
    main()
