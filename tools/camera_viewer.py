"""ESP32-S3 + OV2640 のカメラ映像を COM ポート経由で表示するビューア。

使い方: python tools/camera_viewer.py
必要なパッケージ: pip install -r tools/requirements.txt
通信の仕様は src/main.cpp を参照。
"""

import collections
import dataclasses
import io
import pathlib
import queue
import re
import struct
import threading
import time
import zlib

import PySimpleGUI as sg
import serial
import serial.tools.list_ports
from PIL import Image

# src/main.cpp の FrameHeader と一致させること
HEADER = struct.Struct("<4sBBHHIIII")
MAGIC = b"RFRM"
FORMAT_GRAY = 0
FORMAT_JPEG = 1

MAX_TEXT_LINE = 4096
TEXT_LINE = re.compile(rb"[\x20-\x7e]{4,}$")
SILENCE_BEFORE_DROP_S = 1.0  # 受信が途切れたら書きかけのフレームを諦めるまでの時間
DEFAULT_BAUD = 921600
SIZE_NAMES = ["qqvga", "qvga", "cif", "vga", "svga", "uxga"]
VIEW_SIZE = (640, 480)
SNAPSHOT_DIR = pathlib.Path(__file__).resolve().parent / "snapshots"


@dataclasses.dataclass
class Frame:
    format: int
    width: int
    height: int
    seq: int
    payload: bytes


@dataclasses.dataclass
class CorruptFrame:
    reason: str


class StreamParser:
    """テキストとフレームが混在するバイト列を、イベントの列に分解する。

    イベントは str (ログ1行) / Frame / CorruptFrame のいずれか。
    カメラドライバのログがフレームに割り込むとデータがずれるので、CRC 不一致なら
    そのフレームを捨てて次の MAGIC から同期し直す。
    """

    def __init__(self):
        self._buf = bytearray()
        self._text = bytearray()

    def feed(self, data=b""):
        self._buf.extend(data)
        events = []
        while True:
            index = self._buf.find(MAGIC)
            if index < 0:
                self._consume_text(len(self._buf) - self._partial_magic_len(), events)
                return events
            self._consume_text(index, events)

            if len(self._buf) < HEADER.size:
                return events
            fields = HEADER.unpack_from(self._buf)
            _, fmt, _, width, height, seq, payload_len, payload_crc, header_crc = fields
            if zlib.crc32(bytes(self._buf[: HEADER.size - 4])) != header_crc:
                events.append(CorruptFrame("header CRC"))
                del self._buf[: len(MAGIC)]
                continue

            end = HEADER.size + payload_len
            if len(self._buf) < end:
                return events
            payload = bytes(self._buf[HEADER.size : end])
            if zlib.crc32(payload) != payload_crc:
                events.append(CorruptFrame("payload CRC"))
                # 本物の次フレームが payload に含まれた可能性があるので、MAGIC の直後から探し直す
                del self._buf[: len(MAGIC)]
                continue
            del self._buf[:end]
            events.append(Frame(fmt, width, height, seq, payload))

    def drop_partial_frame(self):
        """受信が途切れたときに呼ぶ。ESP32 のリセット等で届かなくなったフレームを諦める。"""
        if not self._buf.startswith(MAGIC):
            return []
        del self._buf[: len(MAGIC)]
        return [CorruptFrame("truncated")] + self.feed()

    def _partial_magic_len(self):
        """バッファ末尾が MAGIC の書きかけである場合の長さ。"""
        for length in range(len(MAGIC) - 1, 0, -1):
            if self._buf.endswith(MAGIC[:length]):
                return length
        return 0

    def _consume_text(self, count, events):
        if count == 0:
            return
        self._text.extend(self._buf[:count])
        del self._buf[:count]
        *lines, rest = self._text.split(b"\n")
        for line in lines:
            # 壊れたフレームの残りや、ボーレートの違う ROM ブートログを除き、行末の文字列だけ拾う
            match = TEXT_LINE.search(line.rstrip(b"\r"))
            if match:
                events.append(match.group().decode("ascii"))
        self._text = bytearray(rest) if len(rest) <= MAX_TEXT_LINE else bytearray()


def open_port(port, baud):
    device = serial.Serial()
    device.port = port
    device.baudrate = baud
    device.timeout = 0.1
    # DTR/RTS が立つと基板の自動リセット回路が働き、ESP32 がリセットされる
    device.dtr = False
    device.rts = False
    device.open()
    return device


def read_serial(device, events, stop):
    """シリアルを読み続け、(種類, 値) を events に積むスレッド本体。"""
    parser = StreamParser()
    last_data = time.monotonic()
    while not stop.is_set():
        try:
            data = device.read(device.in_waiting or 1)
        except (serial.SerialException, OSError) as error:
            events.put(("disconnected", str(error)))
            return

        now = time.monotonic()
        if data:
            last_data = now
            events.put(("bytes", len(data)))
            parsed = parser.feed(data)
        elif now - last_data > SILENCE_BEFORE_DROP_S:
            last_data = now
            parsed = parser.drop_partial_frame()
        else:
            parsed = []

        for event in parsed:
            events.put(("event", event))


def decode_frame(frame):
    if frame.format == FORMAT_GRAY:
        return Image.frombytes("L", (frame.width, frame.height), frame.payload)
    image = Image.open(io.BytesIO(frame.payload))
    image.load()
    return image


def to_png(image):
    """プレビュー領域に収まるよう拡大縮小する。拡大は画素が見えるよう整数倍にする。"""
    scale = min(VIEW_SIZE[0] / image.width, VIEW_SIZE[1] / image.height)
    if scale >= 2:
        factor = int(scale)
        image = image.resize((image.width * factor, image.height * factor), Image.NEAREST)
    elif scale < 1:
        size = (int(image.width * scale), int(image.height * scale))
        image = image.resize(size, Image.BILINEAR)
    buffer = io.BytesIO()
    image.save(buffer, format="PNG", compress_level=1)
    return buffer.getvalue()


def save_snapshot(frame):
    """センサーの出力をそのまま保存する (JPEG は再圧縮せず、グレーは PNG で可逆に)。"""
    SNAPSHOT_DIR.mkdir(exist_ok=True)
    stem = f"{time.strftime('%Y%m%d_%H%M%S')}_{frame.width}x{frame.height}_{frame.seq}"
    if frame.format == FORMAT_JPEG:
        path = SNAPSHOT_DIR / f"{stem}.jpg"
        path.write_bytes(frame.payload)
    else:
        path = SNAPSHOT_DIR / f"{stem}.png"
        decode_frame(frame).save(path)
    return path


def list_ports():
    return [port.device for port in serial.tools.list_ports.comports()]


class Connection:
    def __init__(self, port, baud):
        self.events = queue.Queue()
        self._stop = threading.Event()
        self._device = open_port(port, baud)
        self._thread = threading.Thread(
            target=read_serial, args=(self._device, self.events, self._stop), daemon=True
        )
        self._thread.start()

    def send(self, command):
        try:
            self._device.write(command.encode("ascii") + b"\n")
        except (serial.SerialException, OSError) as error:
            self.events.put(("disconnected", str(error)))

    def close(self):
        self.send("stop")
        self._stop.set()
        self._thread.join(timeout=1)
        self._device.close()


def build_window():
    ports = list_ports()
    layout = [
        [
            sg.Text("Port"),
            sg.Combo(ports, default_value=ports[-1] if ports else "", key="-PORT-", size=(10, 1)),
            sg.Button("Refresh", key="-REFRESH-"),
            sg.Text("Baud"),
            sg.Combo([115200, 921600, 2000000], default_value=DEFAULT_BAUD, key="-BAUD-", size=(9, 1)),
            sg.Button("Connect", key="-CONNECT-"),
        ],
        [
            sg.Text("Format"),
            sg.Combo(["jpeg", "gray"], default_value="jpeg", key="-FORMAT-",
                     enable_events=True, readonly=True, size=(6, 1)),
            sg.Text("Size"),
            sg.Combo(SIZE_NAMES, default_value="qvga", key="-SIZE-",
                     enable_events=True, readonly=True, size=(6, 1)),
            sg.Button("Start", key="-START-", disabled=True),
            sg.Button("Stop", key="-STOP-", disabled=True),
            sg.Button("Info", key="-INFO-", disabled=True),
            sg.Button("Measure FPS", key="-FPS-", disabled=True),
            sg.Button("Save", key="-SAVE-", disabled=True),
        ],
        [sg.Image(key="-IMAGE-", size=VIEW_SIZE, background_color="black")],
        [sg.Text("Not connected", key="-STATUS-", size=(80, 1))],
        [sg.Multiline(key="-LOG-", size=(90, 8), disabled=True, autoscroll=True,
                      font=("Consolas", 9))],
    ]
    return sg.Window("radiatSP32 camera viewer", layout, finalize=True)


def set_connected(window, connected):
    window["-CONNECT-"].update("Disconnect" if connected else "Connect")
    for key in ("-START-", "-STOP-", "-INFO-", "-FPS-"):
        window[key].update(disabled=not connected)


def main():
    window = build_window()
    connection = None
    latest = None
    shown_times = collections.deque()
    byte_times = collections.deque()
    corrupt_count = 0
    next_status = 0.0

    def log(text):
        window["-LOG-"].print(text)

    while True:
        event, values = window.read(timeout=20)
        if event == sg.WIN_CLOSED:
            break

        if event == "-REFRESH-":
            ports = list_ports()
            window["-PORT-"].update(values=ports, value=ports[-1] if ports else "")
        elif event == "-CONNECT-":
            if connection is not None:
                connection.close()
                connection = None
                set_connected(window, False)
                window["-STATUS-"].update("Not connected")
            else:
                try:
                    connection = Connection(values["-PORT-"], int(values["-BAUD-"]))
                except (serial.SerialException, ValueError) as error:
                    log(f"[PC] open failed: {error}")
                else:
                    set_connected(window, True)
                    # DTR/RTS を立てないので ESP32 はリセットされない。起動ログの代わりに状態を問い合わせる
                    for command in ("stop", f"fmt={values['-FORMAT-']}",
                                    f"size={values['-SIZE-']}", "info"):
                        connection.send(command)
        elif connection is not None:
            commands = {
                "-START-": "start",
                "-STOP-": "stop",
                "-INFO-": "info",
                "-FPS-": "fps",
                "-FORMAT-": f"fmt={values['-FORMAT-']}",
                "-SIZE-": f"size={values['-SIZE-']}",
            }
            if event in commands:
                connection.send(commands[event])

        if event == "-SAVE-" and latest is not None:
            log(f"[PC] saved {save_snapshot(latest)}")

        new_frame = None
        now = time.monotonic()
        if connection is not None:
            try:
                while True:
                    kind, value = connection.events.get_nowait()
                    if kind == "bytes":
                        byte_times.append((now, value))
                    elif kind == "disconnected":
                        log(f"[PC] disconnected: {value}")
                        connection.close()
                        connection = None
                        set_connected(window, False)
                        break
                    elif isinstance(value, str):
                        log(value)
                    elif isinstance(value, CorruptFrame):
                        corrupt_count += 1
                    else:
                        new_frame = value
            except queue.Empty:
                pass

        # 複数フレームが溜まっていても表示は最新の1枚だけ
        if new_frame is not None:
            try:
                image = decode_frame(new_frame)
            except (OSError, ValueError) as error:
                corrupt_count += 1
                log(f"[PC] decode failed: {error}")
            else:
                window["-IMAGE-"].update(data=to_png(image))
                latest = new_frame
                window["-SAVE-"].update(disabled=False)
                shown_times.append(now)

        if connection is not None and now >= next_status:
            next_status = now + 0.5
            window_s = 3.0
            while shown_times and now - shown_times[0] > window_s:
                shown_times.popleft()
            while byte_times and now - byte_times[0][0] > window_s:
                byte_times.popleft()
            kbytes = sum(count for _, count in byte_times) / 1024 / window_s
            status = f"view {len(shown_times) / window_s:.1f} fps | {kbytes:.0f} kB/s | corrupt {corrupt_count}"
            if latest is not None:
                name = "jpeg" if latest.format == FORMAT_JPEG else "gray"
                status += (f" | #{latest.seq} {latest.width}x{latest.height} {name} "
                           f"{len(latest.payload) / 1024:.1f} kB")
            window["-STATUS-"].update(status)

    if connection is not None:
        connection.close()
    window.close()


if __name__ == "__main__":
    main()
