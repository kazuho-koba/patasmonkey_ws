#!/usr/bin/env python3

"""
rosbag2（MCAP形式）に保存されたDepth画像とRGB画像を、同期して2画面で再生する。

対象トピック:
    /oak/depth/image_raw
    /oak/rgb/image_raw

主な機能:
    1. rosbagディレクトリまたは単一MCAPファイルを直接読み込む
    2. sensor_msgs/msg/ImageのDepth画像をNumPy配列へ変換する
    3. DepthAIのdisparity表示に近い見え方として、
       距離の逆数（inverse depth）を0～255へ正規化する
    4. OpenCVのCOLORMAP_JETを適用し、
       近距離を赤、遠距離を青、無効値を黒で表示する
    5. メッセージのheader.stampに基づき、元の時間間隔に近い速度で再生する
    6. 必要な場合だけ、色付き動画またはPNG連番を保存する
    7. GUIを表示せず、動画・画像保存だけを行うこともできる
    8. Depth画像本体は保持せず、header時刻・MCAP log_time・ファイルだけを索引化する
    9. OpenCVのシークバーから任意のDepthフレームへ移動できる
    10. DepthとRGBを別ウインドウへ表示し、再生状態・キー操作を共有する
    11. どちらのシークバーを操作しても両ウインドウが同じ時刻へ移動する
    12. d/lキーはDepth表示だけを変更し、RGB画像には影響しない

対応する主な画像エンコーディング:
    - 16UC1
    - mono16
    - 32FC1

ROSの一般的なDepth画像規約:
    - 16UC1 / mono16:
        距離単位をmmとして扱う
    - 32FC1:
        距離単位をmとして扱い、内部でmmへ変換する

実行例:
    表示のみ:
        python3 play_bag_depth.py \
            /workspaces/patasmonkey_ws/bags/rosbag2_2026_07_26-09_17_38

    色付きMP4も保存:
        python3 play_bag_depth.py \
            /workspaces/patasmonkey_ws/bags/rosbag2_2026_07_26-09_17_38 \
            --save-video depth_color.mp4

    PNG連番を保存:
        python3 play_bag_depth.py \
            /workspaces/patasmonkey_ws/bags/rosbag2_2026_07_26-09_17_38 \
            --save-frames depth_frames

    表示せず保存だけ:
        python3 play_bag_depth.py \
            /workspaces/patasmonkey_ws/bags/rosbag2_2026_07_26-09_17_38 \
            --save-video depth_color.mp4 \
            --no-display

    表示距離範囲を0.5～8mに設定:
        python3 play_bag_depth.py \
            /workspaces/patasmonkey_ws/bags/rosbag2_2026_07_26-09_17_38 \
            --near-mm 500 \
            --far-mm 8000

操作:
    q または Esc:
        終了

    Space:
        一時停止・再開

    n:
        一時停止中に1フレーム進む

    d:
        disparity（逆距離）表示へ切り替える

    l:
        linear-depth（距離線形）表示へ切り替える

    s:
        現在表示中の色付きフレームをPNG保存する

    シークバー:
        Depth側またはRGB側のどちらを動かしても、
        指定Depthフレーム時刻へ両ウインドウが同期して移動する

注意:
    - 本スクリプトはrosbag2_pyを使用しない。
      mcap_ros2.reader.read_ros2_messages()でMCAPを直接読む。
    - DepthAI実機のdisparity値そのものはbagに保存されていないため、
      Depth画像からinverse depthを計算して見た目を近づけている。
      カラーマップは同等でも、実機disparity表示と値が完全一致するとは限らない。
    - 動画保存は固定FPSであるため、不規則な記録間隔は完全には再現できない。
      FPSはメッセージのheader.stamp差の中央値で推定するか、--output-fpsで明示する。
    - シーク後に動画保存を継続した場合、保存動画には実際に表示した順番で
      フレームが記録される。後戻りや重複もそのまま含まれる。
    - MCAPのlog_timeは、索引化したメッセージをファイルから再読込するための
      検索キーとしてだけ使用する。同期、FPS、再生間隔、時間範囲、表示時刻は
      sensor_msgs/msg/Image.header.stampを使用する。
    - readerキャッシュにはmcap_ros2.decoder.DecoderFactoryを登録し、
      CDR形式のROS 2 Imageメッセージを正しく復号する。
"""

import argparse
import bisect
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from mcap_ros2.reader import read_ros2_messages


DEFAULT_TOPIC = "/oak/depth/image_raw"
DEFAULT_RGB_TOPIC = "/oak/color/image_raw"
DEFAULT_NEAR_MM = 400.0
DEFAULT_FAR_MM = 10000.0

DEPTH_WINDOW_NAME = "OAK-D Depth / Stereo visualization"
RGB_WINDOW_NAME = "OAK-D RGB visualization"
SEEK_TRACKBAR_NAME = "Frame"
DEFAULT_RGB_SYNC_TOLERANCE_MS = 100.0


@dataclass(frozen=True)
class ImageFrameIndex:
    """
    1枚の画像へ再アクセスするための軽量な索引。

    画像本体は保持せず、次の3情報だけを保持する。

        log_time_ns:
            MCAPのlog_time。対象メッセージをMCAPから再読込するための
            検索キーとしてだけ使用する。

        header_time_ns:
            sensor_msgs/msg/Image.header.stamp。
            フレーム順序、同期判定、FPS、再生間隔、時間範囲、表示時刻など、
            読み込み以外の評価・判定に使用する。

        mcap_path:
            この画像メッセージを格納している分割MCAPファイル。

    画像本体をRAMへ保持しないため、長時間bagでもメモリ消費を
    フレーム数にほぼ比例する小さな索引だけに抑えられる。
    """

    log_time_ns: int
    header_time_ns: int
    mcap_path: Path


class ImageFrameReaderCache:
    """
    ランダムアクセス時にMCAP readerを再利用する簡易キャッシュ。

    通常再生では同じ分割MCAPから連続してフレームを読むため、
    毎フレームファイルをopenし直すのは不要な負荷になる。

    シーク先のmcap_pathが現在と同じ場合:
        既存のstreamとMcapReaderを再利用する。

    別の分割MCAPへ移動した場合:
        現在のstreamを閉じ、新しいMCAPをopenしてreaderを作る。
    """

    def __init__(self) -> None:
        self._current_path: Optional[Path] = None
        self._stream = None
        self._reader = None

    def get_reader(self, mcap_path: Path):
        """
        指定MCAPに対応するMcapReaderを返す。
        """
        resolved = mcap_path.resolve()

        if (
            self._reader is not None
            and self._current_path == resolved
        ):
            return self._reader

        self.close()

        self._stream = resolved.open("rb")
        # 汎用McapReaderだけでは、channel.message_encoding="cdr"の
        # ROS 2メッセージをデコードできない。
        #
        # DecoderFactory()を登録することで、
        # sensor_msgs/msg/Imageなどros2msg schemaを持つCDRデータを
        # iter_decoded_messages/read_ros2_messages経由で復号できる。
        self._reader = make_reader(
            self._stream,
            decoder_factories=[
                DecoderFactory(),
            ],
        )
        self._current_path = resolved

        print(
            f"[OPEN] {resolved}"
        )

        return self._reader

    def close(self) -> None:
        """
        現在保持しているMCAP streamを閉じる。
        """
        if self._stream is not None:
            self._stream.close()

        self._current_path = None
        self._stream = None
        self._reader = None


@dataclass
class SeekState:
    """
    OpenCV trackbarのコールバックと再生ループの間で共有する状態。

    requested_position:
        ユーザーがシークバーで指定したフレーム位置。
        再生ループが処理した後はNoneへ戻す。

    internal_update:
        再生ループ自身がsetTrackbarPos()を呼んでいる間だけTrueにする。
        プログラムによるシークバー更新を、ユーザー操作と誤認しないためのフラグ。
    """

    requested_position: Optional[int] = None
    internal_update: bool = False


def parse_args() -> argparse.Namespace:
    """
    コマンドライン引数を定義して返す。

    保存機能はデフォルトで無効であり、
    --save-videoまたは--save-framesを指定した場合だけ保存する。
    """
    parser = argparse.ArgumentParser(
        description=(
            "Play a depth-image topic from an MCAP rosbag with a "
            "DepthAI-style false-color visualization."
        )
    )

    parser.add_argument(
        "bag",
        type=Path,
        help=(
            "Rosbag directory containing *.mcap files, "
            "or one .mcap file."
        ),
    )

    parser.add_argument(
        "--topic",
        default=DEFAULT_TOPIC,
        help=(
            f"Depth image topic. Default: {DEFAULT_TOPIC}"
        ),
    )

    parser.add_argument(
        "--rgb-topic",
        default=DEFAULT_RGB_TOPIC,
        help=(
            f"RGB image topic. Default: {DEFAULT_RGB_TOPIC}"
        ),
    )

    parser.add_argument(
        "--rgb-sync-tolerance-ms",
        type=float,
        default=DEFAULT_RGB_SYNC_TOLERANCE_MS,
        help=(
            "Maximum allowed timestamp difference between the Depth frame "
            "and the nearest RGB frame [ms]. "
            f"Default: {DEFAULT_RGB_SYNC_TOLERANCE_MS:g}"
        ),
    )

    parser.add_argument(
        "--near-mm",
        type=float,
        default=DEFAULT_NEAR_MM,
        help=(
            "Near limit of the displayed depth range [mm]. "
            f"Default: {DEFAULT_NEAR_MM:g}"
        ),
    )

    parser.add_argument(
        "--far-mm",
        type=float,
        default=DEFAULT_FAR_MM,
        help=(
            "Far limit of the displayed depth range [mm]. "
            f"Default: {DEFAULT_FAR_MM:g}"
        ),
    )

    parser.add_argument(
        "--scale-mode",
        choices=("disparity", "linear-depth"),
        default="disparity",
        help=(
            "Color normalization. 'disparity' uses inverse depth and "
            "resembles DepthAI disparity visualization. "
            "'linear-depth' maps metric depth linearly. "
            "Default: disparity"
        ),
    )

    parser.add_argument(
        "--colormap",
        choices=("jet", "turbo", "hot"),
        default="jet",
        help=(
            "OpenCV colormap. Default: jet"
        ),
    )

    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help=(
            "Playback speed multiplier. "
            "2.0 is twice as fast, 0.5 is half speed. Default: 1.0"
        ),
    )

    parser.add_argument(
        "--start-sec",
        type=float,
        default=0.0,
        help=(
            "Skip this many seconds from the first depth frame. Default: 0"
        ),
    )

    parser.add_argument(
        "--duration-sec",
        type=float,
        default=None,
        help=(
            "Process only this duration after --start-sec. "
            "Default: process to the end."
        ),
    )

    parser.add_argument(
        "--max-frames",
        type=int,
        default=None,
        help=(
            "Maximum number of processed frames. Default: no limit."
        ),
    )

    parser.add_argument(
        "--output-fps",
        type=float,
        default=None,
        help=(
            "FPS used for saved video. "
            "Default: estimate from message header timestamps and apply --speed."
        ),
    )

    parser.add_argument(
        "--save-video",
        type=Path,
        default=None,
        help=(
            "Save the colorized playback as an MP4 video. "
            "If omitted, no video is saved."
        ),
    )

    parser.add_argument(
        "--save-frames",
        type=Path,
        default=None,
        help=(
            "Save colorized frames as sequential PNG files in this directory. "
            "If omitted, PNG frames are not saved."
        ),
    )

    parser.add_argument(
        "--snapshot-dir",
        type=Path,
        default=Path("depth_snapshots"),
        help=(
            "Directory used when the 's' key saves the current frame. "
            "Default: depth_snapshots"
        ),
    )

    parser.add_argument(
        "--no-display",
        action="store_true",
        help=(
            "Do not open an OpenCV window. "
            "Useful when only saving video or frames."
        ),
    )

    parser.add_argument(
        "--overwrite",
        action="store_true",
        help=(
            "Allow existing output video or frame directory contents "
            "to be overwritten."
        ),
    )

    parser.add_argument(
        "--hide-overlay",
        action="store_true",
        help=(
            "Do not draw frame number, timestamp, range, and controls."
        ),
    )

    parser.add_argument(
        "--hide-colorbar",
        action="store_true",
        help=(
            "Do not append the distance color scale to the right side."
        ),
    )

    return parser.parse_args()


def natural_mcap_sort_key(path: Path) -> Tuple[str, int]:
    """
    分割されたMCAPファイルを末尾番号の数値順に並べる。

    文字列順では「_10」が「_2」より先になる場合があるため、
    ファイル名末尾の「_数字」を整数として解釈する。
    """
    stem = path.stem
    prefix, separator, suffix = stem.rpartition("_")

    if separator and suffix.isdigit():
        return prefix, int(suffix)

    return stem, -1


def find_mcap_files(bag_path: Path) -> List[Path]:
    """
    入力されたrosbagディレクトリまたはMCAPファイルから、
    処理対象となるMCAPファイル一覧を返す。

    Raises:
        FileNotFoundError:
            入力パスが存在しない場合。

        ValueError:
            対応しないファイル、またはMCAPを含まないディレクトリの場合。
    """
    resolved = bag_path.expanduser().resolve()

    if not resolved.exists():
        raise FileNotFoundError(
            f"Input path does not exist: {resolved}"
        )

    if resolved.is_file():
        if resolved.suffix.lower() != ".mcap":
            raise ValueError(
                f"Input file is not an MCAP file: {resolved}"
            )

        return [resolved]

    if not resolved.is_dir():
        raise ValueError(
            f"Input path is neither a directory nor an MCAP file: {resolved}"
        )

    mcap_files = sorted(
        resolved.glob("*.mcap"),
        key=natural_mcap_sort_key,
    )

    if not mcap_files:
        raise ValueError(
            f"No MCAP files found in directory: {resolved}"
        )

    return mcap_files


def validate_args(args: argparse.Namespace) -> None:
    """
    引数の値と保存先の状態を検証する。

    誤って既存結果を上書きしないよう、
    --overwriteがない場合は既存の動画ファイルを拒否する。
    """
    if args.near_mm <= 0.0:
        raise ValueError("--near-mm must be greater than zero.")

    if args.far_mm <= args.near_mm:
        raise ValueError("--far-mm must be greater than --near-mm.")

    if args.speed <= 0.0:
        raise ValueError("--speed must be greater than zero.")

    if args.rgb_sync_tolerance_ms < 0.0:
        raise ValueError(
            "--rgb-sync-tolerance-ms must be zero or greater."
        )

    if args.start_sec < 0.0:
        raise ValueError("--start-sec must be zero or greater.")

    if args.duration_sec is not None and args.duration_sec <= 0.0:
        raise ValueError("--duration-sec must be greater than zero.")

    if args.max_frames is not None and args.max_frames <= 0:
        raise ValueError("--max-frames must be greater than zero.")

    if args.output_fps is not None and args.output_fps <= 0.0:
        raise ValueError("--output-fps must be greater than zero.")

    if args.no_display and args.save_video is None and args.save_frames is None:
        raise ValueError(
            "--no-display requires --save-video and/or --save-frames."
        )

    if args.save_video is not None:
        video_path = args.save_video.expanduser().resolve()

        if video_path.exists() and not args.overwrite:
            raise FileExistsError(
                f"Output video already exists: {video_path}\n"
                "Use --overwrite to replace it."
            )

    if args.save_frames is not None:
        frame_dir = args.save_frames.expanduser().resolve()

        if frame_dir.exists():
            existing_pngs = list(frame_dir.glob("*.png"))

            if existing_pngs and not args.overwrite:
                raise FileExistsError(
                    f"PNG files already exist in: {frame_dir}\n"
                    "Use --overwrite to replace them."
                )


def message_header_stamp_ns(msg) -> int:
    """
    ROSメッセージのheader.stampを整数ナノ秒へ変換する。

    headerまたはstampを持たないメッセージは、MCAP log_timeへ暗黙に
    フォールバックせず例外にする。これにより、同期評価に異なる時刻基準が
    混在することを防ぐ。
    """
    try:
        stamp = msg.header.stamp
        sec = int(stamp.sec)
        nanosec = int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError) as error:
        raise ValueError(
            "Image message does not contain a usable header.stamp."
        ) from error

    if nanosec < 0 or nanosec >= 1_000_000_000:
        raise ValueError(
            f"Invalid header.stamp.nanosec: {nanosec}"
        )

    return sec * 1_000_000_000 + nanosec


def build_image_frame_index(
    mcap_files: Sequence[Path],
    topic: str,
) -> List[ImageFrameIndex]:
    """
    対象画像トピックの時刻索引を作成する。

    索引作成時にROS Imageメッセージを逐次デコードしてheader.stampを取得する。
    画像データは索引へ保持せず、その場で破棄する。

    各フレームについて保持する情報:
        - MCAP log_time: 後で同じメッセージを再読込するための検索キー
        - Image.header.stamp: 読み込み以外の評価・判定に使う時刻
        - 格納先MCAPファイル

    分割MCAPをすべて走査した後、header.stampで昇順に並べる。
    同一header時刻ではMCAP log_timeを第2キーにする。

    Returns:
        ImageFrameIndexのheader時刻順リスト。
    """
    frame_index: List[ImageFrameIndex] = []

    for mcap_path in mcap_files:
        print(f"[INDEX] topic={topic} file={mcap_path}")

        with mcap_path.open("rb") as stream:
            reader = make_reader(
                stream,
                decoder_factories=[
                    DecoderFactory(),
                ],
            )

            for decoded in read_ros2_messages(
                reader,
                topics={topic},
                log_time_order=True,
            ):
                if decoded.channel.topic != topic:
                    continue

                frame_index.append(
                    ImageFrameIndex(
                        log_time_ns=int(decoded.log_time_ns),
                        header_time_ns=message_header_stamp_ns(
                            decoded.ros_msg
                        ),
                        mcap_path=mcap_path,
                    )
                )

    frame_index.sort(
        key=lambda item: (
            item.header_time_ns,
            item.log_time_ns,
        )
    )

    return frame_index


def frame_index_header_timestamps(
    frame_index: Sequence[ImageFrameIndex],
) -> List[int]:
    """
    ImageFrameIndexからheader.stampだけの配列を作る。

    RGB最近傍探索、開始・終了範囲検索、FPS推定に使用する。
    MCAP log_timeはここでは使用しない。
    """
    return [
        item.header_time_ns
        for item in frame_index
    ]


def estimate_fps(timestamps: Sequence[int]) -> Optional[float]:
    """
    連続フレームのheader.stamp差中央値から記録FPSを推定する。

    平均値ではなく中央値を使うことで、
    一時的なドロップや長い停止区間の影響を減らす。
    """
    if len(timestamps) < 2:
        return None

    times = np.asarray(
        timestamps,
        dtype=np.int64,
    )

    intervals_sec = np.diff(times).astype(
        np.float64
    ) * 1.0e-9

    valid = intervals_sec[
        np.isfinite(intervals_sec)
        & (intervals_sec > 0.0)
    ]

    if valid.size == 0:
        return None

    median_interval = float(
        np.median(valid)
    )

    if median_interval <= 0.0:
        return None

    return 1.0 / median_interval


def image_message_to_depth_mm(msg) -> np.ndarray:
    """
    sensor_msgs/msg/Imageをmm単位の2次元Depth配列へ変換する。

    stepがwidth×1画素バイト数より大きい場合もあるため、
    単純reshapeではなくNumPyのstridesを使用して行間隔を反映する。

    対応:
        16UC1 / mono16:
            uint16、mm単位として解釈する。

        32FC1:
            float32、m単位として解釈して1000倍する。

    Returns:
        shape=(height, width)、dtype=float32、単位mm。

    Raises:
        ValueError:
            未対応encoding、サイズ不整合、異常なstepの場合。
    """
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)
    encoding = str(msg.encoding).strip().lower()
    is_bigendian = bool(msg.is_bigendian)

    if height <= 0 or width <= 0:
        raise ValueError(
            f"Invalid image size: width={width}, height={height}"
        )

    raw = bytes(msg.data)

    if encoding in ("16uc1", "mono16"):
        bytes_per_pixel = 2
        dtype = np.dtype(
            ">u2" if is_bigendian else "<u2"
        )
        unit_scale_to_mm = 1.0

    elif encoding == "32fc1":
        bytes_per_pixel = 4
        dtype = np.dtype(
            ">f4" if is_bigendian else "<f4"
        )
        unit_scale_to_mm = 1000.0

    else:
        raise ValueError(
            f"Unsupported depth image encoding: {msg.encoding!r}. "
            "Supported encodings are 16UC1, mono16, and 32FC1."
        )

    minimum_step = width * bytes_per_pixel

    if step < minimum_step:
        raise ValueError(
            f"Invalid Image.step={step}; expected at least {minimum_step}."
        )

    required_bytes = step * height

    if len(raw) < required_bytes:
        raise ValueError(
            f"Image data is too short: {len(raw)} bytes; "
            f"expected at least {required_bytes}."
        )

    # shapeは画素配列、stridesは「次の行」「次の画素」までの
    # バイト距離を指定する。これにより行末paddingを安全に無視できる。
    image = np.ndarray(
        shape=(height, width),
        dtype=dtype,
        buffer=raw,
        strides=(step, bytes_per_pixel),
    )

    # MCAPのバッファから独立させ、後続処理で安全に利用する。
    depth_mm = image.astype(
        np.float32,
        copy=True,
    )

    if unit_scale_to_mm != 1.0:
        depth_mm *= unit_scale_to_mm

    return depth_mm



def image_message_to_bgr(msg) -> np.ndarray:
    """
    sensor_msgs/msg/Imageのカラー画像をOpenCVのBGR配列へ変換する。

    OpenCVはBGR順を標準として使うため、rgb8やrgba8は色順を変換する。
    Image.stepがwidth×1画素バイト数より大きい場合も考慮し、
    行末paddingをstridesで読み飛ばす。

    対応encoding:
        bgr8 / rgb8 / bgra8 / rgba8 / mono8 / 8UC1 / 8UC3
    """
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)
    encoding = str(msg.encoding).strip().lower()

    if height <= 0 or width <= 0:
        raise ValueError(
            f"Invalid RGB image size: width={width}, height={height}"
        )

    raw = bytes(msg.data)

    if encoding in ("bgr8", "rgb8", "8uc3"):
        channels = 3
    elif encoding in ("bgra8", "rgba8"):
        channels = 4
    elif encoding in ("mono8", "8uc1"):
        channels = 1
    else:
        raise ValueError(
            f"Unsupported RGB image encoding: {msg.encoding!r}. "
            "Supported encodings are bgr8, rgb8, bgra8, rgba8, "
            "mono8, 8UC1, and 8UC3."
        )

    minimum_step = width * channels

    if step < minimum_step:
        raise ValueError(
            f"Invalid RGB Image.step={step}; "
            f"expected at least {minimum_step}."
        )

    required_bytes = step * height

    if len(raw) < required_bytes:
        raise ValueError(
            f"RGB image data is too short: {len(raw)} bytes; "
            f"expected at least {required_bytes}."
        )

    if channels == 1:
        image = np.ndarray(
            shape=(height, width),
            dtype=np.uint8,
            buffer=raw,
            strides=(step, 1),
        ).copy()

        return cv2.cvtColor(
            image,
            cv2.COLOR_GRAY2BGR,
        )

    image = np.ndarray(
        shape=(height, width, channels),
        dtype=np.uint8,
        buffer=raw,
        strides=(step, channels, 1),
    ).copy()

    if encoding == "rgb8":
        return cv2.cvtColor(
            image,
            cv2.COLOR_RGB2BGR,
        )

    if encoding == "rgba8":
        return cv2.cvtColor(
            image,
            cv2.COLOR_RGBA2BGR,
        )

    if encoding == "bgra8":
        return cv2.cvtColor(
            image,
            cv2.COLOR_BGRA2BGR,
        )

    # bgr8および8UC3は、すでにOpenCVのBGR順として扱う。
    return image


def find_nearest_frame_position(
    frame_index: Sequence[ImageFrameIndex],
    timestamps: Sequence[int],
    target_timestamp_ns: int,
) -> Tuple[int, int]:
    """
    target_timestamp_nsに最も近いフレーム位置と時刻差を返す。

    bisect_left()で挿入位置を求め、その直前・直後の2候補だけを比較する。
    全フレームを毎回走査しないため、シーク時も高速に対応できる。

    Returns:
        position:
            frame_index内の位置。

        delta_ns:
            選ばれた画像timestamp - target timestamp。
            正ならRGBがDepthより後、負なら前。
    """
    if not frame_index:
        raise ValueError("frame_index is empty.")

    insertion = bisect.bisect_left(
        timestamps,
        target_timestamp_ns,
    )

    candidates: List[int] = []

    if insertion < len(frame_index):
        candidates.append(insertion)

    if insertion > 0:
        candidates.append(insertion - 1)

    best_position = min(
        candidates,
        key=lambda position: abs(
            int(timestamps[position])
            - int(target_timestamp_ns)
        ),
    )

    delta_ns = (
        int(timestamps[best_position])
        - int(target_timestamp_ns)
    )

    return best_position, delta_ns


def normalize_depth_for_colormap(
    depth_mm: np.ndarray,
    near_mm: float,
    far_mm: float,
    scale_mode: str,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    mm単位Depth画像を0～255の8bit画像へ変換する。

    disparityモード:
        視差が距離の逆数に比例する性質を利用し、
        inverse depthを線形に0～255へ写像する。

        near_mm -> 255
        far_mm  -> 0

        DepthAIのdisparity表示に近く、近距離の変化を強調する。

    linear-depthモード:
        metric depthを線形に反転して0～255へ写像する。

        near_mm -> 255
        far_mm  -> 0

    Returns:
        normalized_u8:
            カラーマップ入力用uint8画像。

        valid_mask:
            0、NaN、Infを除いた有効Depth画素のマスク。
    """
    valid_mask = (
        np.isfinite(depth_mm)
        & (depth_mm > 0.0)
    )

    safe_depth = np.clip(
        depth_mm,
        near_mm,
        far_mm,
    )

    if scale_mode == "disparity":
        inverse_depth = 1.0 / safe_depth
        inverse_near = 1.0 / near_mm
        inverse_far = 1.0 / far_mm

        normalized = (
            inverse_depth - inverse_far
        ) / (
            inverse_near - inverse_far
        )

    elif scale_mode == "linear-depth":
        normalized = (
            far_mm - safe_depth
        ) / (
            far_mm - near_mm
        )

    else:
        raise ValueError(
            f"Unknown scale mode: {scale_mode}"
        )

    normalized = np.clip(
        normalized,
        0.0,
        1.0,
    )

    normalized_u8 = np.round(
        normalized * 255.0
    ).astype(np.uint8)

    # 無効画素を一旦0にする。
    # applyColorMap後に完全な黒へ置き換えるための準備でもある。
    normalized_u8[~valid_mask] = 0

    return normalized_u8, valid_mask


def get_opencv_colormap(name: str) -> int:
    """
    コマンドライン上のカラーマップ名をOpenCV定数へ変換する。
    """
    mapping = {
        "jet": cv2.COLORMAP_JET,
        "turbo": cv2.COLORMAP_TURBO,
        "hot": cv2.COLORMAP_HOT,
    }

    try:
        return mapping[name]
    except KeyError as error:
        raise ValueError(
            f"Unsupported colormap: {name}"
        ) from error


def colorize_depth(
    depth_mm: np.ndarray,
    near_mm: float,
    far_mm: float,
    scale_mode: str,
    colormap_name: str,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Depth画像へ疑似カラーを適用する。

    無効値はカラーマップ由来の色を残さず、明示的に黒へする。

    Returns:
        color_bgr:
            OpenCV表示・保存用BGR画像。

        valid_mask:
            有効Depth画素のマスク。
    """
    normalized_u8, valid_mask = normalize_depth_for_colormap(
        depth_mm=depth_mm,
        near_mm=near_mm,
        far_mm=far_mm,
        scale_mode=scale_mode,
    )

    color_bgr = cv2.applyColorMap(
        normalized_u8,
        get_opencv_colormap(colormap_name),
    )

    color_bgr[~valid_mask] = (
        0,
        0,
        0,
    )

    return color_bgr, valid_mask


def colorbar_depth_values(
    near_mm: float,
    far_mm: float,
    scale_mode: str,
) -> Tuple[float, float, float]:
    """
    カラーバー上端・中央・下端に表示する距離値を返す。

    disparityモードの中央色は距離の算術平均ではない。
    inverse depth空間の中央値を距離へ戻して求める。
    """
    if scale_mode == "disparity":
        middle_inverse = 0.5 * (
            (1.0 / near_mm)
            + (1.0 / far_mm)
        )
        middle_mm = 1.0 / middle_inverse

    else:
        middle_mm = 0.5 * (
            near_mm + far_mm
        )

    return near_mm, middle_mm, far_mm


def format_distance(mm: float) -> str:
    """
    距離を読みやすい単位へ整形する。

    1000mm以上はm、未満はmmで表示する。
    """
    if mm >= 1000.0:
        return f"{mm / 1000.0:.2f} m"

    return f"{mm:.0f} mm"


def add_colorbar(
    image_bgr: np.ndarray,
    near_mm: float,
    far_mm: float,
    scale_mode: str,
    colormap_name: str,
) -> np.ndarray:
    """
    画像右側へ疑似カラーの距離スケールを追加する。

    上端を近距離、下端を遠距離とし、
    実画像と同じ正規化方向・カラーマップを使用する。
    """
    height = image_bgr.shape[0]
    bar_width = 32
    label_width = 118
    margin = 8
    panel_width = bar_width + label_width + margin * 3

    panel = np.zeros(
        (height, panel_width, 3),
        dtype=np.uint8,
    )

    # 上が255（近距離）、下が0（遠距離）となる縦グラデーション。
    gradient = np.linspace(
        255,
        0,
        height,
        dtype=np.uint8,
    ).reshape(height, 1)

    gradient = np.repeat(
        gradient,
        bar_width,
        axis=1,
    )

    color_bar = cv2.applyColorMap(
        gradient,
        get_opencv_colormap(colormap_name),
    )

    panel[
        :,
        margin:margin + bar_width,
    ] = color_bar

    near_value, middle_value, far_value = colorbar_depth_values(
        near_mm=near_mm,
        far_mm=far_mm,
        scale_mode=scale_mode,
    )

    text_x = (
        margin + bar_width + margin
    )

    label_positions = [
        (
            format_distance(near_value),
            22,
        ),
        (
            format_distance(middle_value),
            max(22, height // 2),
        ),
        (
            format_distance(far_value),
            max(22, height - 10),
        ),
    ]

    for label, y in label_positions:
        cv2.putText(
            panel,
            label,
            (text_x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    cv2.putText(
        panel,
        "NEAR",
        (text_x, min(height - 10, 42)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.40,
        (200, 200, 200),
        1,
        cv2.LINE_AA,
    )

    cv2.putText(
        panel,
        "FAR",
        (text_x, max(18, height - 30)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.40,
        (200, 200, 200),
        1,
        cv2.LINE_AA,
    )

    return np.hstack(
        (image_bgr, panel)
    )


def draw_overlay(
    image_bgr: np.ndarray,
    frame_index: int,
    total_frames: int,
    relative_time_sec: float,
    encoding: str,
    near_mm: float,
    far_mm: float,
    scale_mode: str,
    valid_ratio: float,
    paused: bool,
) -> np.ndarray:
    """
    再生状況とDepth設定を画像左上へ描画する。

    元の色付き画像を保持するため、コピーへ描画する。
    """
    output = image_bgr.copy()

    lines = [
        (
            f"Frame: {frame_index + 1}/{total_frames}   "
            f"Time: {relative_time_sec:.3f} s"
        ),
        (
            f"Encoding: {encoding}   "
            f"Valid: {valid_ratio * 100.0:.1f}%"
        ),
        (
            f"Range: {near_mm:.0f}-{far_mm:.0f} mm   "
            f"Scale: {scale_mode}"
        ),
        (
            "Space: pause/resume   n: step   "
            "d: disparity   l: linear"
        ),
        (
            "Seek bar: jump to frame   s: snapshot   q/Esc: quit"
        ),
    ]

    if paused:
        lines.append("PAUSED")

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.48
    thickness = 1
    line_height = 21
    left = 10
    top = 10

    maximum_width = 0

    for line in lines:
        text_size, _ = cv2.getTextSize(
            line,
            font,
            font_scale,
            thickness,
        )
        maximum_width = max(
            maximum_width,
            text_size[0],
        )

    overlay = output.copy()

    cv2.rectangle(
        overlay,
        (left - 5, top - 5),
        (
            left + maximum_width + 8,
            top + line_height * len(lines) + 2,
        ),
        (0, 0, 0),
        -1,
    )

    cv2.addWeighted(
        overlay,
        0.60,
        output,
        0.40,
        0.0,
        output,
    )

    for index, line in enumerate(lines):
        y = top + 15 + index * line_height

        color = (
            (0, 255, 255)
            if line == "PAUSED"
            else (255, 255, 255)
        )

        cv2.putText(
            output,
            line,
            (left, y),
            font,
            font_scale,
            color,
            thickness,
            cv2.LINE_AA,
        )

    return output



def draw_rgb_overlay(
    image_bgr: np.ndarray,
    frame_index: int,
    total_frames: int,
    relative_time_sec: float,
    encoding: str,
    sync_offset_ms: float,
    sync_tolerance_ms: float,
    paused: bool,
) -> np.ndarray:
    """
    RGB画像へ再生状態とDepth画像との時刻差を描画する。

    d/lキーはDepth側だけに意味があるため、RGB側では操作案内に
    "depth only"と明記する。
    """
    output = image_bgr.copy()

    synchronized = (
        abs(sync_offset_ms)
        <= sync_tolerance_ms
    )

    sync_text = (
        f"RGB-Depth offset: {sync_offset_ms:+.1f} ms"
    )

    if not synchronized:
        sync_text += "  OUT OF SYNC"

    lines = [
        (
            f"Depth frame: {frame_index + 1}/{total_frames}   "
            f"Time: {relative_time_sec:.3f} s"
        ),
        (
            f"RGB encoding: {encoding}   "
            f"{sync_text}"
        ),
        (
            "Space: pause/resume   n: step   "
            "d/l: depth window only"
        ),
        (
            "Seek bar: linked jump   s: save both   q/Esc: quit"
        ),
    ]

    if paused:
        lines.append("PAUSED")

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.48
    thickness = 1
    line_height = 21
    left = 10
    top = 10

    maximum_width = 0

    for line in lines:
        text_size, _ = cv2.getTextSize(
            line,
            font,
            font_scale,
            thickness,
        )
        maximum_width = max(
            maximum_width,
            text_size[0],
        )

    overlay = output.copy()

    cv2.rectangle(
        overlay,
        (left - 5, top - 5),
        (
            left + maximum_width + 8,
            top + line_height * len(lines) + 2,
        ),
        (0, 0, 0),
        -1,
    )

    cv2.addWeighted(
        overlay,
        0.60,
        output,
        0.40,
        0.0,
        output,
    )

    for index, line in enumerate(lines):
        y = top + 15 + index * line_height

        if line == "PAUSED":
            color = (0, 255, 255)
        elif "OUT OF SYNC" in line:
            color = (0, 0, 255)
        else:
            color = (255, 255, 255)

        cv2.putText(
            output,
            line,
            (left, y),
            font,
            font_scale,
            color,
            thickness,
            cv2.LINE_AA,
        )

    return output


def prepare_output_paths(
    args: argparse.Namespace,
) -> Tuple[Optional[Path], Optional[Path], Path]:
    """
    動画・PNG連番・手動snapshotの保存先を作成して返す。
    """
    video_path: Optional[Path] = None
    frame_dir: Optional[Path] = None

    if args.save_video is not None:
        video_path = args.save_video.expanduser().resolve()
        video_path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

    if args.save_frames is not None:
        frame_dir = args.save_frames.expanduser().resolve()
        frame_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

    snapshot_dir = args.snapshot_dir.expanduser().resolve()

    return video_path, frame_dir, snapshot_dir


def create_video_writer(
    output_path: Path,
    frame_size: Tuple[int, int],
    fps: float,
) -> cv2.VideoWriter:
    """
    MP4用VideoWriterを作成する。

    OpenCVで広く利用可能なmp4vを使用し、
    Writerが開けなければ例外を送出する。
    """
    fourcc = cv2.VideoWriter_fourcc(
        *"mp4v"
    )

    writer = cv2.VideoWriter(
        str(output_path),
        fourcc,
        fps,
        frame_size,
        True,
    )

    if not writer.isOpened():
        raise RuntimeError(
            f"Could not open video writer: {output_path}"
        )

    return writer


def read_image_message_at(
    frame_entry: ImageFrameIndex,
    topic: str,
    reader_cache: ImageFrameReaderCache,
) -> Tuple[int, int, object]:
    """
    索引で指定された画像メッセージをMCAPから1枚だけ読み込む。

    この関数だけはMCAP log_timeを検索条件として使用する。
    read_ros2_messages()へstart_timeとend_timeを指定し、索引に記録した
    [log_time, log_time+1ns)の範囲から厳密に同じメッセージを取得する。

    取得後はメッセージ自身のheader.stampを抽出し、索引作成時の値と
    一致することを確認する。別メッセージへの暗黙のフォールバックは行わない。

    Returns:
        (実際のMCAP log_time[ns], header.stamp[ns], ROS Imageメッセージ)

    Raises:
        RuntimeError:
            索引に対応する画像メッセージを取得できない、または
            header.stampが索引と一致しない場合。
    """
    target_log_time_ns = int(
        frame_entry.log_time_ns
    )

    reader = reader_cache.get_reader(
        frame_entry.mcap_path
    )

    for decoded in read_ros2_messages(
        reader,
        topics={topic},
        start_time=target_log_time_ns,
        end_time=target_log_time_ns + 1,
        log_time_order=True,
    ):
        if (
            decoded.channel.topic != topic
            or int(decoded.log_time_ns) != target_log_time_ns
        ):
            continue

        msg = decoded.ros_msg
        header_time_ns = message_header_stamp_ns(
            msg
        )

        if header_time_ns != int(frame_entry.header_time_ns):
            raise RuntimeError(
                "Indexed header timestamp does not match the reloaded "
                "message: "
                f"indexed={frame_entry.header_time_ns}, "
                f"actual={header_time_ns}, "
                f"log_time={target_log_time_ns}, "
                f"file={frame_entry.mcap_path}"
            )

        return (
            int(decoded.log_time_ns),
            header_time_ns,
            msg,
        )

    raise RuntimeError(
        "Could not load indexed image frame by MCAP log_time: "
        f"log_time={target_log_time_ns}, file={frame_entry.mcap_path}"
    )


def select_frame_range(
    frame_index: Sequence[ImageFrameIndex],
    start_sec: float,
    duration_sec: Optional[float],
    max_frames: Optional[int],
) -> List[ImageFrameIndex]:
    """
    --start-sec、--duration-sec、--max-framesから処理対象範囲を切り出す。

    header.stamp配列に対してbisectを使うため、フレーム数が多くても
    開始位置・終了位置を線形走査せずに求められる。

    Returns:
        選択されたImageFrameIndexの新しいリスト。
    """
    if not frame_index:
        return []

    timestamps = frame_index_header_timestamps(
        frame_index
    )

    timeline_start_ns = timestamps[0]

    selected_start_ns = (
        timeline_start_ns
        + int(round(start_sec * 1.0e9))
    )

    start_position = bisect.bisect_left(
        timestamps,
        selected_start_ns,
    )

    end_position = len(
        frame_index
    )

    if duration_sec is not None:
        selected_end_ns = (
            selected_start_ns
            + int(round(duration_sec * 1.0e9))
        )

        end_position = bisect.bisect_right(
            timestamps,
            selected_end_ns,
        )

    if max_frames is not None:
        end_position = min(
            end_position,
            start_position + max_frames,
        )

    return list(
        frame_index[start_position:end_position]
    )

def calculate_wait_milliseconds(
    previous_timestamp_ns: Optional[int],
    current_timestamp_ns: int,
    speed: float,
) -> int:
    """
    直前フレームとのheader.stamp差からOpenCV待機時間[ms]を計算する。

    極端に長い停止区間でGUIが操作不能になることを避けるため、
    1回のwaitKeyは最大1000msに制限する。
    """
    if previous_timestamp_ns is None:
        return 1

    delta_sec = (
        current_timestamp_ns - previous_timestamp_ns
    ) * 1.0e-9

    if not math.isfinite(delta_sec) or delta_sec <= 0.0:
        return 1

    adjusted_sec = delta_sec / speed

    return max(
        1,
        min(
            1000,
            int(round(adjusted_sec * 1000.0)),
        ),
    )


def wait_for_playback_key(
    delay_ms: int,
    paused: bool,
    seek_state: SeekState,
) -> Tuple[Optional[str], bool]:
    """
    OpenCVキーボード入力を処理する。

    モード切替キーは、再生中・一時停止中のどちらでも受け付ける。

    Returns:
        command:
            "quit":
                再生を終了する。

            "snapshot":
                現在表示中の疑似カラー画像を保存する。

            "step":
                一時停止状態のまま次のDepthフレームへ進む。

            "disparity":
                inverse depthを用いるdisparity風表示へ切り替える。

            "linear-depth":
                距離を線形に色へ割り当てる表示へ切り替える。

            "seek":
                シークバーで別フレームが指定された。

            "refresh":
                一時停止表示など、同じフレームを再描画する。

            None:
                通常再生を継続する。

        paused:
            更新後の一時停止状態。
    """
    while True:
        # 一時停止中は短い周期でキー入力を確認し続ける。
        # 再生中はheader.stamp差に対応する時間だけ待機する。
        wait_ms = 30 if paused else delay_ms

        key = cv2.waitKey(
            wait_ms
        ) & 0xFF

        # OpenCVのtrackbar callbackはwaitKey()によるGUIイベント処理中に
        # 呼ばれるため、キー判定より先にシーク要求を確認する。
        if seek_state.requested_position is not None:
            return "seek", paused

        if key in (
            27,
            ord("q"),
        ):
            return "quit", paused

        if key == ord(" "):
            paused = not paused

            # 再開時は現在フレームから抜けて、次フレームへ進む。
            if not paused:
                return None, paused

            # 一時停止へ入った直後は、同じ画像をPAUSED表示付きで
            # 再描画するため、外側ループへrefreshを返す。
            return "refresh", paused

        if key == ord("d"):
            return "disparity", paused

        if key == ord("l"):
            return "linear-depth", paused

        if key == ord("s"):
            return "snapshot", paused

        if key == ord("n") and paused:
            return "step", paused

        if not paused:
            return None, paused


def save_snapshot(
    frame_bgr: np.ndarray,
    snapshot_dir: Path,
    frame_index: int,
    timestamp_ns: int,
    prefix: str,
) -> Path:
    """
    キー操作で指定された現在フレームをPNGとして保存する。

    prefixへ"depth"または"rgb"を渡すことで、同じ時刻の2画像を
    区別できるファイル名にする。
    """
    snapshot_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    output_path = snapshot_dir / (
        f"{prefix}_snapshot_"
        f"{frame_index:06d}_"
        f"{timestamp_ns}.png"
    )

    if not cv2.imwrite(
        str(output_path),
        frame_bgr,
    ):
        raise RuntimeError(
            f"Failed to write snapshot: {output_path}"
        )

    return output_path


def run_playback(
    args: argparse.Namespace,
    depth_frame_index: Sequence[ImageFrameIndex],
    rgb_frame_index: Sequence[ImageFrameIndex],
    estimated_fps: Optional[float],
) -> None:
    """
    Depthを基準時系列として、Depth画像とRGB画像を同期再生する。

    同期方法:
        現在のDepth header.stampに最も近いRGB header.stampを二分探索で選ぶ。
        RGB画像側のフレームレートがDepthと異なっていても、
        各Depthフレームに最も近いカラー画像を表示できる。

    キーボード:
        OpenCVのcv2.waitKey()はHighGUI全体のイベントを処理するため、
        Depth/RGBのどちらのウインドウがアクティブでも、
        同じpaused・current_position・scale_modeへ操作を反映する。

    シークバー:
        2つのウインドウに同じDepthフレーム番号のtrackbarを作る。
        片方のcallbackが共有SeekStateへ要求を書き込み、
        再生ループが両方を同じ位置へ更新する。

    d/lキー:
        Depth画像の疑似カラースケールだけを変更する。
        RGB画像は同じフレームをそのまま表示し続ける。
    """
    video_path, frame_dir, snapshot_dir = prepare_output_paths(
        args
    )

    selected_depth_frames = select_frame_range(
        frame_index=depth_frame_index,
        start_sec=args.start_sec,
        duration_sec=args.duration_sec,
        max_frames=args.max_frames,
    )

    if not selected_depth_frames:
        raise RuntimeError(
            "No depth frames remain after applying "
            "--start-sec/--duration-sec/--max-frames."
        )

    rgb_header_timestamps = frame_index_header_timestamps(
        rgb_frame_index
    )

    writer: Optional[cv2.VideoWriter] = None

    if args.output_fps is not None:
        video_fps = float(
            args.output_fps
        )
    elif estimated_fps is not None:
        video_fps = estimated_fps * args.speed
    else:
        video_fps = 20.0 * args.speed

    full_timeline_start_ns = int(
        depth_frame_index[0].header_time_ns
    )

    previous_displayed_header_time_ns: Optional[int] = None
    current_position = 0
    processed_count = 0
    paused = False
    quit_requested = False
    current_scale_mode = args.scale_mode

    seek_state = SeekState()

    # DepthとRGBは同じMCAPファイルを参照する場合があるが、
    # それぞれ独立したreaderを持たせることでイテレータ状態の干渉を避ける。
    depth_reader_cache = ImageFrameReaderCache()
    rgb_reader_cache = ImageFrameReaderCache()

    rgb_out_of_sync_count = 0
    maximum_rgb_offset_ms = 0.0

    def on_seek_trackbar(position: int) -> None:
        """
        どちらかのシークバーが操作されたとき、共有位置へ要求を書く。
        """
        if seek_state.internal_update:
            return

        seek_state.requested_position = int(
            position
        )

    def update_trackbars(position: int) -> None:
        """
        Depth側とRGB側のシークバーを同じ位置へそろえる。

        setTrackbarPos()でもcallbackが呼ばれる実装があるため、
        internal_update中はcallbackからの要求を無視する。
        """
        if args.no_display:
            return

        seek_state.internal_update = True

        try:
            cv2.setTrackbarPos(
                SEEK_TRACKBAR_NAME,
                DEPTH_WINDOW_NAME,
                int(position),
            )
            cv2.setTrackbarPos(
                SEEK_TRACKBAR_NAME,
                RGB_WINDOW_NAME,
                int(position),
            )
        finally:
            seek_state.internal_update = False

    try:
        if not args.no_display:
            cv2.namedWindow(
                DEPTH_WINDOW_NAME,
                cv2.WINDOW_NORMAL,
            )
            cv2.namedWindow(
                RGB_WINDOW_NAME,
                cv2.WINDOW_NORMAL,
            )

            trackbar_max = max(
                1,
                len(selected_depth_frames) - 1,
            )

            cv2.createTrackbar(
                SEEK_TRACKBAR_NAME,
                DEPTH_WINDOW_NAME,
                0,
                trackbar_max,
                on_seek_trackbar,
            )
            cv2.createTrackbar(
                SEEK_TRACKBAR_NAME,
                RGB_WINDOW_NAME,
                0,
                trackbar_max,
                on_seek_trackbar,
            )

            # 2画面を並べやすい初期位置へ置く。
            # ウインドウマネージャによっては位置指定が無視される場合がある。
            cv2.moveWindow(
                DEPTH_WINDOW_NAME,
                0,
                0,
            )
            cv2.moveWindow(
                RGB_WINDOW_NAME,
                850,
                0,
            )

            # createTrackbar()直後のcallbackをユーザー操作と誤認しない。
            seek_state.requested_position = None

        while current_position < len(selected_depth_frames):
            update_trackbars(
                current_position
            )

            depth_entry = selected_depth_frames[
                current_position
            ]

            (
                _depth_log_time_ns,
                depth_header_time_ns,
                depth_msg,
            ) = read_image_message_at(
                frame_entry=depth_entry,
                topic=args.topic,
                reader_cache=depth_reader_cache,
            )

            depth_mm = image_message_to_depth_mm(
                depth_msg
            )

            _rgb_log_time_ns: Optional[int] = None
            rgb_header_time_ns: Optional[int] = None
            rgb_msg = None
            rgb_bgr: Optional[np.ndarray] = None
            rgb_sync_offset_ms = math.nan

            if not args.no_display:
                rgb_position, _rgb_delta_ns = find_nearest_frame_position(
                    frame_index=rgb_frame_index,
                    timestamps=rgb_header_timestamps,
                    target_timestamp_ns=depth_header_time_ns,
                )

                rgb_entry = rgb_frame_index[
                    rgb_position
                ]

                (
                    _rgb_log_time_ns,
                    rgb_header_time_ns,
                    rgb_msg,
                ) = read_image_message_at(
                    frame_entry=rgb_entry,
                    topic=args.rgb_topic,
                    reader_cache=rgb_reader_cache,
                )

                rgb_bgr = image_message_to_bgr(
                    rgb_msg
                )

                # 同期誤差は両メッセージ自身のheader.stampで計算する。
                rgb_sync_offset_ms = (
                    rgb_header_time_ns
                    - depth_header_time_ns
                ) * 1.0e-6

                maximum_rgb_offset_ms = max(
                    maximum_rgb_offset_ms,
                    abs(rgb_sync_offset_ms),
                )

                if (
                    abs(rgb_sync_offset_ms)
                    > args.rgb_sync_tolerance_ms
                ):
                    rgb_out_of_sync_count += 1

            frame_finished = False
            seek_happened = False
            depth_display_frame: Optional[np.ndarray] = None
            rgb_display_frame: Optional[np.ndarray] = None

            while not frame_finished:
                depth_color_bgr, valid_mask = colorize_depth(
                    depth_mm=depth_mm,
                    near_mm=args.near_mm,
                    far_mm=args.far_mm,
                    scale_mode=current_scale_mode,
                    colormap_name=args.colormap,
                )

                valid_ratio = float(
                    np.count_nonzero(valid_mask)
                ) / float(valid_mask.size)

                relative_time_sec = (
                    depth_header_time_ns
                    - full_timeline_start_ns
                ) * 1.0e-9

                depth_display_frame = depth_color_bgr

                if not args.hide_colorbar:
                    depth_display_frame = add_colorbar(
                        image_bgr=depth_display_frame,
                        near_mm=args.near_mm,
                        far_mm=args.far_mm,
                        scale_mode=current_scale_mode,
                        colormap_name=args.colormap,
                    )

                if not args.hide_overlay:
                    depth_display_frame = draw_overlay(
                        image_bgr=depth_display_frame,
                        frame_index=current_position,
                        total_frames=len(selected_depth_frames),
                        relative_time_sec=relative_time_sec,
                        encoding=str(depth_msg.encoding),
                        near_mm=args.near_mm,
                        far_mm=args.far_mm,
                        scale_mode=current_scale_mode,
                        valid_ratio=valid_ratio,
                        paused=paused,
                    )

                if args.no_display:
                    frame_finished = True
                    break

                if (
                    rgb_bgr is None
                    or rgb_msg is None
                    or rgb_header_time_ns is None
                ):
                    raise RuntimeError(
                        "RGB frame was not loaded."
                    )

                rgb_display_frame = rgb_bgr

                if not args.hide_overlay:
                    rgb_display_frame = draw_rgb_overlay(
                        image_bgr=rgb_display_frame,
                        frame_index=current_position,
                        total_frames=len(selected_depth_frames),
                        relative_time_sec=relative_time_sec,
                        encoding=str(rgb_msg.encoding),
                        sync_offset_ms=rgb_sync_offset_ms,
                        sync_tolerance_ms=args.rgb_sync_tolerance_ms,
                        paused=paused,
                    )

                cv2.imshow(
                    DEPTH_WINDOW_NAME,
                    depth_display_frame,
                )
                cv2.imshow(
                    RGB_WINDOW_NAME,
                    rgb_display_frame,
                )

                delay_ms = calculate_wait_milliseconds(
                    previous_timestamp_ns=previous_displayed_header_time_ns,
                    current_timestamp_ns=depth_header_time_ns,
                    speed=args.speed,
                )

                # waitKeyはOpenCVの全ウインドウに対するキーイベントを処理する。
                # どちらの画面へフォーカスしていても、返されたcommandは共有される。
                command, paused = wait_for_playback_key(
                    delay_ms=delay_ms,
                    paused=paused,
                    seek_state=seek_state,
                )

                if command == "quit":
                    quit_requested = True
                    frame_finished = True
                    break

                if command == "seek":
                    requested = seek_state.requested_position
                    seek_state.requested_position = None

                    if requested is not None:
                        current_position = max(
                            0,
                            min(
                                len(selected_depth_frames) - 1,
                                int(requested),
                            ),
                        )

                        previous_displayed_header_time_ns = None
                        seek_happened = True
                        frame_finished = True

                        print(
                            "[SEEK] "
                            f"frame={current_position + 1}/"
                            f"{len(selected_depth_frames)}, "
                            f"time="
                            f"{(selected_depth_frames[current_position].header_time_ns - full_timeline_start_ns) * 1.0e-9:.3f} s"
                        )

                    break

                if command in (
                    "disparity",
                    "linear-depth",
                ):
                    if current_scale_mode != command:
                        current_scale_mode = command
                        print(
                            f"[SCALE MODE] {current_scale_mode}"
                        )

                    # Depthだけを再色付けする。
                    # RGBは同じrgb_bgrをそのまま再表示する。
                    continue

                if command == "snapshot":
                    depth_snapshot_path = save_snapshot(
                        frame_bgr=depth_display_frame,
                        snapshot_dir=snapshot_dir,
                        frame_index=current_position,
                        timestamp_ns=depth_header_time_ns,
                        prefix="depth",
                    )

                    rgb_snapshot_path = save_snapshot(
                        frame_bgr=rgb_display_frame,
                        snapshot_dir=snapshot_dir,
                        frame_index=current_position,
                        timestamp_ns=rgb_header_time_ns,
                        prefix="rgb",
                    )

                    print(
                        f"[SNAPSHOT] {depth_snapshot_path}"
                    )
                    print(
                        f"[SNAPSHOT] {rgb_snapshot_path}"
                    )

                    continue

                if command == "refresh":
                    # Spaceで一時停止へ入った直後など、
                    # 同じDepth/RGBフレームへPAUSED表示を付けて再描画する。
                    continue

                if command == "step":
                    # paused=Trueを保ったまま、Depth基準で1フレーム進む。
                    frame_finished = True
                    break

                # command=Noneなら通常再生または一時停止解除。
                frame_finished = True

            if quit_requested:
                break

            if seek_happened:
                continue

            if depth_display_frame is None:
                raise RuntimeError(
                    "Depth display frame was not generated."
                )

            # 従来との互換性を保ち、--save-video/--save-framesは
            # Depth疑似カラー画像を保存する。
            if video_path is not None:
                if writer is None:
                    frame_height, frame_width = (
                        depth_display_frame.shape[:2]
                    )

                    writer = create_video_writer(
                        output_path=video_path,
                        frame_size=(
                            frame_width,
                            frame_height,
                        ),
                        fps=video_fps,
                    )

                    print(
                        f"[SAVE DEPTH VIDEO] {video_path}"
                    )
                    print(
                        f"[VIDEO FPS] {video_fps:.3f}"
                    )

                writer.write(
                    depth_display_frame
                )

            if frame_dir is not None:
                frame_path = frame_dir / (
                    f"depth_{processed_count:06d}_"
                    f"source_{current_position:06d}_"
                    f"{depth_header_time_ns}.png"
                )

                if not cv2.imwrite(
                    str(frame_path),
                    depth_display_frame,
                ):
                    raise RuntimeError(
                        f"Failed to write PNG: {frame_path}"
                    )

            previous_displayed_header_time_ns = (
                depth_header_time_ns
            )
            processed_count += 1
            current_position += 1

            if processed_count % 100 == 0:
                print(
                    f"[PROGRESS] {processed_count} displayed depth frames"
                )

    finally:
        depth_reader_cache.close()
        rgb_reader_cache.close()

        if writer is not None:
            writer.release()

        if not args.no_display:
            cv2.destroyAllWindows()

    print("")
    print("=" * 72)
    print("Depth/RGB synchronized playback finished")
    print("=" * 72)
    print(f"Indexed depth frames : {len(depth_frame_index)}")
    print(f"Indexed RGB frames   : {len(rgb_frame_index)}")
    print(f"Selected depth frames: {len(selected_depth_frames)}")
    print(f"Displayed frames     : {processed_count}")
    print(f"Quit requested       : {quit_requested}")
    print(f"Final scale mode     : {current_scale_mode}")

    if not args.no_display:
        print(
            f"RGB out-of-sync count: {rgb_out_of_sync_count} "
            f"(>{args.rgb_sync_tolerance_ms:.1f} ms)"
        )
        print(
            f"Maximum RGB offset   : {maximum_rgb_offset_ms:.3f} ms"
        )

    if video_path is not None:
        print(f"Saved depth video    : {video_path}")

    if frame_dir is not None:
        print(f"Saved depth PNG dir  : {frame_dir}")


def main() -> int:
    """
    スクリプトのエントリーポイント。

    処理順:
        1. 引数を検証する
        2. MCAPファイル一覧を取得する
        3. DepthとRGBそれぞれのlog_time/header.stamp索引を作る
        4. Depthを基準時系列としてFPSと時間範囲を確認する
        5. 2ウインドウを共通操作で再生・シークする
    """
    args = parse_args()

    try:
        validate_args(
            args
        )

        mcap_files = find_mcap_files(
            args.bag
        )

        print(
            f"[DEPTH TOPIC] {args.topic}"
        )
        print(
            f"[RGB TOPIC] {args.rgb_topic}"
        )
        print(
            "[TIMESTAMP BASIS] MCAP log_time=read locator only; "
            "Image.header.stamp=sync/FPS/playback/range/display"
        )

        depth_frame_index = build_image_frame_index(
            mcap_files=mcap_files,
            topic=args.topic,
        )
        rgb_frame_index = build_image_frame_index(
            mcap_files=mcap_files,
            topic=args.rgb_topic,
        )

        if not depth_frame_index:
            raise RuntimeError(
                f"No messages found on depth topic: {args.topic}"
            )

        if not rgb_frame_index:
            raise RuntimeError(
                f"No messages found on RGB topic: {args.rgb_topic}"
            )

        depth_header_timestamps = frame_index_header_timestamps(
            depth_frame_index
        )
        rgb_header_timestamps = frame_index_header_timestamps(
            rgb_frame_index
        )

        estimated_fps = estimate_fps(
            depth_header_timestamps
        )
        rgb_estimated_fps = estimate_fps(
            rgb_header_timestamps
        )

        depth_duration_sec = (
            depth_header_timestamps[-1]
            - depth_header_timestamps[0]
        ) * 1.0e-9

        rgb_duration_sec = (
            rgb_header_timestamps[-1]
            - rgb_header_timestamps[0]
        ) * 1.0e-9

        print(
            f"[DEPTH MESSAGES] {len(depth_header_timestamps)}"
        )
        print(
            f"[RGB MESSAGES] {len(rgb_header_timestamps)}"
        )
        print(
            f"[DEPTH HEADER DURATION] {depth_duration_sec:.3f} s"
        )
        print(
            f"[RGB HEADER DURATION] {rgb_duration_sec:.3f} s"
        )

        if estimated_fps is not None:
            print(
                f"[DEPTH HEADER ESTIMATED FPS] {estimated_fps:.3f}"
            )
        else:
            print(
                "[DEPTH HEADER ESTIMATED FPS] unavailable"
            )

        if rgb_estimated_fps is not None:
            print(
                f"[RGB HEADER ESTIMATED FPS] {rgb_estimated_fps:.3f}"
            )
        else:
            print(
                "[RGB HEADER ESTIMATED FPS] unavailable"
            )

        print(
            f"[RGB SYNC TOLERANCE] "
            f"{args.rgb_sync_tolerance_ms:.1f} ms"
        )
        print(
            f"[DISPLAY RANGE] "
            f"{args.near_mm:.0f}-{args.far_mm:.0f} mm"
        )
        print(
            f"[INITIAL SCALE MODE] {args.scale_mode}"
        )
        print(
            f"[COLORMAP] {args.colormap}"
        )

        run_playback(
            args=args,
            depth_frame_index=depth_frame_index,
            rgb_frame_index=rgb_frame_index,
            estimated_fps=estimated_fps,
        )

    except KeyboardInterrupt:
        print(
            "\n[INTERRUPTED] Ctrl+C",
            file=sys.stderr,
        )
        return 130

    except Exception as error:
        print(
            f"[ERROR] {type(error).__name__}: {error}",
            file=sys.stderr,
        )
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(
        main()
    )
