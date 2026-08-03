#!/usr/bin/env python3

"""
rosbag2（MCAP形式）に保存されたDepth画像を、DepthAI風の疑似カラーで再生する。

対象トピック:
    /oak/depth/image_raw

主な機能:
    1. rosbagディレクトリまたは単一MCAPファイルを直接読み込む
    2. sensor_msgs/msg/ImageのDepth画像をNumPy配列へ変換する
    3. DepthAIのdisparity表示に近い見え方として、
       距離の逆数（inverse depth）を0～255へ正規化する
    4. OpenCVのCOLORMAP_JETを適用し、
       近距離を赤、遠距離を青、無効値を黒で表示する
    5. bag記録時刻に基づき、元の時間間隔に近い速度で再生する
    6. 必要な場合だけ、色付き動画またはPNG連番を保存する
    7. GUIを表示せず、動画・画像保存だけを行うこともできる

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

注意:
    - 本スクリプトはrosbag2_pyを使用しない。
      mcap_ros2.reader.read_ros2_messages()でMCAPを直接読む。
    - DepthAI実機のdisparity値そのものはbagに保存されていないため、
      Depth画像からinverse depthを計算して見た目を近づけている。
      カラーマップは同等でも、実機disparity表示と値が完全一致するとは限らない。
    - 動画保存は固定FPSであるため、不規則な記録間隔は完全には再現できない。
      FPSはbag時刻から中央値で推定するか、--output-fpsで明示する。
"""

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.reader import read_ros2_messages


DEFAULT_TOPIC = "/oak/depth/image_raw"
DEFAULT_NEAR_MM = 300.0
DEFAULT_FAR_MM = 10000.0
DEFAULT_WINDOW_NAME = "OAK-D DepthAI-style depth playback"


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
            "Default: estimate from bag timestamps and apply --speed."
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


def collect_topic_timestamps(
    mcap_files: Sequence[Path],
    topic: str,
) -> List[int]:
    """
    MCAP低レベルreaderを使い、対象トピックの記録時刻だけを収集する。

    画像データをROSメッセージへデコードせずに時刻だけを調べるため、
    動画保存FPSの推定や処理範囲の表示に使用できる。

    Returns:
        log_timeのナノ秒値を昇順に並べたリスト。
    """
    timestamps: List[int] = []

    for mcap_path in mcap_files:
        with mcap_path.open("rb") as stream:
            reader = make_reader(stream)

            for _, channel, message in reader.iter_messages(
                topics=[topic]
            ):
                if channel.topic == topic:
                    timestamps.append(
                        int(message.log_time)
                    )

    timestamps.sort()
    return timestamps


def estimate_fps(timestamps: Sequence[int]) -> Optional[float]:
    """
    連続フレームの時刻差中央値から記録FPSを推定する。

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
            f"Frame: {frame_index}   "
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
            "s: snapshot   q/Esc: quit"
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


def iterate_depth_messages(
    mcap_files: Sequence[Path],
    topic: str,
) -> Iterable[Tuple[int, object]]:
    """
    分割MCAPを順番に読み、対象Depthトピックだけを返す。

    Yields:
        (bag記録時刻[ns], ROS Imageメッセージ)
    """
    for mcap_path in mcap_files:
        print(f"[READ] {mcap_path}")

        for decoded in read_ros2_messages(
            mcap_path,
            topics={topic},
            log_time_order=True,
        ):
            if decoded.channel.topic != topic:
                continue

            yield (
                int(decoded.log_time_ns),
                decoded.ros_msg,
            )


def calculate_wait_milliseconds(
    previous_timestamp_ns: Optional[int],
    current_timestamp_ns: int,
    speed: float,
) -> int:
    """
    直前フレームとのbag時刻差からOpenCV待機時間[ms]を計算する。

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

            None:
                通常再生を継続する。

        paused:
            更新後の一時停止状態。
    """
    while True:
        # 一時停止中は短い周期でキー入力を確認し続ける。
        # 再生中はbag時刻差に対応する時間だけ待機する。
        wait_ms = 30 if paused else delay_ms

        key = cv2.waitKey(
            wait_ms
        ) & 0xFF

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

            # 一時停止へ入った場合は同じフレーム上でキー待ちを続ける。
            continue

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
) -> Path:
    """
    キー操作で指定された現在フレームをPNGとして保存する。
    """
    snapshot_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    output_path = snapshot_dir / (
        f"depth_snapshot_"
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
    mcap_files: Sequence[Path],
    timestamps: Sequence[int],
    estimated_fps: Optional[float],
) -> None:
    """
    Depthメッセージを順次読み、色付け・表示・保存を実行する。

    保存動画のFPS:
        --output-fps指定時:
            指定値をそのまま使用。

        未指定時:
            bagから推定した記録FPSに--speedを掛ける。
            これにより保存動画の再生時間も速度設定へ概ね一致する。
    """
    video_path, frame_dir, snapshot_dir = prepare_output_paths(
        args
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

    first_topic_timestamp_ns = int(
        timestamps[0]
    )

    selected_start_ns = (
        first_topic_timestamp_ns
        + int(round(args.start_sec * 1.0e9))
    )

    selected_end_ns: Optional[int] = None

    if args.duration_sec is not None:
        selected_end_ns = (
            selected_start_ns
            + int(round(args.duration_sec * 1.0e9))
        )

    previous_processed_timestamp_ns: Optional[int] = None
    processed_count = 0
    paused = False
    quit_requested = False

    # 起動時は--scale-modeで指定された方式を使用し、
    # GUI表示中はd/lキーによってこの値だけを変更する。
    current_scale_mode = args.scale_mode

    try:
        if not args.no_display:
            cv2.namedWindow(
                DEFAULT_WINDOW_NAME,
                cv2.WINDOW_NORMAL,
            )

        for timestamp_ns, msg in iterate_depth_messages(
            mcap_files=mcap_files,
            topic=args.topic,
        ):
            if timestamp_ns < selected_start_ns:
                continue

            if (
                selected_end_ns is not None
                and timestamp_ns > selected_end_ns
            ):
                break

            if (
                args.max_frames is not None
                and processed_count >= args.max_frames
            ):
                break

            depth_mm = image_message_to_depth_mm(
                msg
            )

            # 同じDepthフレームをdisparity/linearで即座に再描画できるよう、
            # 表示・キー処理をフレーム内ループとして構成する。
            #
            # dまたはlが押された場合:
            #   current_scale_modeを変更し、同じdepth_mmから再色付けする。
            #
            # Spaceで一時停止した場合:
            #   フレーム番号を進めず、このループ内で操作を受け続ける。
            #
            # nが押された場合:
            #   一時停止状態を維持したまま、このループを抜けて次フレームへ進む。
            frame_finished = False
            step_requested = False
            display_frame: Optional[np.ndarray] = None
            valid_mask: Optional[np.ndarray] = None

            while not frame_finished:
                color_bgr, valid_mask = colorize_depth(
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
                    timestamp_ns - first_topic_timestamp_ns
                ) * 1.0e-9

                display_frame = color_bgr

                if not args.hide_colorbar:
                    display_frame = add_colorbar(
                        image_bgr=display_frame,
                        near_mm=args.near_mm,
                        far_mm=args.far_mm,
                        scale_mode=current_scale_mode,
                        colormap_name=args.colormap,
                    )

                if not args.hide_overlay:
                    display_frame = draw_overlay(
                        image_bgr=display_frame,
                        frame_index=processed_count,
                        relative_time_sec=relative_time_sec,
                        encoding=str(msg.encoding),
                        near_mm=args.near_mm,
                        far_mm=args.far_mm,
                        scale_mode=current_scale_mode,
                        valid_ratio=valid_ratio,
                        paused=paused,
                    )

                if args.no_display:
                    # GUIなしの場合は途中でキー切替できないため、
                    # 起動時の--scale-modeを全フレームへ適用する。
                    frame_finished = True
                    break

                cv2.imshow(
                    DEFAULT_WINDOW_NAME,
                    display_frame,
                )

                delay_ms = calculate_wait_milliseconds(
                    previous_timestamp_ns=previous_processed_timestamp_ns,
                    current_timestamp_ns=timestamp_ns,
                    speed=args.speed,
                )

                command, paused = wait_for_playback_key(
                    delay_ms=delay_ms,
                    paused=paused,
                )

                if command == "quit":
                    quit_requested = True
                    frame_finished = True
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

                    # 同一Depthフレームを新しい方式で再色付けして表示する。
                    # 一時停止中なら静止画比較になり、再生中でも即時反映される。
                    continue

                if command == "snapshot":
                    snapshot_path = save_snapshot(
                        frame_bgr=display_frame,
                        snapshot_dir=snapshot_dir,
                        frame_index=processed_count,
                        timestamp_ns=timestamp_ns,
                    )
                    print(
                        f"[SNAPSHOT] {snapshot_path}"
                    )

                    # snapshot後も同じフレームを表示し続け、
                    # 特に一時停止中の比較操作を中断しない。
                    continue

                if command == "step":
                    step_requested = True
                    frame_finished = True
                    break

                # commandがNoneなら通常再生。
                # 一時停止解除時もここへ到達し、次フレームへ進む。
                frame_finished = True

            if quit_requested:
                break

            if display_frame is None:
                raise RuntimeError(
                    "Display frame was not generated."
                )

            # 動画・連番画像には、ユーザーがそのフレームで最後に選択した
            # 表示モードを反映する。
            if video_path is not None:
                if writer is None:
                    frame_height, frame_width = display_frame.shape[:2]

                    writer = create_video_writer(
                        output_path=video_path,
                        frame_size=(
                            frame_width,
                            frame_height,
                        ),
                        fps=video_fps,
                    )

                    print(
                        f"[SAVE VIDEO] {video_path}"
                    )
                    print(
                        f"[VIDEO FPS] {video_fps:.3f}"
                    )

                writer.write(
                    display_frame
                )

            if frame_dir is not None:
                frame_path = frame_dir / (
                    f"depth_{processed_count:06d}_"
                    f"{timestamp_ns}.png"
                )

                if not cv2.imwrite(
                    str(frame_path),
                    display_frame,
                ):
                    raise RuntimeError(
                        f"Failed to write PNG: {frame_path}"
                    )

            previous_processed_timestamp_ns = timestamp_ns
            processed_count += 1

            if processed_count % 100 == 0:
                print(
                    f"[PROGRESS] {processed_count} frames"
                )

    finally:
        if writer is not None:
            writer.release()

        if not args.no_display:
            cv2.destroyAllWindows()

    print("")
    print("=" * 72)
    print("Depth playback finished")
    print("=" * 72)
    print(f"Processed frames : {processed_count}")
    print(f"Quit requested   : {quit_requested}")
    print(f"Final scale mode : {current_scale_mode}")

    if video_path is not None:
        print(f"Saved video      : {video_path}")

    if frame_dir is not None:
        print(f"Saved PNG dir    : {frame_dir}")


def main() -> int:
    """
    スクリプトのエントリーポイント。

    処理順:
        1. 引数を検証する
        2. MCAPファイル一覧を取得する
        3. 対象トピックの時刻とFPSを確認する
        4. Depth画像を色付けして再生・保存する
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
            f"[TOPIC] {args.topic}"
        )

        timestamps = collect_topic_timestamps(
            mcap_files=mcap_files,
            topic=args.topic,
        )

        if not timestamps:
            raise RuntimeError(
                f"No messages found on topic: {args.topic}"
            )

        estimated_fps = estimate_fps(
            timestamps
        )

        duration_sec = (
            timestamps[-1] - timestamps[0]
        ) * 1.0e-9

        print(
            f"[MESSAGES] {len(timestamps)}"
        )
        print(
            f"[DURATION] {duration_sec:.3f} s"
        )

        if estimated_fps is not None:
            print(
                f"[ESTIMATED FPS] {estimated_fps:.3f}"
            )
        else:
            print(
                "[ESTIMATED FPS] unavailable"
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
            mcap_files=mcap_files,
            timestamps=timestamps,
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
