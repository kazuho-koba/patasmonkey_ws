#!/usr/bin/env python3

"""
特定のrosbag2（MCAP形式）を読み込み、
ホイールオドメトリとWitMotion IMUの初期方位をコンソールへ出力する診断スクリプト。

主な確認項目:
    1. /wheel/odometry の最初のpose.orientationが示すyaw
    2. ホイールオドメトリ開始時刻に最も近い /wit/imu のyaw
    3. ホイールオドメトリのx, y座標が最初に十分移動した方向
    4. wheel初期yawとIMU yawの角度差
    5. 現行の軌跡解析処理で用いている
       theta = IMU yaw - wheel初期yaw
       によって、初動方向が何度へ回転するか
    6. 各角度を次の2通りで解釈した方角
       - ROS ENU・数学角: 東=0°, 北=90°
       - 北基準方位角: 北=0°, 東=90°

このスクリプトはrosbag2_pyを使用せず、
mcap_ros2.reader.read_ros2_messages()でMCAPを直接読み取る。

必要パッケージ:
    python3 -m pip install mcap-ros2-support

実行例:
    python3 inspect_bag_wheel_imu_heading.py \
        /workspaces/patasmonkey_ws/bags/rosbag2_2026_07_26-09_17_38

移動方向の判定に使う最小変位を変更する例:
    python3 inspect_bag_wheel_imu_heading.py \
        /workspaces/patasmonkey_ws/bags/rosbag2_2026_07_26-09_17_38 \
        --movement-threshold 0.20

注意:
    - 使用する時刻はメッセージ内のheader.stampではなく、
      MCAPのbag記録時刻である。
    - 最初の2点だけで方向を求めると停止中の微小ノイズに影響されるため、
      初期位置から指定距離以上離れた最初の点を使用する。
    - このスクリプトはTFを参照しない。
      /wit/imu.orientationおよび/wheel/odometry.pose.pose.orientationの
      クォータニオンをそのままyawへ変換する。
"""

import argparse
import bisect
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from mcap_ros2.reader import read_ros2_messages


TOPIC_WHEEL = "/wheel/odometry"
TOPIC_IMU = "/wit/imu"


def parse_args() -> argparse.Namespace:
    """
    コマンドライン引数を定義して返す。

    位置引数:
        bag:
            metadata.yamlとMCAPファイルを含むrosbagディレクトリ、
            または単一のMCAPファイル。

    オプション:
        --movement-threshold:
            初動方向を計算するために必要な、初期位置からの最小変位[m]。
            停止中の微小な座標揺らぎを初動と誤認しないために使う。

        --max-imu-time-difference:
            wheel開始時刻とIMUサンプルの許容最大時間差[s]。
            最も近いIMUがこの値より遠い場合は警告する。
    """
    parser = argparse.ArgumentParser(
        description=(
            "Inspect initial wheel-odometry yaw, initial movement direction, "
            "and the nearest WitMotion IMU yaw in one MCAP rosbag."
        )
    )

    parser.add_argument(
        "bag",
        type=Path,
        help=(
            "Rosbag directory containing metadata.yaml and *.mcap, "
            "or a single .mcap file."
        ),
    )

    parser.add_argument(
        "--movement-threshold",
        type=float,
        default=0.20,
        help=(
            "Minimum displacement from the initial wheel position used to "
            "determine the initial movement direction [m]. Default: 0.20"
        ),
    )

    parser.add_argument(
        "--max-imu-time-difference",
        type=float,
        default=0.50,
        help=(
            "Maximum recommended time difference between wheel start and "
            "the nearest IMU sample [s]. Default: 0.50"
        ),
    )

    return parser.parse_args()


def natural_mcap_sort_key(path: Path) -> Tuple[str, int]:
    """
    分割MCAPファイルを末尾番号の数値順に並べるためのキーを返す。

    通常の文字列ソートでは、
        bag_10.mcap
    が
        bag_2.mcap
    より前に来る場合がある。

    末尾の「_数字」を整数として解釈し、
        _0, _1, _2, ..., _10
    の順に並べる。
    """
    stem = path.stem
    prefix, separator, suffix = stem.rpartition("_")

    if separator and suffix.isdigit():
        return prefix, int(suffix)

    return stem, -1


def find_mcap_files(bag_path: Path) -> List[Path]:
    """
    入力パスから解析対象のMCAPファイル一覧を取得する。

    入力が単一MCAPファイルの場合:
        そのファイルだけを返す。

    入力がrosbagディレクトリの場合:
        直下にあるすべての*.mcapを分割番号順に返す。

    Raises:
        FileNotFoundError:
            入力パスが存在しない場合。

        ValueError:
            入力形式がMCAPファイルでもディレクトリでもない場合、
            またはディレクトリ内にMCAPがない場合。
    """
    bag_path = bag_path.expanduser().resolve()

    if not bag_path.exists():
        raise FileNotFoundError(
            f"Input path does not exist: {bag_path}"
        )

    if bag_path.is_file():
        if bag_path.suffix.lower() != ".mcap":
            raise ValueError(
                f"Input file is not an MCAP file: {bag_path}"
            )

        return [bag_path]

    if not bag_path.is_dir():
        raise ValueError(
            f"Input path is neither a directory nor an MCAP file: {bag_path}"
        )

    mcap_files = sorted(
        bag_path.glob("*.mcap"),
        key=natural_mcap_sort_key,
    )

    if not mcap_files:
        raise ValueError(
            f"No MCAP files found in directory: {bag_path}"
        )

    return mcap_files


def quaternion_to_yaw(
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> float:
    """
    クォータニオンからZ軸回りのyawを計算する。

    戻り値はラジアンで、通常は[-pi, pi]の範囲になる。

    この関数はクォータニオンの数値をyawへ変換するだけで、
    次の事項は判定・補正しない。

        - yaw=0が東か北か
        - 時計回りか反時計回りか
        - IMU取付方向
        - imu_linkからbase_linkへのTF
    """
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)

    return math.atan2(
        siny_cosp,
        cosy_cosp,
    )


def normalize_angle_rad(angle: float) -> float:
    """
    角度を[-pi, pi)の範囲へ正規化する。

    例えば:
        3*pi   -> -pi
        -3*pi  -> -pi

    角度差を比較・表示する際に、360°をまたぐ不連続を減らす。
    """
    return (
        angle + math.pi
    ) % (
        2.0 * math.pi
    ) - math.pi


def radians_to_degrees(angle_rad: float) -> float:
    """
    ラジアンを度へ変換する。
    """
    return math.degrees(
        normalize_angle_rad(angle_rad)
    )


def compass_direction_from_enu_yaw(angle_rad: float) -> str:
    """
    ROS ENU・数学角としてyawを方角文字列へ変換する。

    前提:
        0°    = 東
        +90°  = 北
        ±180° = 西
        -90°  = 南

    8方位のうち最も近い方位を返す。
    """
    angle_deg = (
        math.degrees(angle_rad) + 360.0
    ) % 360.0

    directions = [
        "東",
        "北東",
        "北",
        "北西",
        "西",
        "南西",
        "南",
        "南東",
    ]

    index = int(
        (angle_deg + 22.5) // 45.0
    ) % 8

    return directions[index]


def compass_direction_from_north_zero(angle_rad: float) -> str:
    """
    北基準方位角としてyawを方角文字列へ変換する。

    前提:
        0°   = 北
        90°  = 東
        180° = 南
        270° = 西

    ここでは角度が時計回りに増える方位角として解釈する。

    注意:
        /wit/imuの実際の正方向が時計回りか反時計回りかは、
        このスクリプトだけでは判定できない。
        この表示は「北ゼロ・時計回り」と仮定した参考解釈である。
    """
    angle_deg = (
        math.degrees(angle_rad) + 360.0
    ) % 360.0

    directions = [
        "北",
        "北東",
        "東",
        "南東",
        "南",
        "南西",
        "西",
        "北西",
    ]

    index = int(
        (angle_deg + 22.5) // 45.0
    ) % 8

    return directions[index]


def format_angle(angle_rad: Optional[float]) -> str:
    """
    1つの角度を、ラジアン・度・2種類の方角解釈で整形する。

    angle_radがNoneの場合は、判定不能を示す文字列を返す。
    """
    if angle_rad is None:
        return "判定不能"

    normalized = normalize_angle_rad(angle_rad)
    degrees = math.degrees(normalized)

    return (
        f"{normalized:+.6f} rad / {degrees:+.2f} deg"
        f"\n      ROS ENU解釈       : "
        f"{compass_direction_from_enu_yaw(normalized)}"
        f"\n      北0°方位角解釈    : "
        f"{compass_direction_from_north_zero(normalized)}"
    )


def read_wheel_and_imu(
    mcap_files: List[Path],
) -> Tuple[List[Dict], List[Dict]]:
    """
    分割MCAP群から/wheel/odometryと/wit/imuだけを読み取る。

    不要な画像・点群等はtopics引数で除外し、
    対象2トピックだけをデコードする。

    Returns:
        wheel_samples:
            各要素が次を持つリスト。
                t
                x
                y
                yaw
                frame_id
                child_frame_id

        imu_samples:
            各要素が次を持つリスト。
                t
                yaw
                frame_id
    """
    wheel_samples: List[Dict] = []
    imu_samples: List[Dict] = []

    needed_topics = {
        TOPIC_WHEEL,
        TOPIC_IMU,
    }

    for mcap_path in mcap_files:
        print(f"[READ] {mcap_path}")

        for decoded in read_ros2_messages(
            mcap_path,
            topics=needed_topics,
            log_time_order=True,
        ):
            topic_name = decoded.channel.topic
            bag_timestamp = int(decoded.log_time_ns)
            msg = decoded.ros_msg

            if topic_name == TOPIC_WHEEL:
                position = msg.pose.pose.position
                orientation = msg.pose.pose.orientation

                yaw = quaternion_to_yaw(
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                )

                # headerやchild_frame_idが存在しない特殊メッセージでも
                # 診断自体を継続できるようgetattrを使用する。
                header = getattr(msg, "header", None)
                frame_id = getattr(
                    header,
                    "frame_id",
                    "",
                ) if header is not None else ""

                child_frame_id = getattr(
                    msg,
                    "child_frame_id",
                    "",
                )

                wheel_samples.append(
                    {
                        "t": bag_timestamp,
                        "x": float(position.x),
                        "y": float(position.y),
                        "yaw": float(yaw),
                        "frame_id": str(frame_id),
                        "child_frame_id": str(child_frame_id),
                    }
                )

            elif topic_name == TOPIC_IMU:
                orientation = msg.orientation

                yaw = quaternion_to_yaw(
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                )

                header = getattr(msg, "header", None)
                frame_id = getattr(
                    header,
                    "frame_id",
                    "",
                ) if header is not None else ""

                imu_samples.append(
                    {
                        "t": bag_timestamp,
                        "yaw": float(yaw),
                        "frame_id": str(frame_id),
                    }
                )

    # 分割MCAPの境界をまたいでも時系列順になるよう、
    # bag記録時刻で明示的に再ソートする。
    wheel_samples.sort(
        key=lambda item: item["t"]
    )
    imu_samples.sort(
        key=lambda item: item["t"]
    )

    return wheel_samples, imu_samples


def build_time_index(samples: List[Dict]) -> List[int]:
    """
    時刻付きサンプルから時刻だけの配列を作る。

    nearest_sample()でbisectによる二分探索を行うために使用する。
    """
    return [
        int(item["t"])
        for item in samples
    ]


def nearest_sample(
    samples: List[Dict],
    timestamps: List[int],
    query_t: int,
) -> Optional[Dict]:
    """
    指定時刻に最も近いサンプルを二分探索で取得する。

    検索時刻を時系列へ挿入した場合の位置をbisect_left()で求め、
    その直前と直後の2候補だけを比較する。

    samplesが空の場合はNoneを返す。
    """
    if not samples:
        return None

    index = bisect.bisect_left(
        timestamps,
        query_t,
    )

    if index <= 0:
        return samples[0]

    if index >= len(samples):
        return samples[-1]

    previous_item = samples[index - 1]
    next_item = samples[index]

    if (
        abs(previous_item["t"] - query_t)
        <= abs(next_item["t"] - query_t)
    ):
        return previous_item

    return next_item


def find_initial_movement(
    wheel_samples: List[Dict],
    movement_threshold: float,
) -> Optional[Dict]:
    """
    ホイールオドメトリが最初に十分移動した方向を求める。

    単純に最初の点と2点目を使うと、停止中のエンコーダノイズや
    浮動小数点の微小変化から無意味な方向が計算されることがある。

    そこで、最初の座標からの直線距離がmovement_threshold以上に
    なった最初の点を使用する。

    Returns:
        次の情報を持つ辞書。
            start:
                基準となる最初のwheelサンプル。

            end:
                閾値以上移動した最初のwheelサンプル。

            dx, dy:
                初期位置からの座標変化。

            distance:
                初期位置からの直線距離[m]。

            movement_yaw:
                atan2(dy, dx)で求めた、wheel座標上の初動方向[rad]。

            elapsed_sec:
                初期サンプルからの経過時間[s]。

        閾値以上移動した点がない場合はNone。
    """
    if not wheel_samples:
        return None

    start = wheel_samples[0]
    x0 = start["x"]
    y0 = start["y"]
    t0 = start["t"]

    for item in wheel_samples[1:]:
        dx = item["x"] - x0
        dy = item["y"] - y0
        distance = math.hypot(
            dx,
            dy,
        )

        if distance < movement_threshold:
            continue

        movement_yaw = math.atan2(
            dy,
            dx,
        )

        return {
            "start": start,
            "end": item,
            "dx": dx,
            "dy": dy,
            "distance": distance,
            "movement_yaw": movement_yaw,
            "elapsed_sec": (
                item["t"] - t0
            ) * 1.0e-9,
        }

    return None


def print_separator() -> None:
    """
    コンソール表示を区切る罫線を出力する。
    """
    print("=" * 78)


def print_report(
    bag_path: Path,
    wheel_samples: List[Dict],
    imu_samples: List[Dict],
    movement_threshold: float,
    max_imu_time_difference: float,
) -> None:
    """
    読み込んだwheel・IMUデータから診断結果を計算して表示する。

    主に比較する値:
        - wheel poseの初期yaw
        - 同時刻付近のIMU yaw
        - wheel x,yから求めた初動方向
        - theta = IMU yaw - wheel初期yaw
        - thetaで回転した後の初動方向

    最後の「整列後初動方向」は、軌跡プロットスクリプトの
    align_local_trajectory_to_north()が行っている回転を
    数値だけで再現したもの。
    """
    print_separator()
    print("ホイールオドメトリ・IMU 初期方位診断")
    print_separator()
    print(f"入力                 : {bag_path}")
    print(f"wheelサンプル数      : {len(wheel_samples)}")
    print(f"IMUサンプル数        : {len(imu_samples)}")
    print(f"初動判定距離         : {movement_threshold:.3f} m")

    if not wheel_samples:
        print("")
        print(f"[ERROR] {TOPIC_WHEEL} が見つかりません。")
        return

    wheel_start = wheel_samples[0]
    wheel_initial_yaw = wheel_start["yaw"]

    print("")
    print_separator()
    print("1. ホイールオドメトリの最初のメッセージ")
    print_separator()
    print(f"bag記録時刻          : {wheel_start['t']} ns")
    print(
        f"初期位置             : "
        f"x={wheel_start['x']:+.6f} m, "
        f"y={wheel_start['y']:+.6f} m"
    )
    print(f"header.frame_id      : {wheel_start['frame_id']!r}")
    print(f"child_frame_id       : {wheel_start['child_frame_id']!r}")
    print("pose初期yaw          :")
    print(f"      {format_angle(wheel_initial_yaw)}")

    imu_start = None
    imu_time_difference_sec = None

    if imu_samples:
        imu_timestamps = build_time_index(
            imu_samples
        )

        imu_start = nearest_sample(
            imu_samples,
            imu_timestamps,
            wheel_start["t"],
        )

    print("")
    print_separator()
    print("2. wheel開始時刻に最も近いIMU")
    print_separator()

    if imu_start is None:
        print(f"[WARN] {TOPIC_IMU} が見つかりません。")
    else:
        imu_time_difference_sec = (
            imu_start["t"] - wheel_start["t"]
        ) * 1.0e-9

        print(f"IMU bag記録時刻      : {imu_start['t']} ns")
        print(
            f"wheelとの差          : "
            f"{imu_time_difference_sec:+.6f} s"
        )
        print(f"IMU header.frame_id  : {imu_start['frame_id']!r}")
        print("IMU yaw              :")
        print(f"      {format_angle(imu_start['yaw'])}")

        if (
            abs(imu_time_difference_sec)
            > max_imu_time_difference
        ):
            print(
                f"[WARN] wheel開始時刻と最近傍IMUの差が"
                f"{max_imu_time_difference:.3f} sを超えています。"
            )

    movement = find_initial_movement(
        wheel_samples,
        movement_threshold,
    )

    print("")
    print_separator()
    print("3. wheelのx,y座標から求めた最初の移動方向")
    print_separator()

    if movement is None:
        print(
            "[WARN] 指定距離以上の移動が見つからないため、"
            "初動方向を判定できません。"
        )
    else:
        print(
            f"使用した変位         : "
            f"dx={movement['dx']:+.6f} m, "
            f"dy={movement['dy']:+.6f} m"
        )
        print(
            f"直線距離             : "
            f"{movement['distance']:.6f} m"
        )
        print(
            f"到達までの時間       : "
            f"{movement['elapsed_sec']:.6f} s"
        )
        print(
            f"到達点のbag記録時刻  : "
            f"{movement['end']['t']} ns"
        )
        print("初動方向 atan2(dy,dx):")
        print(
            f"      {format_angle(movement['movement_yaw'])}"
        )

    print("")
    print_separator()
    print("4. wheel初期yawとIMU yawの比較")
    print_separator()

    if imu_start is None:
        print(
            "[WARN] IMUがないため、角度差と解析時回転量を計算できません。"
        )
        return

    # 現在の軌跡解析スクリプトと同じ回転角。
    alignment_theta = normalize_angle_rad(
        imu_start["yaw"] - wheel_initial_yaw
    )

    print("解析時の回転量 theta = IMU yaw - wheel初期yaw:")
    print(f"      {format_angle(alignment_theta)}")

    if movement is not None:
        # 生の初動方向へ解析時回転量を加え、
        # 現行プロット処理後の初動方向を数値的に再現する。
        aligned_movement_yaw = normalize_angle_rad(
            movement["movement_yaw"] + alignment_theta
        )

        print("")
        print("解析スクリプトによる回転後の初動方向:")
        print(f"      {format_angle(aligned_movement_yaw)}")

        pose_vs_movement_difference = normalize_angle_rad(
            movement["movement_yaw"] - wheel_initial_yaw
        )

        print("")
        print("wheel pose初期yawと生の初動方向との差:")
        print(f"      {format_angle(pose_vs_movement_difference)}")

        print("")
        print("この差が0°付近なら、wheel内部ではorientationとx,y初動が整合。")
        print("±90°付近なら、wheelの姿勢と位置積分の軸規約が異なる可能性。")
        print("±180°付近なら、前進・後進または車体前方軸の反転も確認対象。")

    print("")
    print_separator()
    print("5. 解釈上の注意")
    print_separator()
    print(
        "ROS ENU・数学角では、0°=東、+90°=北、±180°=西、-90°=南です。"
    )
    print(
        "北基準方位角では、0°=北、90°=東、180°=南、270°=西です。"
    )
    print(
        "本スクリプトはTFを適用せず、各メッセージ内のクォータニオンを"
        "そのままyawへ変換しています。"
    )


def main() -> int:
    """
    スクリプト全体のエントリーポイント。

    処理:
        1. 引数を解析する。
        2. MCAPファイルを特定する。
        3. wheelとIMUの対象メッセージを読み取る。
        4. 初期yaw・初動方向・解析回転量を表示する。

    Returns:
        0:
            正常終了。

        1:
            bag読込みや解析中のエラー。

        2:
            引数値が不正。
    """
    args = parse_args()

    if args.movement_threshold <= 0.0:
        print(
            "--movement-threshold must be greater than zero.",
            file=sys.stderr,
        )
        return 2

    if args.max_imu_time_difference < 0.0:
        print(
            "--max-imu-time-difference must be zero or greater.",
            file=sys.stderr,
        )
        return 2

    try:
        mcap_files = find_mcap_files(
            args.bag
        )

        wheel_samples, imu_samples = read_wheel_and_imu(
            mcap_files
        )

        print_report(
            bag_path=args.bag.expanduser().resolve(),
            wheel_samples=wheel_samples,
            imu_samples=imu_samples,
            movement_threshold=args.movement_threshold,
            max_imu_time_difference=args.max_imu_time_difference,
        )

    except Exception as error:
        print(
            f"[ERROR] {type(error).__name__}: {error}",
            file=sys.stderr,
        )
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())