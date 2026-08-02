#!/usr/bin/env python3

"""
rosbag2（MCAP形式）に記録されたGNSS・各種オドメトリ・EKF出力を読み込み、
走行軌跡を地図背景付きで可視化するスクリプト。

主な処理:
    1. 指定されたbagsルート以下のmetadata.yamlを再帰的に探索する。
    2. 各bagについて、次の情報を読み取る。
       - GNSS緯度・経度
       - RTK FIX / FLOATなどの測位状態
       - IMU orientationから求めたyaw
       - ホイールオドメトリ
       - ビジュアルオドメトリ
       - EKF local
       - EKF global
    3. GNSSを局所的な東・北方向のメートル座標へ変換する。
    4. ローカル座標系の軌跡を、開始時のIMU yawとGNSS位置に基づいて整列する。
    5. GNSS基準の共通表示範囲で6枚、全軌跡を含む表示範囲で1枚を保存する。
    6. 解析完了マーカーを保存し、次回以降は解析済みbagをスキップする。

注意:
    - GNSS座標変換にはWeb Mercator（EPSG:3857）を使用する。
    - OpenStreetMap背景はcontextilyが利用可能で、かつネットワーク接続が
      ある場合のみ描画される。
    - ローカル軌跡の整列は初期位置・初期yawを合わせる処理であり、
      途中のドリフトやスケール誤差そのものは補正しない。
"""

import argparse
import bisect
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

import matplotlib

# GUIを持たないDocker・サーバ環境でもPNGを生成できるよう、
# 対話表示を必要としないAggバックエンドを明示的に使用する。
matplotlib.use("Agg")

import matplotlib.pyplot as plt

from pyproj import Transformer

from mcap_ros2.reader import read_ros2_messages


# contextilyはOpenStreetMap等のタイル背景を取得するための任意依存。
# 未インストール、またはimport時に問題が発生しても、
# 軌跡そのものの描画は継続できるよう例外を捕捉する。
try:
    import contextily as ctx

    HAS_CONTEXTILY = True
except Exception:
    HAS_CONTEXTILY = False


# ---------------------------------------------------------------------------
# 解析対象トピック
# ---------------------------------------------------------------------------

# GNSS位置と方位基準用IMU
TOPIC_FIX = "/fix"
TOPIC_WIT_IMU = "/wit/imu"

# RTK状態を取得する候補トピック
TOPIC_NAVRELPOSNED = "/navrelposned"
TOPIC_NAVPVT = "/navpvt"
TOPIC_NAVSTATUS = "/navstatus"

# 比較対象となるオドメトリ・自己位置推定トピック
TOPIC_WHEEL = "/wheel/odometry"
TOPIC_VIO = "/vio/odometry"
TOPIC_EKF_LOCAL = "/odometry/local"
TOPIC_EKF_GLOBAL = "/odometry/global"


# ---------------------------------------------------------------------------
# 出力ファイル
# ---------------------------------------------------------------------------

OUTPUT_FILES = {
    "gnss": "trajectory_01_gnss.png",
    "wheel": "trajectory_02_wheel_odometry.png",
    "vio": "trajectory_03_visual_odometry.png",
    "ekf_local": "trajectory_04_ekf_local.png",
    "ekf_global": "trajectory_05_ekf_global.png",
    "overlay": "trajectory_06_overlay_all.png",

    # GNSS基準範囲から大きく外れた軌跡も含めて確認するための
    # ズームアウト版重ね合わせ画像。
    "overlay_full": "trajectory_07_overlay_full_extent.png",
}


# ---------------------------------------------------------------------------
# 描画スタイル
# ---------------------------------------------------------------------------

# GNSS点はRTK状態に応じて色・マーカーを変える。
GNSS_STATE_STYLE = {
    "RTK_FIX": {
        "color": "limegreen",
        "marker": "o",
        "label": "RTK FIX",
    },
    "RTK_FLOAT": {
        "color": "dodgerblue",
        "marker": "s",
        "label": "RTK FLOAT",
    },
    "GNSS": {
        "color": "orange",
        "marker": "^",
        "label": "GNSS",
    },
    "NO_FIX": {
        "color": "red",
        "marker": "x",
        "label": "NO FIX",
    },
    "UNKNOWN": {
        "color": "gray",
        "marker": ".",
        "label": "UNKNOWN",
    },
}

# 各軌跡の線色・線幅・透明度・凡例名。
LINE_STYLE = {
    "gnss": {
        "color": "black",
        "linewidth": 1.0,
        "alpha": 0.5,
        "label": "GNSS path",
    },
    "wheel": {
        "color": "tab:blue",
        "linewidth": 1.5,
        "alpha": 0.9,
        "label": "Wheel odometry",
    },
    "vio": {
        "color": "tab:orange",
        "linewidth": 1.5,
        "alpha": 0.9,
        "label": "Visual odometry",
    },
    "ekf_local": {
        "color": "tab:green",
        "linewidth": 1.5,
        "alpha": 0.9,
        "label": "EKF local",
    },
    "ekf_global": {
        "color": "tab:red",
        "linewidth": 1.5,
        "alpha": 0.9,
        "label": "EKF global",
    },
}


# ---------------------------------------------------------------------------
# 解析完了マーカー
# ---------------------------------------------------------------------------

# PNGが一部だけ生成された状態を「解析完了」と誤認しないよう、
# すべての処理が正常終了した後に専用JSONを生成する。
ANALYSIS_COMPLETE_FILENAME = "trajectory_analysis_complete.json"

# 解析ロジックや出力仕様を変更した際に識別できるようにするバージョン。
ANALYSIS_VERSION = 2


def analysis_complete_marker_path(bag_dir: Path) -> Path:
    """
    指定bagディレクトリに対応する解析完了マーカーのパスを返す。

    Args:
        bag_dir:
            metadata.yamlとMCAPファイルが存在するrosbagディレクトリ。

    Returns:
        bagディレクトリ直下のtrajectory_analysis_complete.jsonのパス。
    """
    return bag_dir / ANALYSIS_COMPLETE_FILENAME


def should_skip_bag(bag_dir: Path, overwrite: bool) -> bool:
    """
    指定bagを解析対象から除外するか判定する。

    --overwriteが指定されている場合は、完了マーカーの有無にかかわらず
    再解析する。

    --overwriteが指定されていない場合は、完了マーカーが存在するbagを
    解析済みとみなしてスキップする。

    PNGファイルの存在だけで判定しない理由:
        解析途中で例外終了した場合、一部のPNGだけが残る可能性がある。
        完了マーカーは全画像保存後に生成されるため、部分的な処理結果を
        完了済みと誤認しにくい。

    Args:
        bag_dir:
            判定対象のrosbagディレクトリ。

        overwrite:
            Trueなら既存解析結果を無視して再解析する。

    Returns:
        スキップする場合はTrue、解析する場合はFalse。
    """
    if overwrite:
        return False

    return analysis_complete_marker_path(bag_dir).exists()


def parse_args() -> argparse.Namespace:
    """
    コマンドライン引数を定義し、解析済みNamespaceを返す。

    位置引数:
        bags_root:
            複数のrosbagディレクトリを含むルートディレクトリ。

    オプション:
        --overwrite:
            解析完了マーカーが存在するbagも再解析し、出力を上書きする。
    """
    parser = argparse.ArgumentParser(
        description=(
            "Generate trajectory plots for every rosbag2 directory "
            "under a bags root."
        )
    )

    parser.add_argument(
        "bags_root",
        type=Path,
        help="Root directory containing rosbag2 bag directories.",
    )

    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing plot files.",
    )

    return parser.parse_args()


def quaternion_to_yaw(
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> float:
    """
    クォータニオンからZ軸回りの回転角yawを計算する。

    ROSのgeometry_msgs/Quaternionはx, y, z, wの順で値を持つ。
    ここではroll・pitchを直接使用せず、平面上の軌跡整列に必要な
    yawだけを取り出す。

    Args:
        qx, qy, qz, qw:
            クォータニオン成分。

    Returns:
        yaw角。単位はラジアンで、atan2の戻り値範囲に従い
        おおむね[-pi, pi]となる。
    """
    # クォータニオンからyawを求める標準的な変換式。
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)

    return math.atan2(siny_cosp, cosy_cosp)


def finite_number(x) -> bool:
    """
    値がNone、NaN、正負の無限大ではなく、有限な数値か確認する。

    GNSSやオドメトリに不正値が含まれていた場合に、
    座標範囲計算や描画全体がNaNになることを防ぐ。

    Args:
        x:
            数値として評価したい値。

    Returns:
        有限な数値ならTrue、それ以外はFalse。
    """
    return x is not None and math.isfinite(float(x))

def natural_mcap_sort_key(path: Path):
    """
    分割MCAPファイルを末尾番号の数値順に並べる。

    文字列順では_10が_2より前に来るため、
    末尾の数字を整数として解釈する。
    """
    stem = path.stem
    prefix, separator, suffix = stem.rpartition("_")

    if separator and suffix.isdigit():
        return prefix, int(suffix)

    return stem, -1


def find_mcap_files(bag_dir: Path):
    """
    rosbagディレクトリ直下の全MCAPファイルを取得する。
    """
    return sorted(
        bag_dir.glob("*.mcap"),
        key=natural_mcap_sort_key,
    )


def infer_rtk_state_from_msg(msg) -> str:
    """
    u-bloxメッセージの整数ビットフィールドから測位状態を判定する。

    対応対象:
        - NAV-RELPOSNED
        - NAV-PVT
        - NAV-STATUS

    Returns:
        "RTK_FIX"
        "RTK_FLOAT"
        "GNSS"
        "NO_FIX"
        "UNKNOWN"
    """

    # --------------------------------------------------------------
    # NAV-RELPOSNED
    #
    # このメッセージは、次のような固有フィールドを持つ。
    #   rel_pos_n
    #   rel_pos_e
    #   rel_pos_d
    #   rel_pos_heading
    #
    # flagsのビット3～4がcarrSoln。
    # --------------------------------------------------------------
    if (
        hasattr(msg, "rel_pos_n")
        and hasattr(msg, "rel_pos_e")
        and hasattr(msg, "flags")
    ):
        flags = int(msg.flags)

        gnss_fix_ok = bool(flags & 0x01)
        carr_soln = (flags & 0x18) >> 3

        if not gnss_fix_ok:
            return "NO_FIX"

        if carr_soln == 2:
            return "RTK_FIX"

        if carr_soln == 1:
            return "RTK_FLOAT"

        return "GNSS"

    # --------------------------------------------------------------
    # NAV-PVT
    #
    # NAV-PVTはfix_type、num_sv、lon、latなどを持つ。
    # flagsのビット6～7がcarrSoln。
    # --------------------------------------------------------------
    if (
        hasattr(msg, "fix_type")
        and hasattr(msg, "num_sv")
        and hasattr(msg, "flags")
    ):
        flags = int(msg.flags)
        fix_type = int(msg.fix_type)

        gnss_fix_ok = bool(flags & 0x01)
        carr_soln = (flags & 0xC0) >> 6

        if not gnss_fix_ok or fix_type == 0:
            return "NO_FIX"

        if carr_soln == 2:
            return "RTK_FIX"

        if carr_soln == 1:
            return "RTK_FLOAT"

        return "GNSS"

    # --------------------------------------------------------------
    # NAV-STATUS
    #
    # NAV-STATUSにはRTK FIX/FLOATを直接判定できる
    # carrSolnがないため、有効な通常GNSS解かどうかだけを判定する。
    # --------------------------------------------------------------
    if (
        hasattr(msg, "gps_fix")
        and hasattr(msg, "flags")
    ):
        flags = int(msg.flags)
        gps_fix = int(msg.gps_fix)

        gnss_fix_ok = bool(flags & 0x01)

        if not gnss_fix_ok or gps_fix == 0:
            return "NO_FIX"

        if gps_fix in (2, 3, 4):
            return "GNSS"

        return "UNKNOWN"

    # --------------------------------------------------------------
    # 他ドライバでcarr_solnが独立フィールドになっている場合
    # --------------------------------------------------------------
    if hasattr(msg, "carr_soln"):
        carr_soln = int(msg.carr_soln)

        if carr_soln == 2:
            return "RTK_FIX"

        if carr_soln == 1:
            return "RTK_FLOAT"

        if carr_soln == 0:
            return "GNSS"

    return "UNKNOWN"


def read_bag_data(bag_dir: Path) -> dict:
    """
    1つのrosbagから軌跡描画に必要なデータだけを抽出する。

    rosbag2_pyは使用せず、mcap_ros2.reader.read_ros2_messages()で
    分割MCAPを直接読み取る。

    MCAP内に埋め込まれたROS 2メッセージ定義を使って、
    CDRデータをPythonオブジェクトへデコードする。
    """
    needed_topics = {
        TOPIC_FIX,
        TOPIC_WIT_IMU,
        TOPIC_NAVRELPOSNED,
        TOPIC_NAVPVT,
        TOPIC_NAVSTATUS,
        TOPIC_WHEEL,
        TOPIC_VIO,
        TOPIC_EKF_LOCAL,
        TOPIC_EKF_GLOBAL,
    }

    fixes = []
    imu_yaws = []

    navrelposned_states = []
    navpvt_states = []
    navstatus_states = []

    odom_data = {
        "wheel": [],
        "vio": [],
        "ekf_local": [],
        "ekf_global": [],
    }

    topic_to_key = {
        TOPIC_WHEEL: "wheel",
        TOPIC_VIO: "vio",
        TOPIC_EKF_LOCAL: "ekf_local",
        TOPIC_EKF_GLOBAL: "ekf_global",
    }

    mcap_files = find_mcap_files(bag_dir)

    if not mcap_files:
        raise FileNotFoundError(
            f"No MCAP files found in: {bag_dir}"
        )

    for mcap_path in mcap_files:
        print(f"      Reading: {mcap_path.name}")

        # topicsを指定することで、画像・点群など不要なメッセージを
        # デコード対象から除外する。
        for decoded in read_ros2_messages(
            mcap_path,
            topics=needed_topics,
            log_time_order=True,
        ):
            topic_name = decoded.channel.topic
            bag_timestamp = int(decoded.log_time_ns)
            msg = decoded.ros_msg

            if topic_name == TOPIC_FIX:
                lat = getattr(msg, "latitude", None)
                lon = getattr(msg, "longitude", None)

                if finite_number(lat) and finite_number(lon):
                    fixes.append(
                        {
                            "t": bag_timestamp,
                            "lat": float(lat),
                            "lon": float(lon),
                        }
                    )

            elif topic_name == TOPIC_WIT_IMU:
                orientation = msg.orientation

                yaw = quaternion_to_yaw(
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                )

                imu_yaws.append(
                    {
                        "t": bag_timestamp,
                        "yaw": float(yaw),
                    }
                )

            elif topic_name == TOPIC_NAVRELPOSNED:
                navrelposned_states.append(
                    {
                        "t": bag_timestamp,
                        "state": infer_rtk_state_from_msg(msg),
                    }
                )

            elif topic_name == TOPIC_NAVPVT:
                navpvt_states.append(
                    {
                        "t": bag_timestamp,
                        "state": infer_rtk_state_from_msg(msg),
                    }
                )

            elif topic_name == TOPIC_NAVSTATUS:
                navstatus_states.append(
                    {
                        "t": bag_timestamp,
                        "state": infer_rtk_state_from_msg(msg),
                    }
                )

            elif topic_name in topic_to_key:
                position = msg.pose.pose.position
                orientation = msg.pose.pose.orientation

                yaw = quaternion_to_yaw(
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                )

                odom_data[topic_to_key[topic_name]].append(
                    {
                        "t": bag_timestamp,
                        "x": float(position.x),
                        "y": float(position.y),
                        "yaw": float(yaw),
                    }
                )

    # 分割MCAPごとに読むため、念のため全系列を時刻順へ並べ直す。
    fixes.sort(key=lambda item: item["t"])
    imu_yaws.sort(key=lambda item: item["t"])
    navrelposned_states.sort(key=lambda item: item["t"])
    navpvt_states.sort(key=lambda item: item["t"])
    navstatus_states.sort(key=lambda item: item["t"])

    for series in odom_data.values():
        series.sort(key=lambda item: item["t"])

    return {
        "fixes": fixes,
        "imu_yaws": imu_yaws,
        "navrelposned_states": navrelposned_states,
        "navpvt_states": navpvt_states,
        "navstatus_states": navstatus_states,
        "odom_data": odom_data,
    }


def choose_rtk_status_stream(data: dict) -> list:
    """
    利用可能なRTK状態系列から、最も情報量が多い候補を1つ選ぶ。

    優先順位:
        1. navrelposned
        2. navpvt
        3. navstatus

    navrelposnedを最優先にする理由:
        搬送波解の状態を直接持つ可能性が高く、
        RTK FIX / FLOATを区別しやすいため。

    Args:
        data:
            read_bag_data()が返した辞書。

    Returns:
        選択した時刻付き状態系列。
        いずれも存在しない場合は空リスト。
    """
    if data["navrelposned_states"]:
        return data["navrelposned_states"]

    if data["navpvt_states"]:
        return data["navpvt_states"]

    if data["navstatus_states"]:
        return data["navstatus_states"]

    return []


def build_time_index(samples: list) -> list:
    """
    時刻付きサンプル列から時刻だけのリストを作る。

    nearest_sample()ではbisectによる二分探索を行うため、
    辞書リストとは別に時刻だけの配列を用意する。

    前提:
        samplesはbagから読み出した時系列順に並んでいる。

    Args:
        samples:
            各要素が少なくとも"t"キーを持つリスト。

    Returns:
        ナノ秒時刻のリスト。
    """
    return [
        item["t"]
        for item in samples
    ]


def nearest_sample(
    samples: list,
    timestamps: list,
    query_t: int,
):
    """
    指定時刻に最も近いサンプルを二分探索で取得する。

    線形探索ではGNSS点ごとに全RTK状態を確認することになり、
    データ量が増えると計算量が大きくなる。
    bisect_left()を使用することで、挿入位置の前後2点だけを比較する。

    Args:
        samples:
            時系列順に並んだサンプル辞書のリスト。

        timestamps:
            samplesと同じ順序で抽出した時刻リスト。

        query_t:
            検索対象時刻。単位はナノ秒。

    Returns:
        最も時刻差が小さいサンプル。
        samplesが空の場合はNone。
    """
    if not samples:
        return None

    # query_tを挿入して時刻順を保てる位置を得る。
    idx = bisect.bisect_left(
        timestamps,
        query_t,
    )

    # 検索時刻が全サンプルより前なら最初のサンプルを返す。
    if idx <= 0:
        return samples[0]

    # 検索時刻が全サンプルより後なら最後のサンプルを返す。
    if idx >= len(samples):
        return samples[-1]

    # 挿入位置の直前・直後の時刻差を比較する。
    prev_item = samples[idx - 1]
    next_item = samples[idx]

    if (
        abs(prev_item["t"] - query_t)
        <= abs(next_item["t"] - query_t)
    ):
        return prev_item

    return next_item


def convert_fixes_to_local_xy(
    fixes: list,
    rtk_states: list,
):
    """
    GNSS緯度・経度を局所的な東・北方向のメートル座標へ変換する。

    処理:
        1. WGS84緯度経度（EPSG:4326）をWeb Mercator（EPSG:3857）へ変換。
        2. 最初のGNSS点を原点として差分を取る。
        3. 各GNSS点に最も時刻が近いRTK状態を割り当てる。

    出力座標:
        x:
            おおむね東方向。単位はメートル。

        y:
            おおむね北方向。単位はメートル。

    注意:
        Web Mercatorでは緯度による縮尺歪みがある。
        小規模な走行領域の可視化には扱いやすいが、
        厳密な測量距離評価にはUTMや局所ENU座標の方が適する。

    Args:
        fixes:
            {"t", "lat", "lon"}を持つGNSS点のリスト。

        rtk_states:
            {"t", "state"}を持つRTK状態系列。

    Returns:
        tuple:
            gnss_points:
                局所x, y座標とRTK状態を持つGNSS点。

            mercator_origin:
                背景地図の座標シフトに使う最初のGNSS点の
                Web Mercator絶対座標。
                GNSS点がない場合はNone。
    """
    if not fixes:
        return [], None

    # always_xy=Trueにより、入力順をlon, latに固定する。
    transformer = Transformer.from_crs(
        "EPSG:4326",
        "EPSG:3857",
        always_xy=True,
    )

    # 最初のGNSS点を局所座標原点とする。
    first_fix = fixes[0]

    origin_mx, origin_my = transformer.transform(
        first_fix["lon"],
        first_fix["lat"],
    )

    # RTK状態との時刻対応付けを高速化するため時刻配列を作る。
    state_timestamps = build_time_index(rtk_states)

    gnss_points = []

    for item in fixes:
        # 経度・緯度からWeb Mercator絶対座標へ変換する。
        mx, my = transformer.transform(
            item["lon"],
            item["lat"],
        )

        # 最初のGNSS位置との差分を取り、局所座標へ変換する。
        local_x = mx - origin_mx
        local_y = my - origin_my

        # GNSS位置メッセージとRTK状態メッセージは完全に同時とは限らないため、
        # 最も近い時刻の状態を割り当てる。
        state_item = nearest_sample(
            rtk_states,
            state_timestamps,
            item["t"],
        )

        if state_item is None:
            state = "UNKNOWN"
        else:
            state = state_item["state"]

        gnss_points.append(
            {
                "t": item["t"],
                "x": local_x,
                "y": local_y,
                "state": state,

                # 後から元の緯度・経度も参照できるよう保持する。
                "lat": item["lat"],
                "lon": item["lon"],
            }
        )

    return gnss_points, {
        "origin_mx": origin_mx,
        "origin_my": origin_my,
    }


def align_local_trajectory_to_north(
    samples: list,
    imu_yaws: list,
    gnss_points: list,
) -> list:
    """
    ローカル座標系のオドメトリ軌跡をGNSS・北基準へ初期整列する。

    対象:
        - ホイールオドメトリ
        - VIO
        - EKF local
        - EKF global

    処理:
        1. 軌跡の最初の位置(x0, y0)をローカル原点として差し引く。
        2. 軌跡開始時のorientation yaw0を取得する。
        3. 軌跡開始時刻に最も近いIMU yawを取得する。
        4. theta = IMU yaw - 軌跡初期yaw だけ全軌跡を回転する。
        5. 軌跡開始時刻に最も近いGNSS位置へ平行移動する。

    この処理により:
        - 図の上方向を北として扱いやすくなる。
        - VO等の初期向きが実車・地理座標と異なる場合でも、
          初期姿勢を揃えて比較できる。
        - 各軌跡の開始位置をGNSS位置に合わせられる。

    補正しないもの:
        - 途中の方位ドリフト
        - 距離スケール誤差
        - GNSSとオドメトリ間の時間遅延
        - IMU磁気方位のオフセットや磁気外乱

    Args:
        samples:
            {"t", "x", "y", "yaw"}を持つオドメトリ点列。

        imu_yaws:
            {"t", "yaw"}を持つIMU yaw系列。

        gnss_points:
            {"t", "x", "y"}を持つ局所GNSS点列。

    Returns:
        GNSS局所座標系へ初期整列した{"t", "x", "y"}のリスト。
    """
    if not samples:
        return []

    # 軌跡自身の開始状態。
    start_sample = samples[0]

    t0 = start_sample["t"]
    x0 = start_sample["x"]
    y0 = start_sample["y"]
    yaw0 = start_sample["yaw"]

    # 開始時刻に最も近いIMU yawとGNSS位置を検索するための時刻配列。
    imu_timestamps = build_time_index(imu_yaws)
    gnss_timestamps = build_time_index(gnss_points)

    imu_at_start = nearest_sample(
        imu_yaws,
        imu_timestamps,
        t0,
    )

    gnss_at_start = nearest_sample(
        gnss_points,
        gnss_timestamps,
        t0,
    )

    # IMUが存在しないbagでは回転補正を0 radとする。
    # その場合、軌跡初期yawとの差により-theta0相当の回転になる。
    if imu_at_start is None:
        imu_yaw = 0.0
    else:
        imu_yaw = imu_at_start["yaw"]

    # GNSSが存在しないbagでは原点(0, 0)へ配置する。
    if gnss_at_start is None:
        anchor_x = 0.0
        anchor_y = 0.0
    else:
        anchor_x = gnss_at_start["x"]
        anchor_y = gnss_at_start["y"]

    # 軌跡初期yawをIMU方位へ一致させるための回転角。
    theta = imu_yaw - yaw0

    # 全点で同じ三角関数を使うため、ループ外で一度だけ計算する。
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    aligned = []

    for item in samples:
        # 軌跡開始位置をローカル原点へ移す。
        dx = item["x"] - x0
        dy = item["y"] - y0

        # 2次元回転行列:
        #
        # [rx] = [ cos(theta) -sin(theta)] [dx]
        # [ry]   [ sin(theta)  cos(theta)] [dy]
        rx = cos_t * dx - sin_t * dy
        ry = sin_t * dx + cos_t * dy

        # 回転後の軌跡を、開始時刻のGNSS位置へ平行移動する。
        aligned.append(
            {
                "t": item["t"],
                "x": anchor_x + rx,
                "y": anchor_y + ry,
            }
        )

    return aligned


def compute_square_bounds_from_series(
    series_list: list,
    minimum_half_span: float = 10.0,
    margin_ratio: float = 0.10,
) -> dict:
    """
    複数の軌跡系列から、正方形の描画範囲を計算する。

    x方向とy方向で同じ表示幅を使うことで、
    距離・曲率・方位が図上で歪まないようにする。

    Args:
        series_list:
            軌跡系列のリスト。
            各点は少なくとも"x"と"y"を持つ。

        minimum_half_span:
            描画範囲の最小半幅。単位はメートル。
            15.0なら最低でも幅30 m、高さ30 mとなる。

        margin_ratio:
            データ最大幅に対して上下左右へ追加する余白率。

    Returns:
        次のキーを持つ辞書:
            xmin
            xmax
            ymin
            ymax
    """
    xs = []
    ys = []

    for series in series_list:
        for item in series:
            x = item.get("x")
            y = item.get("y")

            # NaNやinfが1点でも入るとmin/maxが壊れるため除外する。
            if not finite_number(x) or not finite_number(y):
                continue

            xs.append(float(x))
            ys.append(float(y))

    # 有効な座標がない場合は原点中心の既定範囲を返す。
    if not xs or not ys:
        return {
            "xmin": -minimum_half_span,
            "xmax": minimum_half_span,
            "ymin": -minimum_half_span,
            "ymax": minimum_half_span,
        }

    xmin = min(xs)
    xmax = max(xs)
    ymin = min(ys)
    ymax = max(ys)

    # データ全体の中心。
    center_x = 0.5 * (xmin + xmax)
    center_y = 0.5 * (ymin + ymax)

    # x幅とy幅の大きい方を正方形の基準幅にする。
    data_span = max(
        xmax - xmin,
        ymax - ymin,
    )

    # 余白込み半幅と最低半幅の大きい方を採用する。
    half_span = max(
        minimum_half_span,
        0.5 * data_span * (1.0 + 2.0 * margin_ratio),
    )

    return {
        "xmin": center_x - half_span,
        "xmax": center_x + half_span,
        "ymin": center_y - half_span,
        "ymax": center_y + half_span,
    }


def compute_gnss_reference_bounds(
    gnss_points: list,
) -> dict:
    """
    標準6図で共通使用する、GNSS基準の描画範囲を計算する。

    GNSS点だけを範囲計算に使用するため、
    wheel・VIO・EKFのいずれかが発散しても標準図が極端に縮小されない。

    Returns:
        GNSS軌跡を含む正方形描画範囲。
    """
    return compute_square_bounds_from_series(
        series_list=[gnss_points],
        minimum_half_span=15.0,
        margin_ratio=0.10,
    )


def compute_full_extent_bounds(
    gnss_points: list,
    wheel: list,
    vio: list,
    ekf_local: list,
    ekf_global: list,
) -> dict:
    """
    すべての軌跡を含むズームアウト版の描画範囲を計算する。

    標準図では範囲外へはみ出す発散軌跡も、
    trajectory_07_overlay_full_extent.pngでは全体を確認できる。

    Returns:
        全系列を含む正方形描画範囲。
    """
    return compute_square_bounds_from_series(
        series_list=[
            gnss_points,
            wheel,
            vio,
            ekf_local,
            ekf_global,
        ],
        minimum_half_span=15.0,
        margin_ratio=0.10,
    )


def try_draw_basemap(
    ax,
    bounds: dict,
    mercator_origin,
) -> bool:
    """
    contextilyを使ってOpenStreetMap背景を描画する。

    軌跡は「最初のGNSS位置を原点とした局所座標」で描画する一方、
    地図タイルはWeb Mercatorの絶対座標で取得する必要がある。

    そのため:
        1. 局所boundsへ原点の絶対Web Mercator座標を加算する。
        2. その絶対範囲で地図タイルを取得する。
        3. 取得画像のextentから原点を減算し、局所座標へ戻して描画する。

    Args:
        ax:
            描画先のMatplotlib Axes。

        bounds:
            局所座標系の描画範囲。

        mercator_origin:
            最初のGNSS点のWeb Mercator絶対座標。

    Returns:
        背景描画に成功した場合はTrue。
        contextily未導入、GNSSなし、通信失敗などではFalse。
    """
    if not HAS_CONTEXTILY:
        return False

    if mercator_origin is None:
        return False

    origin_mx = mercator_origin["origin_mx"]
    origin_my = mercator_origin["origin_my"]

    # 局所描画範囲をWeb Mercator絶対座標へ戻す。
    west = bounds["xmin"] + origin_mx
    east = bounds["xmax"] + origin_mx
    south = bounds["ymin"] + origin_my
    north = bounds["ymax"] + origin_my

    try:
        # ll=Falseなので、引数は緯度経度ではなくEPSG:3857座標として渡す。
        image, extent = ctx.bounds2img(
            west,
            south,
            east,
            north,
            source=ctx.providers.OpenStreetMap.Mapnik,
            ll=False,
        )

        # contextilyが返す絶対座標extentを、軌跡と同じ局所座標へ変換する。
        shifted_extent = (
            extent[0] - origin_mx,
            extent[1] - origin_mx,
            extent[2] - origin_my,
            extent[3] - origin_my,
        )

        ax.imshow(
            image,
            extent=shifted_extent,
            origin="upper",
            alpha=0.85,

            # 軌跡より背面に配置する。
            zorder=0,
        )

        return True

    except Exception as error:
        # ネットワーク不通やタイル取得失敗でも、
        # 軌跡画像そのものは生成できるよう処理を継続する。
        print(
            f"    [WARN] Failed to fetch basemap: {error}"
        )
        return False


def configure_axis(
    ax,
    title: str,
    bounds: dict,
    mercator_origin,
) -> None:
    """
    各図で共通する軸設定と背景地図描画を行う。

    設定内容:
        - タイトル
        - x/y表示範囲
        - x/y同縮尺
        - 東・北方向の軸ラベル
        - グリッド
        - OpenStreetMap背景

    y軸正方向をNorthとするため、通常のMatplotlib座標系のまま使用する。
    """
    ax.set_title(title)

    ax.set_xlim(
        bounds["xmin"],
        bounds["xmax"],
    )
    ax.set_ylim(
        bounds["ymin"],
        bounds["ymax"],
    )

    # x方向1 mとy方向1 mを同じ画面長で表示する。
    ax.set_aspect(
        "equal",
        adjustable="box",
    )

    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")

    ax.grid(
        True,
        linestyle="--",
        linewidth=0.5,
        alpha=0.5,
    )

    try_draw_basemap(
        ax,
        bounds,
        mercator_origin,
    )


def plot_gnss(
    ax,
    gnss_points: list,
) -> None:
    """
    GNSS軌跡をRTK状態別の色・マーカーで描画する。

    表示:
        - 全GNSS点を黒い細線で接続する。
        - RTK FIX / FLOAT / GNSS / NO FIX / UNKNOWNごとに点を描き分ける。

    Args:
        ax:
            描画先Axes。

        gnss_points:
            局所座標とstateを持つGNSS点列。
    """
    if not gnss_points:
        ax.text(
            0.5,
            0.5,
            "No GNSS data",
            ha="center",
            va="center",
            transform=ax.transAxes,
        )
        return

    xs = [
        item["x"]
        for item in gnss_points
    ]
    ys = [
        item["y"]
        for item in gnss_points
    ]

    # 位置点同士の時系列的なつながりを薄い線で示す。
    ax.plot(
        xs,
        ys,
        color="black",
        linewidth=1.0,
        alpha=0.35,
        label="GNSS path",
        zorder=1,
    )

    # RTK状態ごとに対象点を抽出し、異なるスタイルで散布図表示する。
    for state_name, style in GNSS_STATE_STYLE.items():
        state_x = [
            item["x"]
            for item in gnss_points
            if item["state"] == state_name
        ]
        state_y = [
            item["y"]
            for item in gnss_points
            if item["state"] == state_name
        ]

        # 該当状態の点が存在しない場合は凡例にも追加しない。
        if not state_x:
            continue

        ax.scatter(
            state_x,
            state_y,
            s=18,
            c=style["color"],
            marker=style["marker"],
            label=style["label"],
            zorder=2,
        )

    ax.legend(loc="best")


def plot_single_series(
    ax,
    series: list,
    key: str,
    reference_bounds=None,
) -> None:
    """
    wheel・VIO・EKF等の単独軌跡を1本描画する。

    reference_boundsが指定されている場合は、
    GNSS基準表示範囲から外れた点数を図中へ注記する。

    Args:
        ax:
            描画先Axes。

        series:
            {"x", "y"}を持つ整列済み軌跡。

        key:
            LINE_STYLEを参照する内部キー。

        reference_bounds:
            標準図のGNSS基準範囲。
            Noneの場合は範囲外点数を計算しない。
    """
    if not series:
        ax.text(
            0.5,
            0.5,
            f"No data for {key}",
            ha="center",
            va="center",
            transform=ax.transAxes,
        )
        return

    xs = [
        item["x"]
        for item in series
    ]
    ys = [
        item["y"]
        for item in series
    ]

    style = LINE_STYLE[key]

    ax.plot(
        xs,
        ys,
        color=style["color"],
        linewidth=style["linewidth"],
        alpha=style["alpha"],
        label=style["label"],
        zorder=2,

        # 表示範囲外の線をAxes外へ描かない。
        clip_on=True,
    )

    if reference_bounds is not None:
        outside_count = count_points_outside_bounds(
            series,
            reference_bounds,
        )

        # 発散・大誤差により画面外へ出た点がある場合だけ警告を表示する。
        if outside_count > 0:
            ax.text(
                0.01,
                0.01,
                f"{outside_count} points outside GNSS extent",
                transform=ax.transAxes,
                ha="left",
                va="bottom",
                fontsize=8,
                bbox={
                    "boxstyle": "round",
                    "facecolor": "white",
                    "alpha": 0.85,
                },
                zorder=10,
            )

    ax.legend(loc="best")


def count_points_outside_bounds(
    series: list,
    bounds: dict,
) -> int:
    """
    指定描画範囲の外側にある軌跡点数を数える。

    不正値は範囲外点数へ含めず、単に無視する。

    Args:
        series:
            {"x", "y"}を持つ軌跡点列。

        bounds:
            xmin, xmax, ymin, ymaxを持つ描画範囲。

    Returns:
        描画範囲外に位置する有効点の数。
    """
    count = 0

    for item in series:
        x = item.get("x")
        y = item.get("y")

        if not finite_number(x) or not finite_number(y):
            continue

        if (
            x < bounds["xmin"]
            or x > bounds["xmax"]
            or y < bounds["ymin"]
            or y > bounds["ymax"]
        ):
            count += 1

    return count


def add_outside_extent_annotation(
    ax,
    bounds: dict,
    series_by_name: list,
) -> None:
    """
    重ね合わせ図へ、系列別の範囲外点数を注記する。

    Args:
        ax:
            描画先Axes。

        bounds:
            GNSS基準の標準描画範囲。

        series_by_name:
            (表示名, 軌跡系列)のリスト。
    """
    messages = []

    for name, series in series_by_name:
        outside_count = count_points_outside_bounds(
            series,
            bounds,
        )

        if outside_count > 0:
            messages.append(
                f"{name}: {outside_count} points outside view"
            )

    # すべて範囲内なら注記は不要。
    if not messages:
        return

    ax.text(
        0.01,
        0.01,
        "\n".join(messages),
        transform=ax.transAxes,
        ha="left",
        va="bottom",
        fontsize=8,
        bbox={
            "boxstyle": "round",
            "facecolor": "white",
            "alpha": 0.85,
        },
        zorder=10,
    )


def plot_overlay(
    ax,
    gnss_points: list,
    wheel: list,
    vio: list,
    ekf_local: list,
    ekf_global: list,
    reference_bounds=None,
) -> None:
    """
    GNSSと4種類の自己位置推定軌跡を同じ図へ重ね合わせる。

    GNSS:
        RTK状態別の点と軌跡線を表示する。

    その他:
        LINE_STYLEに従って各軌跡を線表示する。

    reference_boundsが指定される標準図では、
    各推定軌跡の範囲外点数も注記する。

    Args:
        ax:
            描画先Axes。

        gnss_points:
            RTK状態付きGNSS点。

        wheel, vio, ekf_local, ekf_global:
            GNSS・北基準へ整列済みの軌跡。

        reference_bounds:
            GNSS基準描画範囲。
            Noneなら範囲外注記を行わない。
    """
    has_any = False

    if gnss_points:
        plot_gnss(
            ax,
            gnss_points,
        )
        has_any = True

    # 4種類の推定軌跡を同じ処理で描画する。
    for key, series in [
        ("wheel", wheel),
        ("vio", vio),
        ("ekf_local", ekf_local),
        ("ekf_global", ekf_global),
    ]:
        if not series:
            continue

        has_any = True

        xs = [
            item["x"]
            for item in series
        ]
        ys = [
            item["y"]
            for item in series
        ]

        style = LINE_STYLE[key]

        ax.plot(
            xs,
            ys,
            color=style["color"],
            linewidth=style["linewidth"],
            alpha=style["alpha"],
            label=style["label"],
            zorder=3,
            clip_on=True,
        )

    if not has_any:
        ax.text(
            0.5,
            0.5,
            "No trajectory data",
            ha="center",
            va="center",
            transform=ax.transAxes,
        )

    if reference_bounds is not None:
        add_outside_extent_annotation(
            ax=ax,
            bounds=reference_bounds,
            series_by_name=[
                ("Wheel", wheel),
                ("VIO", vio),
                ("EKF local", ekf_local),
                ("EKF global", ekf_global),
            ],
        )

    ax.legend(loc="best")


def save_figure(
    output_path: Path,
    title: str,
    bounds: dict,
    mercator_origin,
    plot_callback,
) -> None:
    """
    1枚の図を作成・保存し、Matplotlibリソースを解放する。

    描画内容をplot_callbackとして外から渡すことで、
    軸設定・背景地図・保存処理を共通化する。

    Args:
        output_path:
            保存するPNGファイルパス。

        title:
            図のタイトル。

        bounds:
            x/y描画範囲。

        mercator_origin:
            背景地図の座標変換に使うWeb Mercator原点。

        plot_callback:
            引数としてAxesを受け取り、軌跡を描画する関数。
            呼び出し側ではlambdaを使用している。
    """
    fig, ax = plt.subplots(
        figsize=(8, 8),
        dpi=150,
    )

    configure_axis(
        ax,
        title,
        bounds,
        mercator_origin,
    )

    # GNSS単独、wheel単独、重ね合わせ等の個別描画処理を実行する。
    plot_callback(ax)

    # タイトル・軸ラベル・凡例が図外へ切れにくいようレイアウトを調整する。
    fig.tight_layout()

    fig.savefig(output_path)

    # 多数のbagを連続処理するため、図を明示的に閉じてメモリを解放する。
    plt.close(fig)


def all_output_files_exist(
    bag_dir: Path,
) -> bool:
    """
    予定される全PNGファイルが存在するか確認する。

    現在のスキップ判定は完了マーカーを使用するため、
    この関数は補助確認用として残されている。

    Args:
        bag_dir:
            確認対象のbagディレクトリ。

    Returns:
        OUTPUT_FILESに列挙した全ファイルが存在すればTrue。
    """
    return all(
        (bag_dir / name).exists()
        for name in OUTPUT_FILES.values()
    )


def process_one_bag(
    bag_dir: Path,
    overwrite: bool,
) -> str:
    """
    1つのrosbagについて、データ抽出・座標整列・7枚の画像保存を行う。

    処理順:
        1. 完了マーカーによるスキップ判定。
        2. bagから必要データを抽出。
        3. RTK状態系列を選択。
        4. GNSSを局所座標化。
        5. wheel / VIO / EKFをGNSS・北基準へ初期整列。
        6. GNSS基準範囲と全軌跡範囲を計算。
        7. 7枚のPNGを保存。
        8. 完了マーカーJSONを原子的に保存。

    Returns:
        "processed":
            正常に解析・保存した。

        "skipped":
            既に完了マーカーが存在し、再解析しなかった。
    """
    if should_skip_bag(
        bag_dir,
        overwrite,
    ):
        print(
            f"[SKIP] {bag_dir}: "
            f"{ANALYSIS_COMPLETE_FILENAME} already exists"
        )
        return "skipped"

    print(f"[BAG ] {bag_dir}")

    # rosbagから軌跡生成に必要なデータを一括抽出する。
    data = read_bag_data(bag_dir)

    # 利用可能な中で最も優先度の高いRTK状態系列を選ぶ。
    rtk_states = choose_rtk_status_stream(data)

    # GNSSをWeb Mercator差分による局所東北座標へ変換する。
    gnss_points, mercator_origin = convert_fixes_to_local_xy(
        data["fixes"],
        rtk_states,
    )

    # 各ローカル軌跡を、それぞれの開始時刻に対応する
    # IMU方位とGNSS位置へ初期整列する。
    wheel_aligned = align_local_trajectory_to_north(
        data["odom_data"]["wheel"],
        data["imu_yaws"],
        gnss_points,
    )

    vio_aligned = align_local_trajectory_to_north(
        data["odom_data"]["vio"],
        data["imu_yaws"],
        gnss_points,
    )

    ekf_local_aligned = align_local_trajectory_to_north(
        data["odom_data"]["ekf_local"],
        data["imu_yaws"],
        gnss_points,
    )

    ekf_global_aligned = align_local_trajectory_to_north(
        data["odom_data"]["ekf_global"],
        data["imu_yaws"],
        gnss_points,
    )

    # ------------------------------------------------------------------
    # 標準表示範囲
    #
    # GNSS軌跡だけを基準にする。
    # wheel / VIO / EKFが破綻・発散しても、
    # GNSSや正常軌跡の表示が小さくなり過ぎない。
    # ------------------------------------------------------------------
    reference_bounds = compute_gnss_reference_bounds(
        gnss_points
    )

    # ------------------------------------------------------------------
    # 全体表示範囲
    #
    # すべての軌跡を含める。
    # 発散の大きさや方向を確認するズームアウト版に使用する。
    # ------------------------------------------------------------------
    full_extent_bounds = compute_full_extent_bounds(
        gnss_points=gnss_points,
        wheel=wheel_aligned,
        vio=vio_aligned,
        ekf_local=ekf_local_aligned,
        ekf_global=ekf_global_aligned,
    )

    bag_name = bag_dir.name

    # 1. GNSS単独
    save_figure(
        bag_dir / OUTPUT_FILES["gnss"],
        f"{bag_name} : GNSS",
        reference_bounds,
        mercator_origin,
        lambda ax: plot_gnss(
            ax,
            gnss_points,
        ),
    )

    # 2. ホイールオドメトリ単独
    save_figure(
        bag_dir / OUTPUT_FILES["wheel"],
        f"{bag_name} : Wheel odometry",
        reference_bounds,
        mercator_origin,
        lambda ax: plot_single_series(
            ax,
            wheel_aligned,
            "wheel",
            reference_bounds=reference_bounds,
        ),
    )

    # 3. ビジュアルオドメトリ単独
    save_figure(
        bag_dir / OUTPUT_FILES["vio"],
        f"{bag_name} : Visual odometry",
        reference_bounds,
        mercator_origin,
        lambda ax: plot_single_series(
            ax,
            vio_aligned,
            "vio",
            reference_bounds=reference_bounds,
        ),
    )

    # 4. EKF local単独
    save_figure(
        bag_dir / OUTPUT_FILES["ekf_local"],
        f"{bag_name} : EKF local",
        reference_bounds,
        mercator_origin,
        lambda ax: plot_single_series(
            ax,
            ekf_local_aligned,
            "ekf_local",
            reference_bounds=reference_bounds,
        ),
    )

    # 5. EKF global単独
    save_figure(
        bag_dir / OUTPUT_FILES["ekf_global"],
        f"{bag_name} : EKF global",
        reference_bounds,
        mercator_origin,
        lambda ax: plot_single_series(
            ax,
            ekf_global_aligned,
            "ekf_global",
            reference_bounds=reference_bounds,
        ),
    )

    # 6. GNSS基準範囲での全軌跡重ね合わせ
    save_figure(
        bag_dir / OUTPUT_FILES["overlay"],
        f"{bag_name} : Overlay — GNSS reference extent",
        reference_bounds,
        mercator_origin,
        lambda ax: plot_overlay(
            ax,
            gnss_points,
            wheel_aligned,
            vio_aligned,
            ekf_local_aligned,
            ekf_global_aligned,
            reference_bounds=reference_bounds,
        ),
    )

    # 7. 全軌跡が収まる範囲でのズームアウト重ね合わせ
    save_figure(
        bag_dir / OUTPUT_FILES["overlay_full"],
        f"{bag_name} : Overlay — full trajectory extent",
        full_extent_bounds,
        mercator_origin,
        lambda ax: plot_overlay(
            ax,
            gnss_points,
            wheel_aligned,
            vio_aligned,
            ekf_local_aligned,
            ekf_global_aligned,
        ),
    )

    # 全画像の保存完了後にだけ完了マーカーを作る。
    marker_path = analysis_complete_marker_path(
        bag_dir
    )

    marker_data = {
        "analysis": "trajectory_plots",
        "analysis_version": ANALYSIS_VERSION,

        # UTCのISO 8601形式で解析完了時刻を記録する。
        "completed_at_utc": (
            datetime.now(timezone.utc).isoformat()
        ),

        # この解析で生成したファイル一覧。
        "output_files": list(OUTPUT_FILES.values()),

        # 標準図の表示範囲決定方法。
        "standard_extent": (
            "GNSS-based common square extent"
        ),

        # 全体表示版のファイル名。
        "full_extent_output": OUTPUT_FILES["overlay_full"],
    }

    # JSONもPNG解析と同様、一時ファイルへ完全に書いてから置換する。
    # 途中停止時に不完全な完了マーカーが残ることを防ぐ。
    temporary_marker = marker_path.with_suffix(
        marker_path.suffix + ".tmp"
    )

    with temporary_marker.open(
        "w",
        encoding="utf-8",
    ) as stream:
        json.dump(
            marker_data,
            stream,
            ensure_ascii=False,
            indent=2,
        )
        stream.write("\n")

    temporary_marker.replace(marker_path)

    # 元スクリプトではSaved 7 plotsが2回表示される。
    # 動作を変えないため、この重複ログもそのまま残している。
    print("      Saved 7 plots")
    print(
        f"      Written marker: {marker_path.name}"
    )

    return "processed"


def find_bag_directories(
    bags_root: Path,
) -> list:
    """
    指定ルート以下のすべてのrosbagディレクトリを再帰的に探索する。

    metadata.yamlを持つディレクトリをrosbagディレクトリとみなす。
    そのため、bags/old以下も自動的に対象となる。

    Args:
        bags_root:
            rosbag群のルートディレクトリ。

    Returns:
        metadata.yamlを含むディレクトリのソート済みリスト。
    """
    return sorted(
        metadata_path.parent
        for metadata_path in bags_root.rglob("metadata.yaml")
        if metadata_path.is_file()
    )


def main() -> int:
    """
    スクリプト全体のエントリーポイント。

    処理:
        1. 引数を解析する。
        2. bagsルートの存在を確認する。
        3. rosbagディレクトリを再帰探索する。
        4. 各bagを順番に処理する。
        5. processed / skipped / failed件数を表示する。
        6. 失敗が1件でもあれば終了コード1を返す。

    Returns:
        0:
            全bagが成功またはスキップ。

        1:
            1件以上のbagで失敗、またはbagが見つからない。

        2:
            指定されたbags_rootがディレクトリではない。
    """
    args = parse_args()

    # "~"を展開し、絶対パスへ正規化する。
    bags_root = args.bags_root.expanduser().resolve()

    if not bags_root.is_dir():
        print(
            f"Not a directory: {bags_root}",
            file=sys.stderr,
        )
        return 2

    bag_dirs = find_bag_directories(
        bags_root
    )

    if not bag_dirs:
        print(
            f"No bag directories found below: {bags_root}",
            file=sys.stderr,
        )
        return 1

    print(f"Bags root: {bags_root}")
    print(
        f"Found {len(bag_dirs)} bag directories"
    )

    result_counts = {
        "processed": 0,
        "skipped": 0,
        "failed": 0,
    }

    for bag_dir in bag_dirs:
        try:
            result = process_one_bag(
                bag_dir,
                args.overwrite,
            )

            result_counts[result] += 1

        except Exception as error:
            # 1つのbagで失敗しても残りのbag処理を継続する。
            result_counts["failed"] += 1

            print(
                f"[FAIL] {bag_dir}: "
                f"{type(error).__name__}: {error}",
                file=sys.stderr,
            )

    print("")
    print("Summary")
    print(
        f"  Processed: "
        f"{result_counts['processed']}"
    )
    print(
        f"  Skipped:   "
        f"{result_counts['skipped']}"
    )
    print(
        f"  Failed:    "
        f"{result_counts['failed']}"
    )

    # 失敗が1件でもあれば、シェルやCIから検知できるよう1を返す。
    return (
        1
        if result_counts["failed"] > 0
        else 0
    )


if __name__ == "__main__":
    # main()の戻り値をプロセス終了コードとして使用する。
    raise SystemExit(main())