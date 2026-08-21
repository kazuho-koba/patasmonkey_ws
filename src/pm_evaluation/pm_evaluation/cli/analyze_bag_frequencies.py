#!/usr/bin/env python3

"""
rosbag2（MCAP形式）に記録された各トピックについて、
メッセージ数、記録開始・終了時刻、平均周波数などを集計するスクリプト。

指定したbagsルートディレクトリ以下を再帰的に探索し、
各rosbagディレクトリにあるmetadata.yamlとMCAP本体を読み込む。

解析結果は、各metadata.yamlと同じディレクトリに
topic_frequency.csvとして保存する。

既に出力CSVが存在するbagは、--overwriteを指定しない限りスキップする。
"""

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import yaml
from mcap.reader import make_reader


# 各bagディレクトリに保存するCSVファイルのデフォルト名
DEFAULT_OUTPUT_FILENAME = "topic_frequency.csv"


@dataclass
class TopicStatistics:
    """
    1つのトピックについて集計した統計情報。

    Attributes:
        topic_name:
            ROSトピック名。

        message_count:
            MCAP本体から実際に読み取ったメッセージ数。

        first_timestamp_ns:
            最初に記録されたメッセージのbag記録時刻。
            単位はナノ秒。

        last_timestamp_ns:
            最後に記録されたメッセージのbag記録時刻。
            単位はナノ秒。
    """

    topic_name: str
    message_count: int = 0
    first_timestamp_ns: Optional[int] = None
    last_timestamp_ns: Optional[int] = None

    def add_message(self, timestamp_ns: int) -> None:
        """
        1件のメッセージ記録時刻を統計に追加する。

        メッセージ数を1増やすとともに、
        これまでに記録された最小時刻と最大時刻を更新する。

        分割MCAPファイルが想定外の順番で読み込まれた場合でも
        正しく開始・終了時刻を得られるよう、単純に先頭・末尾を採用せず、
        各メッセージの時刻を比較している。
        """
        self.message_count += 1

        # 初めて時刻を記録する場合、または現在の開始時刻より古い場合に更新する
        if (
            self.first_timestamp_ns is None
            or timestamp_ns < self.first_timestamp_ns
        ):
            self.first_timestamp_ns = timestamp_ns

        # 初めて時刻を記録する場合、または現在の終了時刻より新しい場合に更新する
        if (
            self.last_timestamp_ns is None
            or timestamp_ns > self.last_timestamp_ns
        ):
            self.last_timestamp_ns = timestamp_ns

    @property
    def active_duration_sec(self) -> float:
        """
        そのトピックの最初のメッセージから最後のメッセージまでの時間を返す。

        単位は秒。

        注意:
            ここでいう有効時間は、
            「最初のメッセージ時刻から最後のメッセージ時刻までの経過時間」
            である。

            途中でトピックが一時停止していた場合、その停止時間も含まれる。

            メッセージが0件または1件の場合は時刻差を計算できないため、
            0.0を返す。
        """
        if (
            self.first_timestamp_ns is None
            or self.last_timestamp_ns is None
            or self.last_timestamp_ns <= self.first_timestamp_ns
        ):
            return 0.0

        # ナノ秒単位の時刻差を秒へ変換する
        return (
            self.last_timestamp_ns - self.first_timestamp_ns
        ) * 1.0e-9

    @property
    def count_divided_by_duration_hz(self) -> Optional[float]:
        """
        「メッセージ数 ÷ トピック有効時間」を計算する。

        ユーザーが当初指定した周波数の定義に対応する値。

        ただし、N件のメッセージの間にはN-1個の時間間隔しか存在しないため、
        通常のpublish周波数よりわずかに大きな値になることがある。

        有効時間が0秒の場合は計算できないためNoneを返す。
        NoneはCSVでは空欄として出力される。
        """
        duration = self.active_duration_sec

        if duration <= 0.0:
            return None

        return self.message_count / duration

    @property
    def interval_frequency_hz(self) -> Optional[float]:
        """
        メッセージ間隔に基づく通常の平均周波数を計算する。

        計算式:
            (メッセージ数 - 1) / 有効時間

        例えば10 Hzでメッセージが100件記録された場合、
        最初から最後までには99個のメッセージ間隔が存在する。

        そのため、通常のpublish周波数を評価する場合は、
        count_divided_by_duration_hzよりもこの値が適している。

        メッセージが2件未満、または有効時間が0秒の場合は
        計算できないためNoneを返す。
        """
        duration = self.active_duration_sec

        if duration <= 0.0 or self.message_count < 2:
            return None

        return (self.message_count - 1) / duration


@dataclass
class BagMetadata:
    """
    1つのrosbagディレクトリについてmetadata.yamlから取得した情報。

    Attributes:
        bag_directory:
            metadata.yamlが存在するbagディレクトリ。

        storage_identifier:
            rosbag2のストレージ形式。
            このスクリプトでは"mcap"を想定する。

        bag_duration_sec:
            metadata.yamlに記録されたbag全体の長さ。
            単位は秒。

        metadata_message_counts:
            metadata.yamlに記録されたトピック別メッセージ数。

        relative_file_paths:
            metadata.yamlに記録された分割bagファイルの相対パス一覧。
    """

    bag_directory: Path
    storage_identifier: str
    bag_duration_sec: float
    metadata_message_counts: Dict[str, int]
    relative_file_paths: List[str]


def parse_arguments() -> argparse.Namespace:
    """
    コマンドライン引数を定義し、解析結果を返す。

    位置引数:
        bags_root:
            rosbagディレクトリ群を含むルートディレクトリ。

    オプション:
        --output-name:
            各bagディレクトリに保存するCSVファイル名。

        --overwrite:
            既存CSVが存在するbagも再解析して上書きする。
    """
    parser = argparse.ArgumentParser(
        description=(
            "Recursively analyze all rosbag2 directories below a bags root "
            "and write per-topic frequency statistics into each bag directory."
        )
    )

    parser.add_argument(
        "bags_roots",
        type=Path,
        nargs="+",
        help=(
            "One or more rosbag2 directories or root directories. "
            "Shell wildcards such as rosbag2_2026_08_21* are supported."
        ),
    )

    parser.add_argument(
        "--output-name",
        default=DEFAULT_OUTPUT_FILENAME,
        help=(
            "Output CSV filename written inside each bag directory. "
            f"Default: {DEFAULT_OUTPUT_FILENAME}"
        ),
    )

    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite an existing output CSV instead of skipping the bag.",
    )

    return parser.parse_args()


def nested_nanoseconds(value: object) -> int:
    """
    metadata.yaml内のナノ秒値を安全に整数として読み取る。

    ROS 2 Foxy世代のmetadata.yamlでは、
    durationが次のような入れ子構造になっていることがある。

        duration:
          nanoseconds: 123

    一方、環境やバージョンによっては値が直接記録される可能性もあるため、
    辞書形式と直接値の両方に対応する。

    値が整数へ変換できない場合は例外を送出せず、0を返す。
    """
    if isinstance(value, dict):
        raw_value = value.get("nanoseconds", 0)
    else:
        raw_value = value

    try:
        return int(raw_value)
    except (TypeError, ValueError):
        return 0


def load_bag_metadata(metadata_path: Path) -> BagMetadata:
    """
    1つのmetadata.yamlを読み込み、解析に必要な情報を返す。

    読み取る主な項目:
        - ストレージ形式
        - bag全体の記録時間
        - 分割MCAPファイル一覧
        - トピック別メッセージ数

    metadata.yamlのルート直下に情報がある形式と、
    rosbag2_bagfile_informationの下に情報がある形式の両方へ対応する。
    """
    with metadata_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)

    # YAML全体が辞書でなければ、期待するmetadata形式ではない
    if not isinstance(document, dict):
        raise ValueError("metadata.yaml root is not a dictionary")

    # 通常はrosbag2_bagfile_information以下に情報が格納される。
    # 該当キーがない形式では、document自体を情報本体として扱う。
    information = document.get(
        "rosbag2_bagfile_information",
        document,
    )

    if not isinstance(information, dict):
        raise ValueError(
            "rosbag2_bagfile_information is missing or invalid"
        )

    # rosbag2のストレージ形式を取得する
    storage_identifier = str(
        information.get("storage_identifier", "")
    )

    # bag全体の記録時間をナノ秒で取得する
    duration_ns = nested_nanoseconds(
        information.get("duration", {})
    )

    # 分割されたbagファイルの相対パス一覧
    relative_file_paths = information.get(
        "relative_file_paths",
        [],
    )

    if not isinstance(relative_file_paths, list):
        raise ValueError("relative_file_paths is not a list")

    # metadata.yamlに記録されたトピック別メッセージ数を格納する
    metadata_message_counts: Dict[str, int] = {}

    topics = information.get(
        "topics_with_message_count",
        [],
    )

    if isinstance(topics, list):
        for entry in topics:
            # 不正な要素が混在していても、その要素だけ無視して処理を続ける
            if not isinstance(entry, dict):
                continue

            topic_metadata = entry.get(
                "topic_metadata",
                {},
            )

            if not isinstance(topic_metadata, dict):
                continue

            topic_name = topic_metadata.get("name")

            # トピック名が存在しない項目は集計対象外
            if not topic_name:
                continue

            try:
                message_count = int(
                    entry.get("message_count", 0)
                )
            except (TypeError, ValueError):
                message_count = 0

            metadata_message_counts[str(topic_name)] = message_count

    return BagMetadata(
        bag_directory=metadata_path.parent,
        storage_identifier=storage_identifier,

        # ナノ秒単位のbag時間を秒へ変換する
        bag_duration_sec=duration_ns * 1.0e-9,

        metadata_message_counts=metadata_message_counts,

        # Pathではなく文字列として保持する
        relative_file_paths=[
            str(relative_path)
            for relative_path in relative_file_paths
        ],
    )


def natural_mcap_sort_key(path: Path) -> Tuple[str, int]:
    """
    分割MCAPファイルを末尾番号の数値順に並べるためのソートキーを返す。

    通常の文字列ソートでは次のように並ぶ。

        bag_0.mcap
        bag_1.mcap
        bag_10.mcap
        bag_2.mcap

    この関数では末尾の「_数字」を数値として解釈し、
    次のような順になるようにする。

        bag_0.mcap
        bag_1.mcap
        bag_2.mcap
        bag_10.mcap
    """
    stem = path.stem

    # 文字列を末尾側の最後のアンダースコアで分割する
    prefix, separator, suffix = stem.rpartition("_")

    # アンダースコア後の部分が数字なら、整数としてソートキーにする
    if separator and suffix.isdigit():
        return prefix, int(suffix)

    # 末尾が数字でない場合もソート可能なキーを返す
    return stem, -1


def find_mcap_files(metadata: BagMetadata) -> List[Path]:
    """
    1つのbagに属するMCAPファイルを特定する。

    処理順:
        1. metadata.yamlのrelative_file_pathsを優先する。
        2. 有効なファイルが見つからない場合は、
           bagディレクトリ直下の*.mcapを探索する。
        3. 重複を除去する。
        4. 分割番号を数値順に並べる。
    """
    files: List[Path] = []

    # metadata.yamlに明示されたファイルを優先して収集する
    for relative_path in metadata.relative_file_paths:
        candidate = metadata.bag_directory / relative_path

        if (
            candidate.suffix.lower() == ".mcap"
            and candidate.is_file()
        ):
            files.append(candidate)

    # metadata.yamlの一覧が空または不完全な場合のフォールバック
    if not files:
        files = list(
            metadata.bag_directory.glob("*.mcap")
        )

    # setで重複を除去し、_0, _1, _2, ...の数値順に並べる
    unique_files = sorted(
        set(files),
        key=natural_mcap_sort_key,
    )

    return unique_files


def read_mcap_statistics(
    mcap_files: Iterable[Path],
) -> Dict[str, TopicStatistics]:
    """
    すべての分割MCAPを読み、トピック別の統計を集計する。

    この処理ではメッセージ本文をデシリアライズせず、
    MCAPが持つ次の情報だけを使用する。

        - channel.topic
        - message.log_time

    そのため、画像や点群など巨大なメッセージについても、
    ROSメッセージへ展開する方法より比較的軽量に、
    メッセージ数と記録時刻を集計できる。
    """
    statistics: Dict[str, TopicStatistics] = {}

    for mcap_path in mcap_files:
        print(f"      Reading: {mcap_path.name}")

        with mcap_path.open("rb") as stream:
            reader = make_reader(stream)

            # schema、channel、messageが返される。
            # この解析ではschemaを使用しないため_schemaとして受け取る。
            for _schema, channel, message in reader.iter_messages():
                topic_name = channel.topic

                # 初めて現れたトピックの場合はTopicStatisticsを生成する。
                # 既に存在する場合は既存のインスタンスを取得する。
                topic_statistics = statistics.setdefault(
                    topic_name,
                    TopicStatistics(topic_name=topic_name),
                )

                # rosbag2 MCAPではmessage.log_timeが
                # bagへの記録時刻に相当する。
                #
                # ここで使用しているのはメッセージ内のHeader.stampではなく、
                # rosbagレコーダがメッセージを記録した時刻である。
                topic_statistics.add_message(
                    int(message.log_time)
                )

    return statistics


def format_float(
    value: Optional[float],
    digits: int = 6,
) -> str:
    """
    Optionalな浮動小数点値をCSV用文字列へ変換する。

    Noneの場合は空文字を返す。

    計算不能な値を0として出力すると、
    「周波数が本当に0 Hzだった」と誤認する可能性があるため、
    計算不能な場合は空欄としている。
    """
    if value is None:
        return ""

    return f"{value:.{digits}f}"


def timestamp_ns_to_sec(
    timestamp_ns: Optional[int],
) -> str:
    """
    ナノ秒単位の時刻を秒単位の文字列へ変換する。

    絶対時刻の精度をできるだけ保持するため、
    小数点以下9桁で出力する。
    """
    if timestamp_ns is None:
        return ""

    return f"{timestamp_ns * 1.0e-9:.9f}"


def write_csv(
    output_path: Path,
    metadata: BagMetadata,
    statistics: Dict[str, TopicStatistics],
) -> None:
    """
    1つのbagについて、トピック別統計をCSVへ保存する。

    一時ファイルへ全内容を書き終えてから、
    正式なファイル名へ置き換える「原子的な保存」を行う。

    この方法により、書き込み途中で処理が停止した場合に、
    不完全なCSVが完成済みファイルとして残る可能性を低減する。
    """
    # 例:
    # topic_frequency.csv
    #     ↓
    # topic_frequency.csv.tmp
    temporary_path = output_path.with_suffix(
        output_path.suffix + ".tmp"
    )

    # CSVへ出力する列の名称と順序
    fieldnames = [
        "topic",
        "mcap_message_count",
        "metadata_message_count",
        "message_count_difference",
        "first_record_timestamp_sec",
        "last_record_timestamp_sec",
        "active_duration_sec",
        "bag_duration_sec",
        "active_coverage_ratio",
        "count_divided_by_active_duration_hz",
        "interval_frequency_hz",
        "count_divided_by_bag_duration_hz",
    ]

    # MCAP本体かmetadata.yamlのどちらか一方だけに存在するトピックも
    # CSVへ残せるよう、両方のトピック名の和集合を使用する
    all_topic_names = sorted(
        set(statistics)
        | set(metadata.metadata_message_counts)
    )

    with temporary_path.open(
        "w",
        encoding="utf-8",
        newline="",
    ) as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=fieldnames,
        )
        writer.writeheader()

        for topic_name in all_topic_names:
            # MCAP本体側にトピックが存在しない場合は、
            # メッセージ数0件の統計情報を仮生成する
            topic_statistics = statistics.get(
                topic_name,
                TopicStatistics(topic_name=topic_name),
            )

            # metadata.yaml側にトピックが存在しない場合は0件とする
            metadata_count = metadata.metadata_message_counts.get(
                topic_name,
                0,
            )

            active_duration_sec = (
                topic_statistics.active_duration_sec
            )

            if metadata.bag_duration_sec > 0.0:
                # bag全体の記録時間に対して、
                # トピックの最初から最後までの時間が占める割合
                active_coverage_ratio = (
                    active_duration_sec
                    / metadata.bag_duration_sec
                )

                # トピックのメッセージ数をbag全体の記録時間で割った値
                #
                # トピックの起動遅延や途中停止も含まれるため、
                # 実際のpublish周期より低くなることがある
                count_divided_by_bag_duration_hz = (
                    topic_statistics.message_count
                    / metadata.bag_duration_sec
                )
            else:
                active_coverage_ratio = None
                count_divided_by_bag_duration_hz = None

            writer.writerow(
                {
                    "topic": topic_name,

                    # MCAP本体を実際に走査して数えたメッセージ数
                    "mcap_message_count": (
                        topic_statistics.message_count
                    ),

                    # metadata.yamlに記録されていたメッセージ数
                    "metadata_message_count": metadata_count,

                    # MCAP本体の件数とmetadata.yamlの件数との差
                    # 通常は0になる
                    "message_count_difference": (
                        topic_statistics.message_count
                        - metadata_count
                    ),

                    # トピックの最初と最後のbag記録時刻
                    "first_record_timestamp_sec": (
                        timestamp_ns_to_sec(
                            topic_statistics.first_timestamp_ns
                        )
                    ),
                    "last_record_timestamp_sec": (
                        timestamp_ns_to_sec(
                            topic_statistics.last_timestamp_ns
                        )
                    ),

                    # トピックの最初のメッセージから
                    # 最後のメッセージまでの経過時間
                    "active_duration_sec": (
                        format_float(
                            active_duration_sec,
                            9,
                        )
                    ),

                    # metadata.yamlに記録されたbag全体の記録時間
                    "bag_duration_sec": (
                        format_float(
                            metadata.bag_duration_sec,
                            9,
                        )
                    ),

                    # active_duration_sec / bag_duration_sec
                    "active_coverage_ratio": (
                        format_float(
                            active_coverage_ratio,
                            6,
                        )
                    ),

                    # メッセージ数 / トピック有効時間
                    #
                    # 当初指定された周波数の計算方法だが、
                    # 一般的なpublish周波数よりわずかに高く出る場合がある
                    "count_divided_by_active_duration_hz": (
                        format_float(
                            topic_statistics
                            .count_divided_by_duration_hz,
                            6,
                        )
                    ),

                    # (メッセージ数 - 1) / トピック有効時間
                    #
                    # 通常の平均publish周波数として
                    # 最も参照しやすい値
                    "interval_frequency_hz": (
                        format_float(
                            topic_statistics
                            .interval_frequency_hz,
                            6,
                        )
                    ),

                    # メッセージ数 / bag全体の記録時間
                    #
                    # トピックの起動遅延や途中停止の影響を含む
                    "count_divided_by_bag_duration_hz": (
                        format_float(
                            count_divided_by_bag_duration_hz,
                            6,
                        )
                    ),
                }
            )

    # 全行の書き込みが成功した後で、
    # 一時ファイルを正式な出力ファイル名へ置き換える
    temporary_path.replace(output_path)


def find_metadata_files(
    bags_root: Path,
) -> List[Path]:
    """
    指定されたルートディレクトリ以下に存在する
    すべてのmetadata.yamlを再帰的に探索する。

    rglobを使用するため、bags/old以下などの
    サブディレクトリも解析対象になる。
    """
    return sorted(
        path
        for path in bags_root.rglob("metadata.yaml")
        if path.is_file()
    )


def analyze_one_bag(
    metadata_path: Path,
    output_name: str,
    overwrite: bool,
) -> str:
    """
    1つのrosbagディレクトリを解析する。

    Returns:
        "processed":
            解析とCSV保存に成功した。

        "skipped":
            既存CSVがあり、--overwriteが指定されていないため
            解析をスキップした。

        "failed":
            非MCAP形式、MCAPファイルなし、読み取り失敗などにより
            解析できなかった。
    """
    output_path = metadata_path.parent / output_name

    # 既存の解析結果があり、上書き指定もない場合は再処理しない
    if output_path.exists() and not overwrite:
        print(
            f"[SKIP] {metadata_path.parent}: "
            f"{output_name} already exists"
        )
        return "skipped"

    print(f"[BAG]  {metadata_path.parent}")

    try:
        # metadata.yamlからbag全体の基本情報を取得する
        metadata = load_bag_metadata(metadata_path)

        # このスクリプトはMCAP形式を直接読み込むため、
        # MCAP以外のストレージ形式は処理しない
        if metadata.storage_identifier.lower() != "mcap":
            print(
                f"[FAIL] Unsupported storage identifier: "
                f"{metadata.storage_identifier!r}",
                file=sys.stderr,
            )
            return "failed"

        # metadata.yamlまたはbagディレクトリから
        # 分割MCAPファイルを特定する
        mcap_files = find_mcap_files(metadata)

        if not mcap_files:
            print(
                "[FAIL] No MCAP files were found",
                file=sys.stderr,
            )
            return "failed"

        # 全MCAPを走査し、
        # トピック別の件数と最初・最後の記録時刻を集計する
        statistics = read_mcap_statistics(mcap_files)

        if not statistics:
            print(
                "[FAIL] No messages were read from the MCAP files",
                file=sys.stderr,
            )
            return "failed"

        # 集計結果をmetadata.yamlと同じディレクトリへ保存する
        write_csv(
            output_path=output_path,
            metadata=metadata,
            statistics=statistics,
        )

    except Exception as error:
        # 1つのbagで例外が発生しても、
        # 呼び出し元で残りのbagを処理できるよう、
        # 例外をここで捕捉してfailedとして返す
        print(
            f"[FAIL] {metadata_path.parent}: "
            f"{type(error).__name__}: {error}",
            file=sys.stderr,
        )
        return "failed"

    print(f"[OK]   Written: {output_path}")
    return "processed"


def main() -> int:
    """
    スクリプト全体のエントリーポイント。

    処理の流れ:
        1. コマンドライン引数を取得する。
        2. bagsルートディレクトリの存在を確認する。
        3. metadata.yamlを再帰的に探索する。
        4. 各bagを順番に解析する。
        5. 成功・スキップ・失敗件数を表示する。
        6. 失敗が1件でもあれば終了コード1を返す。
    """
    args = parse_arguments()

    # "~"を展開し、絶対パスへ正規化する
    bags_roots = [
        path.expanduser().resolve()
        for path in args.bags_roots
    ]

    metadata_files = []

    for bags_root in bags_roots:
        if not bags_root.is_dir():
            print(
                f"Bags root does not exist or is not a directory: "
                f"{bags_root}",
                file=sys.stderr,
            )
            return 2

        metadata_files.extend(
            find_metadata_files(bags_root)
        )

    # 同じbagが複数rootから見つかった場合に重複処理しない。
    metadata_files = sorted(set(metadata_files))

    if not metadata_files:
        print(
            f"No metadata.yaml files found below: "
            f"{bags_root}",
            file=sys.stderr,
        )
        return 1

    print("Input roots:")
    for bags_root in bags_roots:
        print(f"  {bags_root}")
    print(
        f"Found {len(metadata_files)} bag directories"
    )

    # 最終サマリー表示用の件数カウンタ
    result_counts = {
        "processed": 0,
        "skipped": 0,
        "failed": 0,
    }

    # 各metadata.yamlを1つずつ処理する
    for metadata_path in metadata_files:
        result = analyze_one_bag(
            metadata_path=metadata_path,
            output_name=args.output_name,
            overwrite=args.overwrite,
        )

        result_counts[result] += 1

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

    # 1件でも失敗があれば、
    # シェルや自動処理側で失敗を検知できるよう終了コード1を返す
    return (
        1
        if result_counts["failed"] > 0
        else 0
    )


if __name__ == "__main__":
    # main()の戻り値をプロセスの終了コードとして返す
    raise SystemExit(main())