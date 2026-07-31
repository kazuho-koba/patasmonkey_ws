#!/usr/bin/env python3

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import yaml
from mcap.reader import make_reader


DEFAULT_OUTPUT_FILENAME = "topic_frequency.csv"


@dataclass
class TopicStatistics:
    """Accumulated timestamp and message-count information for one topic."""

    topic_name: str
    message_count: int = 0
    first_timestamp_ns: Optional[int] = None
    last_timestamp_ns: Optional[int] = None

    def add_message(self, timestamp_ns: int) -> None:
        """Add one message timestamp to this topic's statistics."""
        self.message_count += 1

        if (
            self.first_timestamp_ns is None
            or timestamp_ns < self.first_timestamp_ns
        ):
            self.first_timestamp_ns = timestamp_ns

        if (
            self.last_timestamp_ns is None
            or timestamp_ns > self.last_timestamp_ns
        ):
            self.last_timestamp_ns = timestamp_ns

    @property
    def active_duration_sec(self) -> float:
        """Return elapsed time between first and last recorded messages."""
        if (
            self.first_timestamp_ns is None
            or self.last_timestamp_ns is None
            or self.last_timestamp_ns <= self.first_timestamp_ns
        ):
            return 0.0

        return (
            self.last_timestamp_ns - self.first_timestamp_ns
        ) * 1.0e-9

    @property
    def count_divided_by_duration_hz(self) -> Optional[float]:
        """
        Return count / active duration.

        This follows the requested definition. For a single message,
        active duration is zero, so no frequency can be calculated.
        """
        duration = self.active_duration_sec

        if duration <= 0.0:
            return None

        return self.message_count / duration

    @property
    def interval_frequency_hz(self) -> Optional[float]:
        """
        Return (count - 1) / active duration.

        This is the conventional average interval-based frequency.
        """
        duration = self.active_duration_sec

        if duration <= 0.0 or self.message_count < 2:
            return None

        return (self.message_count - 1) / duration


@dataclass
class BagMetadata:
    """Metadata required for one rosbag directory."""

    bag_directory: Path
    storage_identifier: str
    bag_duration_sec: float
    metadata_message_counts: Dict[str, int]
    relative_file_paths: List[str]


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Recursively analyze all rosbag2 directories below a bags root "
            "and write per-topic frequency statistics into each bag directory."
        )
    )

    parser.add_argument(
        "bags_root",
        type=Path,
        help=(
            "Root directory containing rosbag2 directories, "
            "for example /workspaces/patasmonkey_ws/bags"
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
    Read a nanoseconds value from rosbag2 metadata.

    Foxy-era metadata commonly represents duration as:
        duration:
          nanoseconds: 123
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
    """Load one rosbag2 metadata.yaml file."""
    with metadata_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)

    if not isinstance(document, dict):
        raise ValueError("metadata.yaml root is not a dictionary")

    information = document.get(
        "rosbag2_bagfile_information",
        document,
    )

    if not isinstance(information, dict):
        raise ValueError(
            "rosbag2_bagfile_information is missing or invalid"
        )

    storage_identifier = str(
        information.get("storage_identifier", "")
    )

    duration_ns = nested_nanoseconds(
        information.get("duration", {})
    )

    relative_file_paths = information.get(
        "relative_file_paths",
        [],
    )

    if not isinstance(relative_file_paths, list):
        raise ValueError("relative_file_paths is not a list")

    metadata_message_counts: Dict[str, int] = {}

    topics = information.get(
        "topics_with_message_count",
        [],
    )

    if isinstance(topics, list):
        for entry in topics:
            if not isinstance(entry, dict):
                continue

            topic_metadata = entry.get("topic_metadata", {})

            if not isinstance(topic_metadata, dict):
                continue

            topic_name = topic_metadata.get("name")

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
        bag_duration_sec=duration_ns * 1.0e-9,
        metadata_message_counts=metadata_message_counts,
        relative_file_paths=[
            str(relative_path)
            for relative_path in relative_file_paths
        ],
    )


def natural_mcap_sort_key(path: Path) -> Tuple[str, int]:
    """
    Sort split files numerically.

    For example:
        bag_2.mcap
        bag_10.mcap

    should be ordered as 2, 10 instead of 10, 2.
    """
    stem = path.stem
    prefix, separator, suffix = stem.rpartition("_")

    if separator and suffix.isdigit():
        return prefix, int(suffix)

    return stem, -1


def find_mcap_files(metadata: BagMetadata) -> List[Path]:
    """
    Resolve MCAP files belonging to one bag.

    Prefer relative_file_paths from metadata.yaml. If it is absent or
    incomplete, fall back to all *.mcap files in the bag directory.
    """
    files: List[Path] = []

    for relative_path in metadata.relative_file_paths:
        candidate = metadata.bag_directory / relative_path

        if candidate.suffix.lower() == ".mcap" and candidate.is_file():
            files.append(candidate)

    if not files:
        files = list(metadata.bag_directory.glob("*.mcap"))

    unique_files = sorted(
        set(files),
        key=natural_mcap_sort_key,
    )

    return unique_files


def read_mcap_statistics(
    mcap_files: Iterable[Path],
) -> Dict[str, TopicStatistics]:
    """Read all split MCAP files and aggregate timestamps by topic."""
    statistics: Dict[str, TopicStatistics] = {}

    for mcap_path in mcap_files:
        print(f"      Reading: {mcap_path.name}")

        with mcap_path.open("rb") as stream:
            reader = make_reader(stream)

            for _schema, channel, message in reader.iter_messages():
                topic_name = channel.topic

                topic_statistics = statistics.setdefault(
                    topic_name,
                    TopicStatistics(topic_name=topic_name),
                )

                # rosbag2 MCAP uses the MCAP log timestamp as the
                # rosbag record timestamp.
                topic_statistics.add_message(
                    int(message.log_time)
                )

    return statistics


def format_float(value: Optional[float], digits: int = 6) -> str:
    """Convert an optional float into a stable CSV representation."""
    if value is None:
        return ""

    return f"{value:.{digits}f}"


def timestamp_ns_to_sec(timestamp_ns: Optional[int]) -> str:
    """Convert an optional nanosecond timestamp into decimal seconds."""
    if timestamp_ns is None:
        return ""

    return f"{timestamp_ns * 1.0e-9:.9f}"


def write_csv(
    output_path: Path,
    metadata: BagMetadata,
    statistics: Dict[str, TopicStatistics],
) -> None:
    """Write one CSV result file atomically."""
    temporary_path = output_path.with_suffix(
        output_path.suffix + ".tmp"
    )

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
            topic_statistics = statistics.get(
                topic_name,
                TopicStatistics(topic_name=topic_name),
            )

            metadata_count = metadata.metadata_message_counts.get(
                topic_name,
                0,
            )

            active_duration_sec = (
                topic_statistics.active_duration_sec
            )

            if metadata.bag_duration_sec > 0.0:
                active_coverage_ratio = (
                    active_duration_sec
                    / metadata.bag_duration_sec
                )

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
                    "mcap_message_count": (
                        topic_statistics.message_count
                    ),
                    "metadata_message_count": metadata_count,
                    "message_count_difference": (
                        topic_statistics.message_count
                        - metadata_count
                    ),
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
                    "active_duration_sec": (
                        format_float(active_duration_sec, 9)
                    ),
                    "bag_duration_sec": (
                        format_float(
                            metadata.bag_duration_sec,
                            9,
                        )
                    ),
                    "active_coverage_ratio": (
                        format_float(
                            active_coverage_ratio,
                            6,
                        )
                    ),
                    "count_divided_by_active_duration_hz": (
                        format_float(
                            topic_statistics
                            .count_divided_by_duration_hz,
                            6,
                        )
                    ),
                    "interval_frequency_hz": (
                        format_float(
                            topic_statistics
                            .interval_frequency_hz,
                            6,
                        )
                    ),
                    "count_divided_by_bag_duration_hz": (
                        format_float(
                            count_divided_by_bag_duration_hz,
                            6,
                        )
                    ),
                }
            )

    temporary_path.replace(output_path)


def find_metadata_files(bags_root: Path) -> List[Path]:
    """Find all rosbag2 metadata.yaml files recursively."""
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
    Analyze one bag.

    Returns:
        "processed", "skipped", or "failed"
    """
    output_path = metadata_path.parent / output_name

    if output_path.exists() and not overwrite:
        print(
            f"[SKIP] {metadata_path.parent}: "
            f"{output_name} already exists"
        )
        return "skipped"

    print(f"[BAG]  {metadata_path.parent}")

    try:
        metadata = load_bag_metadata(metadata_path)

        if metadata.storage_identifier.lower() != "mcap":
            print(
                f"[FAIL] Unsupported storage identifier: "
                f"{metadata.storage_identifier!r}",
                file=sys.stderr,
            )
            return "failed"

        mcap_files = find_mcap_files(metadata)

        if not mcap_files:
            print(
                "[FAIL] No MCAP files were found",
                file=sys.stderr,
            )
            return "failed"

        statistics = read_mcap_statistics(mcap_files)

        if not statistics:
            print(
                "[FAIL] No messages were read from the MCAP files",
                file=sys.stderr,
            )
            return "failed"

        write_csv(
            output_path=output_path,
            metadata=metadata,
            statistics=statistics,
        )

    except Exception as error:
        print(
            f"[FAIL] {metadata_path.parent}: "
            f"{type(error).__name__}: {error}",
            file=sys.stderr,
        )
        return "failed"

    print(f"[OK]   Written: {output_path}")
    return "processed"


def main() -> int:
    args = parse_arguments()

    bags_root = args.bags_root.expanduser().resolve()

    if not bags_root.is_dir():
        print(
            f"Bags root does not exist or is not a directory: "
            f"{bags_root}",
            file=sys.stderr,
        )
        return 2

    metadata_files = find_metadata_files(bags_root)

    if not metadata_files:
        print(
            f"No metadata.yaml files found below: {bags_root}",
            file=sys.stderr,
        )
        return 1

    print(f"Bags root: {bags_root}")
    print(f"Found {len(metadata_files)} bag directories")

    result_counts = {
        "processed": 0,
        "skipped": 0,
        "failed": 0,
    }

    for metadata_path in metadata_files:
        result = analyze_one_bag(
            metadata_path=metadata_path,
            output_name=args.output_name,
            overwrite=args.overwrite,
        )

        result_counts[result] += 1

    print("")
    print("Summary")
    print(f"  Processed: {result_counts['processed']}")
    print(f"  Skipped:   {result_counts['skipped']}")
    print(f"  Failed:    {result_counts['failed']}")

    return 1 if result_counts["failed"] > 0 else 0


if __name__ == "__main__":
    raise SystemExit(main())