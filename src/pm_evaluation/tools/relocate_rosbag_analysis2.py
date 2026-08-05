#!/usr/bin/env python3
"""
ROS 2 MCAP rosbagディレクトリから解析生成物を分離する。

rosbagディレクトリの判定:
  - 直下に metadata.yaml がある
  - 直下に1個以上の *.mcap がある

rosbagデータとして残すもの:
  - metadata.yaml
  - 直下の *.mcap

それ以外のファイル、ディレクトリ、シンボリックリンクは、
  <bags_root>/analysis/<rosbagディレクトリ名>/
へ移動する。

安全設計:
  - デフォルトはdry-run。実際に移動するには --apply が必要
  - 既存ファイルを上書きしない
  - 同名の移動先がある場合は __moved_2 などの連番を付ける
  - analysisディレクトリ以下はrosbag探索対象から除外する
  - rosbagディレクトリ自体やmetadata.yaml、MCAPは削除しない

使用例:
  # 何が移動されるか確認
  python3 relocate_rosbag_analysis.py /workspaces/patasmonkey_ws/bags

  # 実際に移動
  python3 relocate_rosbag_analysis.py /workspaces/patasmonkey_ws/bags --apply
"""

from __future__ import annotations

import argparse
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence


DEFAULT_ROOT = Path("/workspaces/patasmonkey_ws/bags")
METADATA_NAME = "metadata.yaml"
MCAP_SUFFIX = ".mcap"


@dataclass
class MovePlan:
    bag_dir: Path
    source: Path
    destination: Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Move non-rosbag analysis products out of MCAP rosbag "
            "directories into a central analysis directory."
        )
    )
    parser.add_argument(
        "bags_root",
        nargs="?",
        type=Path,
        default=DEFAULT_ROOT,
        help=(
            "Root directory searched recursively for rosbag directories. "
            f"Default: {DEFAULT_ROOT}"
        ),
    )
    parser.add_argument(
        "--analysis-dir",
        type=Path,
        default=None,
        help="Destination root. Default: <bags_root>/analysis",
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help=(
            "Actually create directories and move files. "
            "Without this option, only a dry-run plan is printed."
        ),
    )
    parser.add_argument(
        "--include-hidden",
        action="store_true",
        help=(
            "Also move hidden entries such as .notes. "
            "By default hidden entries are reported but left untouched."
        ),
    )
    return parser.parse_args()


def resolve_paths(args: argparse.Namespace) -> tuple[Path, Path]:
    root = args.bags_root.expanduser().resolve()

    if not root.exists():
        raise FileNotFoundError(f"bags root does not exist: {root}")
    if not root.is_dir():
        raise NotADirectoryError(f"bags root is not a directory: {root}")

    analysis_dir = (
        root / "analysis"
        if args.analysis_dir is None
        else args.analysis_dir.expanduser().resolve()
    )

    if analysis_dir == root:
        raise ValueError("analysis directory must not be the bags root itself")

    return root, analysis_dir


def is_relative_to(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def is_rosbag_directory(path: Path) -> bool:
    if not path.is_dir():
        return False

    if not (path / METADATA_NAME).is_file():
        return False

    return any(
        child.is_file() and child.suffix.lower() == MCAP_SUFFIX
        for child in path.iterdir()
    )


def find_rosbag_directories(root: Path, analysis_dir: Path) -> List[Path]:
    result: List[Path] = []

    for metadata in root.rglob(METADATA_NAME):
        bag_dir = metadata.parent.resolve()

        if is_relative_to(bag_dir, analysis_dir):
            continue

        if is_rosbag_directory(bag_dir):
            result.append(bag_dir)

    return sorted(set(result), key=lambda path: str(path))


def is_rosbag_data_entry(entry: Path) -> bool:
    if entry.name == METADATA_NAME:
        return True

    if entry.is_file() and entry.suffix.lower() == MCAP_SUFFIX:
        return True

    return False


def sanitize_origin(root: Path, bag_dir: Path) -> str:
    try:
        relative_parent = bag_dir.parent.relative_to(root)
        text = "__".join(relative_parent.parts)
    except ValueError:
        text = bag_dir.parent.name

    if not text or text == ".":
        return "root"

    return "".join(
        char if char.isalnum() or char in "-_" else "_"
        for char in text
    )


def unique_destination(desired: Path, root: Path, bag_dir: Path) -> Path:
    if not desired.exists() and not desired.is_symlink():
        return desired

    origin = sanitize_origin(root, bag_dir)

    if desired.is_dir() or not desired.suffix:
        stem = desired.name
        suffix = ""
    else:
        stem = desired.stem
        suffix = desired.suffix

    counter = 2
    while True:
        candidate = desired.with_name(
            f"{stem}__from_{origin}__moved_{counter}{suffix}"
        )
        if not candidate.exists() and not candidate.is_symlink():
            return candidate
        counter += 1


def build_move_plans(
    root: Path,
    analysis_dir: Path,
    bag_dirs: Sequence[Path],
    include_hidden: bool,
) -> tuple[List[MovePlan], List[Path]]:
    plans: List[MovePlan] = []
    skipped_hidden: List[Path] = []

    for bag_dir in bag_dirs:
        destination_dir = analysis_dir / bag_dir.name

        for entry in sorted(bag_dir.iterdir(), key=lambda path: path.name):
            if is_rosbag_data_entry(entry):
                continue

            if entry.name.startswith(".") and not include_hidden:
                skipped_hidden.append(entry)
                continue

            desired = destination_dir / entry.name
            destination = unique_destination(desired, root, bag_dir)
            plans.append(MovePlan(bag_dir, entry, destination))

    return plans, skipped_hidden


def detect_duplicate_bag_names(
    bag_dirs: Sequence[Path],
) -> dict[str, List[Path]]:
    by_name: dict[str, List[Path]] = {}

    for bag_dir in bag_dirs:
        by_name.setdefault(bag_dir.name, []).append(bag_dir)

    return {
        name: paths
        for name, paths in by_name.items()
        if len(paths) > 1
    }


def print_plan(
    root: Path,
    analysis_dir: Path,
    bag_dirs: Sequence[Path],
    plans: Sequence[MovePlan],
    skipped_hidden: Sequence[Path],
    apply: bool,
) -> None:
    mode = "APPLY" if apply else "DRY-RUN"

    print("=" * 88)
    print(f"ROS bag analysis relocation [{mode}]")
    print("=" * 88)
    print(f"Bags root             : {root}")
    print(f"Analysis root         : {analysis_dir}")
    print(f"Rosbag directories    : {len(bag_dirs)}")
    print(f"Entries to move       : {len(plans)}")
    print("")

    current_bag: Optional[Path] = None

    for plan in plans:
        if plan.bag_dir != current_bag:
            current_bag = plan.bag_dir
            print(f"[BAG] {current_bag}")

        print(f"  MOVE: {plan.source.name}")
        print(f"     -> {plan.destination}")

    if skipped_hidden:
        print("")
        print(
            f"[SKIP] {len(skipped_hidden)} hidden entries "
            "(use --include-hidden to move)"
        )
        for path in skipped_hidden:
            print(f"  {path}")

    if not plans:
        print("No non-rosbag analysis entries require moving.")


def apply_plans(
    root: Path,
    analysis_dir: Path,
    bag_dirs: Sequence[Path],
    plans: Sequence[MovePlan],
) -> None:
    analysis_dir.mkdir(parents=True, exist_ok=True)

    # 解析ファイルが無いbagについても、要求どおり同名ディレクトリを作る。
    for bag_dir in bag_dirs:
        (analysis_dir / bag_dir.name).mkdir(parents=True, exist_ok=True)

    moved = 0

    for plan in plans:
        plan.destination.parent.mkdir(parents=True, exist_ok=True)

        destination = plan.destination
        if destination.exists() or destination.is_symlink():
            destination = unique_destination(destination, root, plan.bag_dir)

        shutil.move(str(plan.source), str(destination))
        moved += 1

    print("")
    print(f"[DONE] Moved {moved} entries.")


def main() -> int:
    args = parse_args()

    try:
        root, analysis_dir = resolve_paths(args)
        bag_dirs = find_rosbag_directories(root, analysis_dir)

        duplicates = detect_duplicate_bag_names(bag_dirs)
        if duplicates:
            print(
                "[WARNING] Duplicate rosbag directory names were found. "
                "Their analysis outputs share the same destination directory:",
                file=sys.stderr,
            )
            for name, paths in duplicates.items():
                print(f"  {name}", file=sys.stderr)
                for path in paths:
                    print(f"    - {path}", file=sys.stderr)

        plans, skipped_hidden = build_move_plans(
            root=root,
            analysis_dir=analysis_dir,
            bag_dirs=bag_dirs,
            include_hidden=args.include_hidden,
        )

        print_plan(
            root=root,
            analysis_dir=analysis_dir,
            bag_dirs=bag_dirs,
            plans=plans,
            skipped_hidden=skipped_hidden,
            apply=args.apply,
        )

        if args.apply:
            apply_plans(root, analysis_dir, bag_dirs, plans)
        else:
            print("")
            print(
                "[DRY-RUN] No files were changed. "
                "Run again with --apply to perform the moves."
            )

    except KeyboardInterrupt:
        print("\n[INTERRUPTED]", file=sys.stderr)
        return 130
    except Exception as error:
        print(
            f"[ERROR] {type(error).__name__}: {error}",
            file=sys.stderr,
        )
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
