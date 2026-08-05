#!/usr/bin/env python3
"""Move non-rosbag analysis outputs out of rosbag directories.

Directory layout is preserved below <root>/analysis.
Example:
  <root>/old/rosbag2_xxx/topic_frequency.csv
    -> <root>/analysis/old/rosbag2_xxx/topic_frequency.csv

By default, the script runs in dry-run mode. Actual filesystem changes are
performed only when ``--execute`` is specified.
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
from pathlib import Path, PurePosixPath

try:
    import yaml
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "PyYAML が必要です。Ubuntu/ROS では通常インストール済みです。\n"
        "未導入の場合: sudo apt install python3-yaml"
    ) from exc


DEFAULT_ROOT = Path("/workspaces/patasmonkey_ws/bags")

# metadata.yaml からデータファイル名を取得できなかった場合の保険。
FALLBACK_BAG_SUFFIXES = (
    ".mcap",
    ".mcap.zstd",
    ".db3",
    ".db3.zstd",
    ".sqlite3",
    ".sqlite3.zstd",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "rosbag ディレクトリ内の解析結果を <root>/analysis 以下へ移動し、"
            "元のディレクトリ構造を維持します。既定では dry-run です。"
        )
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=DEFAULT_ROOT,
        help=f"bags ルートディレクトリ (既定: {DEFAULT_ROOT})",
    )
    parser.add_argument(
        "--analysis-dir-name",
        default="analysis",
        help="解析結果格納ディレクトリ名 (既定: analysis)",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="移動先に同名ファイル/ディレクトリがあれば置換する",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="実際にディレクトリ作成・ファイル移動を実行する",
    )
    return parser.parse_args()


def is_relative_to(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def find_rosbag_dirs(root: Path, analysis_root: Path) -> list[Path]:
    """Find directories containing metadata.yaml, without following symlinks."""
    rosbag_dirs: list[Path] = []

    for current, dirnames, filenames in os.walk(root, topdown=True, followlinks=False):
        current_path = Path(current)

        # analysis 以下とシンボリックリンク先は探索対象外。
        dirnames[:] = [
            name
            for name in dirnames
            if not is_relative_to((current_path / name).resolve(strict=False), analysis_root)
            and not (current_path / name).is_symlink()
        ]

        if current_path == analysis_root or is_relative_to(current_path, analysis_root):
            dirnames[:] = []
            continue

        if "metadata.yaml" in filenames:
            rosbag_dirs.append(current_path)
            # rosbag 内部に別の rosbag があるケースは通常ないため、内部は再探索しない。
            dirnames[:] = []

    return sorted(rosbag_dirs)


def read_relative_file_paths(metadata_path: Path) -> set[PurePosixPath]:
    """Read rosbag storage paths from metadata.yaml."""
    try:
        with metadata_path.open("r", encoding="utf-8") as stream:
            metadata = yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError) as exc:
        print(f"[WARN] metadata.yaml を読めません: {metadata_path}: {exc}")
        return set()

    info = metadata.get("rosbag2_bagfile_information", {})
    paths = info.get("relative_file_paths", [])
    if not isinstance(paths, list):
        return set()

    result: set[PurePosixPath] = set()
    for value in paths:
        if not isinstance(value, str) or not value.strip():
            continue
        rel = PurePosixPath(value)
        # metadata に異常な絶対パスや親参照があっても root 外を保護対象にしない。
        if rel.is_absolute() or ".." in rel.parts:
            print(f"[WARN] 不正な relative_file_paths を無視します: {value}")
            continue
        result.add(rel)
    return result


def is_protected_entry(
    entry: Path,
    bag_dir: Path,
    metadata_files: set[PurePosixPath],
) -> bool:
    """Return True for metadata and rosbag storage files."""
    rel = PurePosixPath(entry.relative_to(bag_dir).as_posix())

    if rel == PurePosixPath("metadata.yaml"):
        return True

    # metadata に列挙されたデータファイルそのもの、またはその親ディレクトリを保護。
    for storage_rel in metadata_files:
        if rel == storage_rel:
            return True
        if len(rel.parts) < len(storage_rel.parts) and storage_rel.parts[: len(rel.parts)] == rel.parts:
            return True

    # 古い/一部欠損 metadata へのフォールバック。
    if entry.is_file() and entry.name.endswith(FALLBACK_BAG_SUFFIXES):
        return True

    return False


def remove_destination(path: Path, dry_run: bool) -> None:
    if dry_run:
        print(f"  [DRY] remove existing: {path}")
        return

    if path.is_symlink() or path.is_file():
        path.unlink()
    elif path.is_dir():
        shutil.rmtree(path)


def move_entry(source: Path, destination: Path, overwrite: bool, dry_run: bool) -> bool:
    if destination.exists() or destination.is_symlink():
        if not overwrite:
            print(f"  [SKIP] 移動先が既に存在: {destination}")
            return False
        remove_destination(destination, dry_run)

    action = "DRY-MOVE" if dry_run else "MOVE"
    print(f"  [{action}] {source} -> {destination}")
    if not dry_run:
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(source), str(destination))
    return True


def main() -> int:
    args = parse_args()
    dry_run = not args.execute
    root = args.root.expanduser().resolve()
    analysis_root = root / args.analysis_dir_name

    if not root.is_dir():
        print(f"[ERROR] root ディレクトリが存在しません: {root}", file=sys.stderr)
        return 1

    analysis_dir_name = Path(args.analysis_dir_name)
    if (
        analysis_dir_name.is_absolute()
        or len(analysis_dir_name.parts) != 1
        or args.analysis_dir_name in {"", ".", ".."}
    ):
        print("[ERROR] --analysis-dir-name は単一のディレクトリ名にしてください", file=sys.stderr)
        return 2

    print(f"Root     : {root}")
    print(f"Analysis : {analysis_root}")
    print(f"Mode     : {'DRY-RUN' if dry_run else 'EXECUTE'}")

    if dry_run:
        if not analysis_root.exists():
            print(f"[DRY] mkdir -p {analysis_root}")
    else:
        analysis_root.mkdir(parents=True, exist_ok=True)

    rosbag_dirs = find_rosbag_dirs(root, analysis_root)
    print(f"Found    : {len(rosbag_dirs)} rosbag director{'y' if len(rosbag_dirs) == 1 else 'ies'}")

    moved_count = 0
    skipped_count = 0

    for bag_dir in rosbag_dirs:
        relative_bag_dir = bag_dir.relative_to(root)
        destination_dir = analysis_root / relative_bag_dir
        print(f"\n[BAG] {relative_bag_dir}")

        if dry_run:
            if not destination_dir.exists():
                print(f"  [DRY] mkdir -p {destination_dir}")
        else:
            destination_dir.mkdir(parents=True, exist_ok=True)

        metadata_files = read_relative_file_paths(bag_dir / "metadata.yaml")

        # 先に一覧を固定し、移動中のディレクトリ内容変化の影響を避ける。
        entries = sorted(bag_dir.iterdir(), key=lambda path: path.name)
        for entry in entries:
            if is_protected_entry(entry, bag_dir, metadata_files):
                continue

            destination = destination_dir / entry.name
            if move_entry(entry, destination, args.overwrite, dry_run):
                moved_count += 1
            else:
                skipped_count += 1

    print("\n=== Summary ===")
    print(f"rosbag directories : {len(rosbag_dirs)}")
    print(f"moved entries      : {moved_count}")
    print(f"skipped conflicts  : {skipped_count}")

    if dry_run:
        print("\n[INFO] 現在は dry-run モードです。実際に移動するには --execute オプションを付けて実行してください。")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
