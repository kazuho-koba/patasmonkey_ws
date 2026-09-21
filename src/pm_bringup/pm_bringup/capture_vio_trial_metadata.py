#!/usr/bin/env python3

"""Capture reproducibility metadata beside a VIO validation rosbag."""

import argparse
import datetime
import hashlib
import json
import platform
import re
import shutil
import socket
import subprocess
from pathlib import Path


def sha256(path):
    """Return the SHA-256 digest of a file."""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_snapshot(path, label, output_dir):
    """Save revision, status and tracked diff for a Git checkout."""
    try:
        revision = subprocess.check_output(
            ["git", "-C", str(path), "rev-parse", "HEAD"],
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
        branch = subprocess.check_output(
            ["git", "-C", str(path), "branch", "--show-current"],
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
        status = subprocess.check_output(
            ["git", "-C", str(path), "status", "--porcelain"],
            stderr=subprocess.DEVNULL,
            text=True,
        )
        diff = subprocess.check_output(
            ["git", "-C", str(path), "diff", "--binary"],
            stderr=subprocess.DEVNULL,
            text=True,
        )
        (output_dir / (label + "_git_status.txt")).write_text(
            status, encoding="utf-8"
        )
        (output_dir / (label + "_git_diff.patch")).write_text(
            diff, encoding="utf-8"
        )
        return {
            "revision": revision,
            "branch": branch,
            "dirty": bool(status.strip()),
        }
    except (OSError, subprocess.CalledProcessError):
        return None


def main():
    """Copy effective config inputs and write a trial manifest."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--openvins-config", required=True)
    parser.add_argument("--workspace", required=True)
    parser.add_argument("--openvins-source", required=True)
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    config = Path(args.openvins_config).resolve()
    config_snapshot = output_dir / "config_snapshot"
    config_snapshot.mkdir(parents=True, exist_ok=True)

    # Relative calibration files are resolved beside estimator_config*.yaml.
    # Copy only the selected estimator and the files it actually references.
    config_text = config.read_text(encoding="utf-8")
    referenced_names = re.findall(
        r'^\s*relative_config_(?:imu|imucam):\s*["\']?([^"\'#\s]+)',
        config_text,
        flags=re.MULTILINE,
    )
    sources = [config]
    sources.extend(config.parent / name for name in referenced_names)
    copied = []
    for source in sources:
        if not source.is_file():
            raise FileNotFoundError(
                "Referenced OpenVINS config does not exist: {}".format(source)
            )
        destination = config_snapshot / source.name
        shutil.copy2(str(source), str(destination))
        copied.append({
            "name": source.name,
            "source": str(source),
            "sha256": sha256(source),
        })

    now = datetime.datetime.now(datetime.timezone.utc)
    manifest = {
        "captured_at_utc": now.isoformat(),
        "hostname": socket.gethostname(),
        "platform": platform.platform(),
        "openvins_config": str(config),
        "config_files": copied,
        "patasmonkey_git": git_snapshot(
            Path(args.workspace), "patasmonkey", output_dir
        ),
        "openvins_git": git_snapshot(
            Path(args.openvins_source), "openvins", output_dir
        ),
    }
    manifest_path = output_dir / "trial_manifest.json"
    temporary_path = manifest_path.with_suffix(".json.tmp")
    temporary_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    temporary_path.replace(manifest_path)


if __name__ == "__main__":
    main()
