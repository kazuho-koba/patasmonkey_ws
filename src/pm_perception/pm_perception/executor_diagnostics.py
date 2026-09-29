"""
Mapper executorの実行区間とLinux scheduler待ちを記録・集計する診断ツール.

計測executorはFoxyのSingleThreadedExecutorと同じ順序でready entityを1件選び、
同じspin thread上でhandlerを完了させる。通常動作では選択されず、診断parameterが
明示的に有効な場合だけ使用する。
"""

import argparse
import csv
import math
import os
from pathlib import Path
import statistics
import sys
import threading
import time

from rclpy.executors import (
    ShutdownException,
    SingleThreadedExecutor,
    TimeoutException,
)


EXECUTOR_FIELDS = [
    "record_type",
    "monotonic_ns",
    "thread_id",
    "pid",
    "node_name",
    "entity_type",
    "entity_name",
    "executor_wait_ms",
    "ready_to_dispatch_ms",
    "handler_wall_ms",
    "handler_thread_cpu_ms",
    "handler_non_cpu_ms",
]

SCHEDSTAT_FIELDS = [
    "sample_monotonic_ns",
    "interval_wall_ns",
    "pid",
    "tid",
    "thread_name",
    "runtime_delta_ns",
    "runqueue_wait_delta_ns",
    "timeslices_delta",
    "runtime_one_core_percent",
    "runqueue_wait_percent",
]


def _percentile(values, percentile):
    """NumPyに依存せず、診断値のnearest-rank percentileを返す."""
    if not values:
        return 0.0
    ordered = sorted(values)
    index = max(int(math.ceil(len(ordered) * percentile)) - 1, 0)
    return float(ordered[index])


def _entity_label(entity):
    """ROS entityをsubscription topic名またはtimer callback名へ変換する."""
    if entity is None:
        return "Task", "executor_task"

    entity_type = type(entity).__name__
    topic_name = getattr(entity, "topic_name", None)
    if topic_name:
        return entity_type, str(topic_name)

    callback = getattr(entity, "callback", None)
    if callback is not None:
        callback_name = getattr(callback, "__qualname__", None)
        if not callback_name:
            callback_name = getattr(callback, "__name__", type(callback).__name__)
        return entity_type, str(callback_name)

    return entity_type, entity_type


class MeasuredSingleThreadedExecutor(SingleThreadedExecutor):
    """
    Executorのready entity、dispatch gap、callback wall/CPU時間をCSVへ記録する.

    `wait_for_ready_callbacks()`からready entityが返った時刻と、handler呼び出し直前を
    記録する。ここで得るready-to-dispatch時間はDDS publish時刻からのend-to-end遅延では
    なく、Foxy executorがready entityを選択してからhandlerを呼ぶ直前までの短い区間。
    DDS reader内でsampleがreadyになる前の時間は直接は測らない。

    callbackごとのwall時間とexecutor threadのCPU時間を併記し、差分をscheduler待ち等を
    含む非CPU時間の参考値として残す。Linuxの`/proc/.../schedstat`は別のmonitor subcommand
    で採取し、threadがrunnableのままCPU割り当てを待った時間と照合する。
    """

    def __init__(self, node, output_path, flush_period_sec=1.0):
        # mapperと同じROS contextを使う。FoxyのExecutorが内部guard conditionを作る
        # 前にcontextを渡さないと、独立Contextでのsmoke testや複数context環境で失敗する。
        super().__init__(context=node.context)
        self._node_name = str(node.get_name())
        self._flush_period_sec = max(float(flush_period_sec), 0.1)
        self._pending_rows = []
        self._last_flush_ns = time.monotonic_ns()
        self._thread_cpu_clock = getattr(time, "thread_time_ns", None)
        self._stream = None
        self._writer = None

        path = Path(output_path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        # 既存結果を上書きしない。試験ごとに異なるファイル名を使う。
        self._stream = path.open("x", encoding="utf-8", newline="", buffering=262144)
        self._writer = csv.DictWriter(self._stream, fieldnames=EXECUTOR_FIELDS)
        self._writer.writeheader()
        self._writer.writerow({
            "record_type": "metadata",
            "monotonic_ns": time.monotonic_ns(),
            "thread_id": threading.get_native_id(),
            "pid": os.getpid(),
            "node_name": self._node_name,
        })
        self._stream.flush()
        node.get_logger().warning(
            "executor timing diagnostics ENABLED; csv=%s pid=%d tid=%d; "
            "diagnostic-only SingleThreadedExecutor wrapper"
            % (str(path), os.getpid(), threading.get_native_id())
        )

    def _append_event(self, row, now_ns):
        """callbackごとの計測結果を蓄積し、低頻度でまとめて書き出す."""
        self._pending_rows.append(row)
        if (now_ns - self._last_flush_ns) >= int(self._flush_period_sec * 1e9):
            self._flush_rows()
            self._last_flush_ns = now_ns

    def _flush_rows(self):
        """CSVのwrite/flushをcallbackごとでなく短いbatch単位にまとめる."""
        if self._writer is None or not self._pending_rows:
            return
        self._writer.writerows(self._pending_rows)
        self._pending_rows.clear()
        self._stream.flush()

    def spin_once(self, timeout_sec=None):
        """Foxy SingleThreadedExecutorと同じ順で1 entityを選び、処理時間を測る."""
        wait_started_ns = time.monotonic_ns()
        try:
            handler, entity, node = self.wait_for_ready_callbacks(
                timeout_sec=timeout_sec
            )
        except ShutdownException:
            return
        except TimeoutException:
            return

        # 公開executor APIからready entityが返った直後と、handler呼び出し直前を記録する。
        # 差分にはPython dispatch overheadも含み、RMW内のready時刻そのものではない。
        ready_return_ns = time.monotonic_ns()
        entity_type, entity_name = _entity_label(entity)
        dispatch_start_ns = time.monotonic_ns()
        cpu_start_ns = (
            self._thread_cpu_clock() if self._thread_cpu_clock is not None else None
        )
        try:
            handler()
        finally:
            handler_end_ns = time.monotonic_ns()
            cpu_end_ns = (
                self._thread_cpu_clock()
                if self._thread_cpu_clock is not None else None
            )
            wall_ns = handler_end_ns - dispatch_start_ns
            cpu_ns = (
                max(cpu_end_ns - cpu_start_ns, 0)
                if cpu_start_ns is not None and cpu_end_ns is not None else None
            )
            self._append_event({
                "record_type": "callback",
                "monotonic_ns": dispatch_start_ns,
                "thread_id": threading.get_native_id(),
                "pid": os.getpid(),
                "node_name": node.get_name() if node is not None else self._node_name,
                "entity_type": entity_type,
                "entity_name": entity_name,
                "executor_wait_ms": (ready_return_ns - wait_started_ns) / 1e6,
                "ready_to_dispatch_ms": (dispatch_start_ns - ready_return_ns) / 1e6,
                "handler_wall_ms": wall_ns / 1e6,
                "handler_thread_cpu_ms": "" if cpu_ns is None else cpu_ns / 1e6,
                "handler_non_cpu_ms": (
                    "" if cpu_ns is None else max(wall_ns - cpu_ns, 0) / 1e6
                ),
            }, handler_end_ns)

        if handler.exception() is not None:
            raise handler.exception()

    def close(self):
        """終了時に残りの計測行をflushし、ファイルを閉じる."""
        if self._stream is not None:
            self._flush_rows()
            self._stream.close()
            self._stream = None
            self._writer = None


def _read_proc_cmdline(pid):
    """PIDのcmdlineを読み、消滅・権限エラー時は空文字を返す."""
    try:
        return Path("/proc", str(pid), "cmdline").read_bytes().replace(
            b"\0", b" "
        ).decode("utf-8", errors="replace")
    except (OSError, ValueError):
        return ""


def _find_process(process_match):
    """`/proc`から指定文字列を含むmapper processを1つだけ探す."""
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid == os.getpid():
            continue
        command_line = _read_proc_cmdline(pid)
        # 既定名ではcmdline中の引数文字列だけに一致するmonitor自身やshellを除き、
        # 実際にそのentry pointを実行しているPython processだけを採る。
        if process_match == "depth_elevation_mapper_node":
            argv = command_line.split()
            try:
                executable_name = Path("/proc", str(pid), "exe").resolve().name
            except OSError:
                executable_name = ""
            is_mapper_entry = any(
                Path(token).name == process_match and os.path.sep in token
                for token in argv
            ) and executable_name.startswith("python")
        else:
            is_mapper_entry = process_match in command_line
        if is_mapper_entry:
            found.append((pid, command_line))
    if len(found) > 1:
        raise RuntimeError(
            "複数のprocessがmatchしました。--pidで対象PIDを明示してください: %s"
            % ", ".join(str(item[0]) for item in found)
        )
    return found[0] if found else None


def _read_schedstat(path):
    """Linux schedstatの実行時間・runqueue待ち・timeslice数を読む."""
    try:
        fields = path.read_text(encoding="ascii").split()
        if len(fields) < 3:
            return None
        return int(fields[0]), int(fields[1]), int(fields[2])
    except (OSError, ValueError):
        return None


def monitor_schedstat(args):
    """Mapper各threadのschedstat累積値を一定周期でdelta CSVへ保存する."""
    output_path = Path(args.output).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    interval_sec = max(args.interval_sec, 0.1)
    print("mapper processの起動を待っています: %s" % args.process_match, flush=True)

    try:
        with output_path.open(
            "x", encoding="utf-8", newline="", buffering=65536
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=SCHEDSTAT_FIELDS)
            writer.writeheader()
            stream.flush()
            process = None
            wait_deadline = time.monotonic() + args.wait_timeout_sec
            while process is None:
                if args.pid is not None:
                    cmdline = _read_proc_cmdline(args.pid)
                    process = (args.pid, cmdline) if cmdline else None
                else:
                    process = _find_process(args.process_match)
                if process is None:
                    if time.monotonic() >= wait_deadline:
                        raise TimeoutError("mapper processの起動待ちがtimeoutしました")
                    time.sleep(0.2)

            pid, command_line = process
            print("monitor開始: pid=%d command=%s" % (pid, command_line), flush=True)
            previous = {}
            previous_sample_ns = time.monotonic_ns()
            start_ns = previous_sample_ns
            next_sample = time.monotonic()
            while True:
                sample_ns = time.monotonic_ns()
                if args.duration_sec > 0 and (sample_ns - start_ns) >= int(
                    args.duration_sec * 1e9
                ):
                    break
                if not Path("/proc", str(pid)).exists():
                    print("mapper process終了を検出しました: pid=%d" % pid, flush=True)
                    break

                task_root = Path("/proc", str(pid), "task")
                try:
                    tasks = list(task_root.iterdir())
                except OSError:
                    # PID確認直後にmapperが終了した場合は通常の終了として扱う。
                    print("mapper process終了を検出しました: pid=%d" % pid, flush=True)
                    break
                current_tids = set()
                interval_wall_ns = max(sample_ns - previous_sample_ns, 1)
                for task in tasks:
                    if not task.name.isdigit():
                        continue
                    tid = int(task.name)
                    current_tids.add(tid)
                    values = _read_schedstat(task / "schedstat")
                    if values is None:
                        continue
                    runtime_ns, runqueue_ns, slices = values
                    prior = previous.get(tid)
                    if prior is None:
                        runtime_delta = runqueue_delta = slices_delta = ""
                        runtime_pct = runqueue_pct = ""
                    else:
                        runtime_delta = max(runtime_ns - prior[0], 0)
                        runqueue_delta = max(runqueue_ns - prior[1], 0)
                        slices_delta = max(slices - prior[2], 0)
                        runtime_pct = 100.0 * runtime_delta / interval_wall_ns
                        runqueue_pct = 100.0 * runqueue_delta / interval_wall_ns
                    try:
                        thread_name = (task / "comm").read_text(
                            encoding="utf-8", errors="replace"
                        ).strip()
                    except OSError:
                        thread_name = ""
                    writer.writerow({
                        "sample_monotonic_ns": sample_ns,
                        "interval_wall_ns": interval_wall_ns,
                        "pid": pid,
                        "tid": tid,
                        "thread_name": thread_name,
                        "runtime_delta_ns": runtime_delta,
                        "runqueue_wait_delta_ns": runqueue_delta,
                        "timeslices_delta": slices_delta,
                        "runtime_one_core_percent": runtime_pct,
                        "runqueue_wait_percent": runqueue_pct,
                    })
                    previous[tid] = values
                # 既に終了したthreadの累積値を次のthread生成へ引き継がない。
                previous = {tid: values for tid, values in previous.items()
                            if tid in current_tids}
                stream.flush()
                previous_sample_ns = sample_ns
                next_sample += interval_sec
                time.sleep(max(next_sample - time.monotonic(), 0.0))
    except FileExistsError:
        raise RuntimeError("出力ファイルが既にあります。別名を指定してください: %s" % output_path)

    print("schedstat CSV: %s" % output_path)


def _load_executor_csv(path):
    """Executor CSVを読み、metadataとcallback eventへ分ける."""
    metadata = {}
    events = []
    with Path(path).open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            if row.get("record_type") == "metadata":
                metadata = row
            elif row.get("record_type") == "callback":
                events.append(row)
    return metadata, events


def summarize_diagnostics(args):
    """Executor CSVと任意のschedstat CSVをentity/thread単位で要約する."""
    metadata, events = _load_executor_csv(args.executor_csv)
    schedstat_rows = []
    schedstat_pids = set()
    schedstat_start_ns = None
    schedstat_end_ns = None
    if args.schedstat_csv:
        with Path(args.schedstat_csv).open(encoding="utf-8", newline="") as stream:
            for row in csv.DictReader(stream):
                schedstat_rows.append(row)
                if row.get("pid"):
                    schedstat_pids.add(row["pid"])
                if row.get("sample_monotonic_ns"):
                    sample_ns = int(row["sample_monotonic_ns"])
                    schedstat_start_ns = (
                        sample_ns if schedstat_start_ns is None
                        else min(schedstat_start_ns, sample_ns)
                    )
                    schedstat_end_ns = (
                        sample_ns if schedstat_end_ns is None
                        else max(schedstat_end_ns, sample_ns)
                    )
        executor_pid = metadata.get("pid", "")
        if schedstat_pids and executor_pid not in schedstat_pids:
            raise RuntimeError(
                "executor CSVのPID %sとschedstat CSVのPID %sが一致しません。"
                "同じ試行のCSVを指定してください。"
                % (executor_pid, ", ".join(sorted(schedstat_pids)))
            )
        # monitorをVO初期化後など、比較対象区間の開始時に起動した場合、そのsample範囲を
        # mapper CSVからの切り出し窓として使う。これにより初期化・jerk前の時間を除ける。
        if schedstat_start_ns is not None and schedstat_end_ns is not None:
            events = [
                row for row in events
                if schedstat_start_ns <= int(row["monotonic_ns"]) <= schedstat_end_ns
            ]
    if not events:
        raise RuntimeError(
            "指定窓にexecutor callback eventがありません。CSVのPIDと記録窓を確認してください"
        )
    start_ns = min(int(row["monotonic_ns"]) for row in events)
    end_ns = max(
        int(row["monotonic_ns"])
        + int(float(row["handler_wall_ms"]) * 1e6)
        for row in events
    )
    window_sec = max((end_ns - start_ns) / 1e9, 1e-9)

    groups = {}
    for row in events:
        key = (row["entity_type"], row["entity_name"])
        group = groups.setdefault(key, {
            "count": 0, "wait": [], "wall": [], "cpu": [], "non_cpu": [],
            "dispatch": [],
        })
        group["count"] += 1
        group["wait"].append(float(row["executor_wait_ms"]))
        group["wall"].append(float(row["handler_wall_ms"]))
        if row.get("handler_thread_cpu_ms"):
            group["cpu"].append(float(row["handler_thread_cpu_ms"]))
        if row.get("handler_non_cpu_ms"):
            group["non_cpu"].append(float(row["handler_non_cpu_ms"]))
        group["dispatch"].append(float(row["ready_to_dispatch_ms"]))

    print("mapper executor diagnostics")
    print("  pid/tid: %s/%s; node: %s" % (
        metadata.get("pid", "?"), metadata.get("thread_id", "?"),
        metadata.get("node_name", "?"),
    ))
    print("  event数=%d、計測窓=%.3f秒" % (len(events), window_sec))
    if schedstat_start_ns is not None and schedstat_end_ns is not None:
        print("  schedstat照合window: monotonic_ns %d .. %d "
              "（executor eventもこの範囲へ限定）"
              % (schedstat_start_ns, schedstat_end_ns))
    print("  entity type / name | rate Hz | executor wait mean/p95 ms | "
          "dispatch gap mean/p95/max ms | wall mean/p95/max ms | "
          "CPU/non-CPU mean ms | wall occupancy")
    for (entity_type, entity_name), group in sorted(
        groups.items(), key=lambda item: sum(item[1]["wall"]), reverse=True
    ):
        wall = group["wall"]
        wait = group["wait"]
        cpu = group["cpu"]
        non_cpu = group["non_cpu"]
        occupancy = 100.0 * sum(wall) / (window_sec * 1000.0)
        dispatch = group["dispatch"]
        print("  %s / %s | %.2f | %.3f/%.3f | %.3f/%.3f/%.3f | "
              "%.3f/%.3f/%.3f | %.3f/%.3f | %.2f%%"
              % (
                  entity_type, entity_name, group["count"] / window_sec,
                  statistics.mean(wait), _percentile(wait, 0.95),
                  statistics.mean(dispatch), _percentile(dispatch, 0.95),
                  max(dispatch),
                  statistics.mean(wall), _percentile(wall, 0.95), max(wall),
                  statistics.mean(cpu) if cpu else 0.0,
                  statistics.mean(non_cpu) if non_cpu else 0.0,
                  occupancy,
              ))

    if not args.schedstat_csv:
        return
    executor_tid = metadata.get("thread_id", "")
    per_tid = {}
    for row in schedstat_rows:
        if not row.get("runtime_delta_ns"):
            continue
        tid = row["tid"]
        totals = per_tid.setdefault(tid, {
            "runtime": 0, "runqueue": 0, "wall": 0, "name": row["thread_name"],
        })
        totals["runtime"] += int(row["runtime_delta_ns"])
        totals["runqueue"] += int(row["runqueue_wait_delta_ns"])
        totals["wall"] += int(row["interval_wall_ns"])
    if executor_tid in per_tid:
        values = per_tid[executor_tid]
        wall = max(values["wall"], 1)
        print("  executor thread schedstat: TID=%s (%s), CPU=%.2f%% of one core, "
              "runqueue wait=%.2f%% of wall window"
              % (
                  executor_tid, values["name"],
                  100.0 * values["runtime"] / wall,
                  100.0 * values["runqueue"] / wall,
              ))
    else:
        print("  executor TIDのschedstatがありません。collectorをmapper起動中に"
              "実行したか、PID/TIDと計測窓を確認してください。")
    process_runtime = sum(row["runtime"] for row in per_tid.values())
    process_wall = max((
        row["wall"] for row in per_tid.values()
    ), default=1)
    print("  観測thread合計CPU=%.2f%% of one core（各threadのruntime合計／単一窓）"
          % (100.0 * process_runtime / max(process_wall, 1)))


def main(args=None):
    """Linux thread samplerとexecutor CSV summarizerのCLI entry point."""
    parser = argparse.ArgumentParser(
        description="mapper executor callback時間とLinux schedstatを調べる"
    )
    subparsers = parser.add_subparsers(dest="command")

    monitor = subparsers.add_parser(
        "monitor", help="/proc schedstatをthread単位で記録する"
    )
    monitor.add_argument("--output", required=True, help="新規作成するschedstat CSV")
    monitor.add_argument(
        "--process-match", default="depth_elevation_mapper_node",
        help="/proc cmdlineから対象processを探す文字列",
    )
    monitor.add_argument("--pid", type=int, help="自動検索の代わりにPIDを指定")
    monitor.add_argument("--interval-sec", type=float, default=1.0)
    monitor.add_argument("--duration-sec", type=float, default=0.0,
                         help="0ならprocess終了またはCtrl-Cまで継続")
    monitor.add_argument("--wait-timeout-sec", type=float, default=180.0)

    summary = subparsers.add_parser(
        "summarize", help="executor CSVとschedstat CSVを要約する"
    )
    summary.add_argument("--executor-csv", required=True)
    summary.add_argument("--schedstat-csv")

    parsed = parser.parse_args(args)
    if parsed.command == "monitor":
        monitor_schedstat(parsed)
    elif parsed.command == "summarize":
        summarize_diagnostics(parsed)
    else:
        parser.print_help()
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
