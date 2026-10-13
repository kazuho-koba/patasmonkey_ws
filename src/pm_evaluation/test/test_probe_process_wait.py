"""非侵襲thread診断が名前・待ち先・schedstatを取得することを検証する。"""
import importlib.util
from pathlib import Path
import os

spec = importlib.util.spec_from_file_location('wait_probe', Path(__file__).parents[1]/'tools/probe_process_wait.py')
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


def test_own_process_threads_have_cpu_counters():
    result = probe.thread_snapshot(Path('/proc')/str(os.getpid()))
    assert str(os.getpid()) in result
    main = result[str(os.getpid())]
    assert main['name']
    assert len(main['schedstat'].split()) == 3


def test_thread_disappearance_is_explicit(tmp_path):
    (tmp_path/'task'/'123').mkdir(parents=True)
    assert 'error' in probe.thread_snapshot(tmp_path)['123']


def test_write_syscall_fd_is_a_candidate_not_an_inferred_lock(tmp_path):
    """arm64のwrite候補fdを記録し、lockの原因は断定しない。"""
    task = tmp_path/'task'/'123'
    task.mkdir(parents=True)
    for name, value in [('comm', 'pm_bag_writer'), ('wchan', 'rwsem_down_write_slowpath'),
                        ('schedstat', '1 2 3'), ('syscall', '64 0x9 0x1000 0x100000')]:
        (task/name).write_text(value)
    (tmp_path/'fd').mkdir()
    (tmp_path/'fd'/'9').symlink_to('/tmp/test_bag_0.mcap')
    result = probe.thread_snapshot(tmp_path, syscalls=True)['123']
    assert result['syscall'].startswith('64 ')
    assert result['first_argument_fd_target'] == '/tmp/test_bag_0.mcap'
    assert 'syscall' not in probe.thread_snapshot(tmp_path)['123']
