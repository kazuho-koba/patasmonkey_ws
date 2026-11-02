"""ローカル/遠隔で共用する単一解析ジョブ。bag本文は通信しない。"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import uuid

from pm_evaluation.bag_inspection import atomic_json


class Jobs:
    """許可root配下の停止済みbagを逐次解析し、結果を原本外へ保存する。"""
    def __init__(self, root, results):
        self.root = Path(root).expanduser().resolve()
        self.results = Path(results).expanduser().resolve()
        self.lock = threading.RLock()
        self.thread = None
        self.child = None
        self.cancel = threading.Event()
        self.state = dict(state='IDLE', entries=[], current='', error='')

    def resolve(self, value):
        path = Path(value).expanduser()
        if not path.is_absolute():
            path = self.root/path
        path = path.resolve()
        path.relative_to(self.root)
        return path

    def dispatch(self, operation, payload):
        """要求は固定の操作だけに限定し、shellや任意コマンドは受け付けない。"""
        with self.lock:
            if operation == 'list':
                root = self.resolve(payload.get('root') or str(self.root))
                # UIが既知のbagを指定でき、1階層下の集約directoryも選択できる。
                if not root.is_dir():
                    raise ValueError('directoryがありません: '+str(root))
                children = sorted(root.iterdir())
                offset = max(0, int(payload.get('offset', 0)))
                if len(children) > 10000:
                    raise ValueError('directoryが大きすぎます。対象を集約したdirectoryを指定してください')
                page = children[offset:offset+200]
                bags, directories = [], []
                if (root/'metadata.yaml').is_file() and offset == 0:
                    bags.append(str(root))
                for path in page:
                    if path.is_dir():
                        try:
                            path = self.resolve(str(path))
                        except ValueError:
                            continue
                        (bags if (path/'metadata.yaml').is_file() else directories).append(str(path))
                return dict(root=str(root), bags=bags, directories=directories,
                            next_offset=offset+200 if offset+200 < len(children) else None)
            if operation == 'status':
                answer = json.loads(json.dumps(self.state))
                progress = self.state.get('progress_file')
                if progress:
                    try:
                        answer['progress'] = json.loads(Path(progress).read_text())
                    except (OSError, ValueError):
                        pass
                return answer
            if operation == 'history':
                # 原本を探索せず、結果root直下の完了/中止ジョブだけを列挙する。
                paths = sorted(self.results.glob('*/job.json'), reverse=True)[:100]
                return dict(jobs=[path.parent.name for path in paths])
            if operation == 'load':
                if self.thread and self.thread.is_alive():
                    raise ValueError('実行中の解析が終わってから過去結果を開いてください')
                name = payload['job']
                if Path(name).name != name:
                    raise ValueError('job名が不正です')
                path = (self.results/name).resolve()
                path.relative_to(self.results)
                restored = json.loads((path/'job.json').read_text())
                for entry in restored['entries']:
                    Path(entry['output']).resolve().relative_to(path)
                if restored['state'] == 'RUNNING':
                    restored['state'] = 'ERROR'
                    restored['error'] = '未完了ジョブの履歴です。自動再開は行いません'
                self.state = restored
                return dict(message='過去の結果を開きました')
            if operation == 'cancel':
                self.cancel.set()
                if self.child and self.child.poll() is None:
                    self.child.send_signal(signal.SIGTERM)
                return dict(message='中止を要求しました')
            if operation == 'result':
                index = int(payload.get('index', 0))
                entry = self.state['entries'][index]
                if entry['state'] != 'DONE':
                    raise ValueError('解析結果はまだありません')
                report = json.loads((Path(entry['output'])/'report.json').read_text())
                topic = payload.get('topic')
                if topic:
                    return dict(topic=topic, statistics=report['topics'][topic],
                                window_sec=report['window_sec'], duration_sec=report['duration_sec'])
                for row in report['topics'].values():
                    row.pop('window_hz', None)
                return report
            if operation != 'start':
                raise ValueError('未知の操作')
            if self.thread and self.thread.is_alive():
                raise ValueError('解析は既に実行中です')
            bags = list(dict.fromkeys(str(self.resolve(p)) for p in payload['bags']))
            if not 1 <= len(bags) <= 100:
                raise ValueError('1〜100個のbagを選択してください')
            window = float(payload.get('window_sec', 10.))
            if not math.isfinite(window) or window <= 0:
                raise ValueError('時間窓は正の有限値が必要です')
            for bag in bags:
                if not (Path(bag)/'metadata.yaml').is_file():
                    raise ValueError('metadataがありません: '+bag)
            job = self.results/(time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
            job.mkdir(parents=True, exist_ok=False)
            if any(job == Path(b) or Path(b) in job.parents for b in bags):
                raise ValueError('結果directoryはbag原本の外に設定してください')
            self.cancel.clear()
            self.state = dict(state='RUNNING', current='', error='', job_directory=str(job),
                              entries=[dict(bag=b, output=str(job/str(i)), state='QUEUED')
                                       for i, b in enumerate(bags)])
            atomic_json(job/'job.json', self.state)
            self.thread = threading.Thread(target=self._run, args=(window,), daemon=True)
            self.thread.start()
            return dict(message='解析を受け付けました', job_directory=str(job))

    def _run(self, window):
        """重い処理は低優先度の子processへ分離し、中止時は今回の子だけ終了する。"""
        try:
            for entry in self.state['entries']:
                with self.lock:
                    if self.cancel.is_set():
                        break
                    entry['state'] = 'RUNNING'
                    self.state['current'] = entry['bag']
                    progress = Path(entry['output']+'.progress.json')
                    self.state['progress_file'] = str(progress)
                    log = Path(entry['output']+'.log')
                    command = [sys.executable, '-m', 'pm_evaluation.bag_inspection',
                               '--bag', entry['bag'], '--output', entry['output'],
                               '--window-sec', str(window), '--progress', str(progress)]
                    # niceは解析workerにだけ適用し、Qt/ROS受付の優先度を変更しない。
                    with log.open('w') as stream:
                        self.child = subprocess.Popen(['nice', '-n', '10', *command],
                            stdout=stream, stderr=subprocess.STDOUT)
                code = self.child.wait()
                with self.lock:
                    if self.cancel.is_set():
                        entry['state'] = 'CANCELLED'
                        break
                    entry['state'] = 'DONE' if code == 0 else 'ERROR'
                    if code:
                        entry['error'] = log.read_text(errors='replace')[-3000:]
                    atomic_json(Path(self.state['job_directory'])/'job.json', self.state)
            with self.lock:
                for entry in self.state['entries']:
                    if entry['state'] == 'QUEUED':
                        entry['state'] = 'CANCELLED'
                self.state['state'] = ('CANCELLED' if self.cancel.is_set() else
                                       'ERROR' if any(e['state']=='ERROR' for e in self.state['entries']) else 'DONE')
                atomic_json(Path(self.state['job_directory'])/'job.json', self.state)
        except Exception as error:
            with self.lock:
                self.state.update(state='ERROR', error=str(error))

    def close(self):
        """受付終了時に今回の解析子processを残さない。原本は変更しない。"""
        self.dispatch('cancel', {})
        if self.thread:
            self.thread.join(timeout=10.)
