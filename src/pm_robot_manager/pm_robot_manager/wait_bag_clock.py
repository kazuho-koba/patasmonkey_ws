"""boot時の未復元時計でbagを開始しないためのsystemd ExecStartPre。"""
import os
import subprocess
import time
from datetime import datetime


def wait_clock(timeout, minimum_year, now=datetime.now, monotonic=time.monotonic,
               sleep=time.sleep, synchronized=None):
    """NTPを秒単位で待ち、offline時は復元済みRTC/保存時計へfallbackする。

    minimum_yearは時計未復元の検出用で、日付の正確さを保証する閾値ではない。
    timeoutはwall clock補正の影響を受けないmonotonic秒で評価する。
    """
    if synchronized is None:
        def synchronized():
            try:
                result = subprocess.run(
                    ['timedatectl', 'show', '--property=NTPSynchronized', '--value'],
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                    universal_newlines=True, timeout=2, check=False)
                return result.returncode == 0 and result.stdout.strip() == 'yes'
            except (OSError, subprocess.TimeoutExpired):
                return False
    deadline = monotonic() + timeout
    print('bag: システム時計の復元とNTP同期を待機します', flush=True)
    while True:
        current = now()
        valid = current.year >= minimum_year
        if valid and synchronized():
            print('bag: 時刻同期済み: '+current.isoformat(), flush=True)
            return 0
        if monotonic() >= deadline:
            if valid:
                # Internetなしでも走行記録を残す。復元時計の誤差はログに明示する。
                print('bag: 警告: NTP未同期。復元済み時計を使用: '+current.isoformat(),
                      flush=True)
                return 0
            print('bag: 時計未復元のため録画を開始しません: '+current.isoformat(),
                  flush=True)
            return 1
        sleep(1)


def main():
    """systemd環境変数で待機秒数と未復元判定年を変更できる。"""
    timeout = float(os.environ.get('PM_BAG_CLOCK_WAIT_SEC', '45'))
    minimum_year = int(os.environ.get('PM_BAG_CLOCK_MIN_YEAR', '2020'))
    if timeout < 0 or minimum_year < 1970:
        raise ValueError('時刻確認の待機秒数または判定年が不正です')
    raise SystemExit(wait_clock(timeout, minimum_year))
