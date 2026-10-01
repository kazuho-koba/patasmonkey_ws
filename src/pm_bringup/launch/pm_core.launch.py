"""UGV制御・センサ・推定・perceptionのみ起動する。recorderは含めない。"""
# 具体的なNode構成・launch引数・起動順は ../pm_bringup/stack_launch.py に集約する。
# このファイルは共通実装を「記録なし」で呼び出す起動入口。
from pm_bringup.stack_launch import create_launch_description


def generate_launch_description():
    """既存の起動順と引数を共有し、記録プロセスを切り離す。"""
    return create_launch_description(legacy_recording=False)
