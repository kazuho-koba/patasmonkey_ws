"""旧一括launchの互換入口。topic・引数・従来の記録動作を維持する。"""
# 具体的なNode構成・launch引数・起動順・従来の記録処理は
# ../pm_bringup/stack_launch.py に集約し、この入口では旧一括記録を有効にする。
from pm_bringup.stack_launch import create_launch_description


def generate_launch_description():
    """従来のセンサ/制御/推定と標準bagを同時起動する。"""
    return create_launch_description(legacy_recording=True)
