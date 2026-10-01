"""任意の派生表示を記録する。入力の再生にはmission bagも必要。"""
# 具体的なlaunch引数・記録プロセスの起動処理は ../pm_bringup/bag_launch.py、
# 記録の本体は ../pm_bringup/record_trial.py に集約する。
# 任意topic一覧は ../config/bag_profiles.yaml の debug を参照する。
from pm_bringup.bag_launch import create_bag_launch


def generate_launch_description():
    """下位topicだけを別bagへ記録する。"""
    return create_bag_launch('debug')
