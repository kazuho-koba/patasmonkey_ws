"""必須観測・状態・実行条件を記録する。制御系の起動/停止は独立。"""
# 具体的なlaunch引数・記録プロセスの起動処理は ../pm_bringup/bag_launch.py、
# 記録・実行条件保存の本体は ../pm_bringup/record_trial.py に集約する。
# 必須topic一覧は ../config/bag_profiles.yaml の mission を参照する。
from pm_bringup.bag_launch import create_bag_launch


def generate_launch_description():
    """上位topicと再現metadataを同じbagディレクトリに記録する。"""
    return create_bag_launch('mission')
