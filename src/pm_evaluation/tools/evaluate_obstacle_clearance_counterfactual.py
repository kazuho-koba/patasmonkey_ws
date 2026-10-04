#!/usr/bin/env python3
"""障害物解除診断の旧コマンドを保つ互換入口。

低高さ幅・2画素以上の解除方式は空間保持評価の既定方式に採用済み。
処理を二重実装せず同じmainへ委譲する。旧方式との比較には
--legacy-obstacle-retentionを指定する。通常perceptionは変更しない。
"""
from evaluate_spatial_feature_coverage import main


if __name__ == "__main__":
    main()
