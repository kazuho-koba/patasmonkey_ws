"""実際のmapper入力変換を保存する、明示有効化専用の軽量JSONL診断。

点列・画像は保存せず、元bagの画像stampと使用TFを結び付ける。ログにない
画像を評価側が黙って追加しないことで、TF待ち・rate gate後の入力列を再利用する。
"""
import json
from pathlib import Path


def transform_record(transform):
    """geometry_msgs Transformをm単位translationとxyzw quaternionにする。"""
    t, q = transform.translation, transform.rotation
    return {"translation": [t.x, t.y, t.z], "quaternion": [q.x, q.y, q.z, q.w]}


class MapperReplayTrace:
    def __init__(self, directory, parameters):
        folder = Path(directory)
        folder.mkdir(parents=True, exist_ok=True)
        # 誤って同じ出力先を指定しても既存の比較証拠を上書きしない。
        self.stream = (folder/"mapper_trace.jsonl").open("x", buffering=1)
        self.fusion_count = 0
        self.last_image_stamp_ns = None
        self.write({"type": "metadata", "schema": 1, "parameters": parameters})

    def write(self, record):
        self.stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False)+"\n")

    def fusion(self, stamp, frame, intrinsics, width, height, camera_tf, base_tf):
        """grid更新後に呼び、処理順・使用した変換そのものを保存する。"""
        self.fusion_count += 1
        self.last_image_stamp_ns = int(stamp)
        self.write({"type": "fusion", "fusion_index": self.fusion_count,
                    "image_stamp_ns": int(stamp), "image_frame": frame,
                    "width": int(width), "height": int(height),
                    "intrinsics": list(intrinsics),
                    "camera_to_map": transform_record(camera_tf.transform),
                    "base_to_map": transform_record(base_tf.transform),
                    "map_frame": camera_tf.header.frame_id,
                    "camera_frame": camera_tf.child_frame_id,
                    "base_frame": base_tf.child_frame_id})

    def snapshot(self, stamp):
        """評価時刻とそのgridが何枚目まで融合済みかを対応付ける。"""
        self.write({"type": "snapshot", "evaluation_stamp_ns": int(stamp),
                    "fusion_count": self.fusion_count,
                    "last_image_stamp_ns": self.last_image_stamp_ns})

    def close(self):
        if not self.stream.closed:
            self.write({"type": "complete", "fusion_count": self.fusion_count})
            self.stream.close()


def load_mapper_trace(path):
    """未完了・重複stamp・逆順ログは拒否し、欠落を黙って補間しない。"""
    records = [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
    if not records or records[0].get("schema") != 1 or records[-1].get("type") != "complete":
        raise ValueError("未完了または未対応のmapper trace")
    fusions, snapshots = {}, []
    for record in records:
        if record["type"] == "fusion":
            stamp = record["image_stamp_ns"]
            if stamp in fusions or (fusions and stamp <= next(reversed(fusions))):
                raise ValueError("重複または逆順の画像stamp")
            if record["fusion_index"] != len(fusions)+1:
                raise ValueError("fusion indexが不連続")
            fusions[stamp] = record
        elif record["type"] == "snapshot":
            if record["fusion_count"] != len(fusions) or record["last_image_stamp_ns"] != (next(reversed(fusions)) if fusions else None):
                raise ValueError("snapshotとfusion順序が不整合")
            snapshots.append(record)
    if records[-1]["fusion_count"] != len(fusions):
        raise ValueError("完了件数が不一致")
    if not fusions:
        raise ValueError("採用画像0件のtraceは評価に使えません")
    return records[0], fusions, snapshots
