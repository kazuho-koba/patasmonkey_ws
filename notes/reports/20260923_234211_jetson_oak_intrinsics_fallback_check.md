# OAK-D S2 intrinsics と mapper fallback の照合

## 結論

7月bagのhazard検証では、bagに `/oak/depth/camera_info` が記録されていなかったため、mapperの設定ファイルにある静的intrinsics fallbackが使われていた。今回Jetson上のOAK-D S2（MX ID `19443010C10EF91200`）からDepthAI SDKで読み出したRGB EEPROM intrinsicsは、そのfallbackの4値と一致した。したがって、ユーザーの判断どおり、このintrinsics照合を理由にしたhazard bag再生のやり直しは行っていない。

## 前回検証での値の入手経路

前回のbag再生時にライブカメラやbag内のCameraInfoから値を取得したのではない。7月bagには `/oak/depth/camera_info` がなく、`depth_elevation_mapper.yaml` に記載された静的値を使った。設定コメントには、OAK-D S2の対象シリアル番号について保存した640×400 RGB EEPROM intrinsicsと記録されている。

mapperはまず受信済み `CameraInfo` のwidth/heightがdepth imageと一致するか確認し、一致すれば `K` の `fx, fy, cx, cy` を使う。CameraInfoがない、または寸法が合わず、fallbackが有効かつ設定解像度が画像と一致するときだけfallbackを使う。どちらも使えなければ、その画像はintrinsics不足として処理できない。

「fallback」は、通常は画像に対応したCameraInfoを優先し、それがない古いbag等のために用意した静的な代替値、という意味である。実行中にカメラへ問い合わせて補完する機能ではない。

リポジトリの設定コメントとgit履歴から、fallback値が「EEPROMから保存された値」として導入されたことは分かる。一方、当初それを読み出した具体的なコマンドや記録は履歴に残っていないため、過去の取得手順そのものまでは断定できない。

## 今回の読み出しと照合

Jetsonホスト上でDepthAI SDK 2.30.0.0を用い、対象デバイスのEEPROM calibrationを直接読み出した。画像ストリーム/pipelineは開始せず、車両を動かす操作もしていない。

RGB camera、640×400、`keepAspectRatio=True` のEEPROM Kは次のとおり。

| 項目 | EEPROM読み出し値 | mapper fallback |
|---|---:|---:|
| fx | 574.2882690429688 | 574.28826904 |
| fy | 574.2882690429688 | 574.28826904 |
| cx | 354.7508544921875 | 354.75085449 |
| cy | 215.2326202392578 | 215.23262024 |

4値とも設定値と丸め誤差の範囲で一致する。EEPROMには別geometry（`keepAspectRatio=False`）のKもあり、そちらはfx=516.8594971、fy=574.2882690、cx=351.2757568、cy=215.2326202。今回一致を確認したのはmapper fallbackと同じ`keepAspectRatio=True`のgeometryである。

## 適用範囲と制約

- 前回のmapperが使ったのは上記4つのK値で、歪み係数Dはback-projectionに適用していない。よって今回の一致確認は、当時の処理が利用したintrinsicsについての確認であり、歪み補正込みのcamera model全体を検証したものではない。
- Kalibr YAMLの左右モノカメラintrinsicsは別streamの校正値であり、今回照合したRGB EEPROM Kと同一の値・用途ではない。前回のterrain replayでKalibr YAMLを直接読み込んだわけではない。
- mapperの実行時選択と条件は `src/pm_perception/pm_perception/depth_elevation_mapper_node.py` の `intrinsics_for()`、値は `src/pm_perception/config/depth_elevation_mapper.yaml` にある。
- この照合でintrinsics由来の差が完全に否定されたわけではない。特にdistortion適用、depth/RGB alignment geometry、外部姿勢やframe仮定は別の論点である。

## 参照

- 7月bagのCameraInfo不在とfallback使用の記録: `notes/reports/20260923_july_hazard_cause_hypothesis_tests.md` のH6
- mapper fallback設定: `src/pm_perception/config/depth_elevation_mapper.yaml`
- CameraInfo優先・fallback選択: `src/pm_perception/pm_perception/depth_elevation_mapper_node.py`
