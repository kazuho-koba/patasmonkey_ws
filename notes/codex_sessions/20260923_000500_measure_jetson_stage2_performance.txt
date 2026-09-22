User prompt:
ありがとうございます。jetsonでは.yamlファイルをもとに戻してビルド済みです。sourceはしていません。
すなわち、max_mapが6.0、max_depthが4.0です。

jetsonにリモートログインして、評価まで実施してもらえますか？
ssh pmjet1でログインできます。

Codex final response:
Jetson AGX Xavier上で、現在install済みのStage 2を120秒間のオフラインbag再生で評価しました。車両制御や実機センサは起動していません。

確認した設定は、`map_size_x/y: 6.0`、`max_depth: 4.0`、`resolution: 0.05`、`pixel_stride: 4`です。9月21日の実走RGB-D bagを、記録済み`/tf`や`/odometry/local`を再生せず、最新localizationを再計算する経路へ流しました。

結果:

| 指標 | 実測 |
|---|---:|
| mapper 平均処理時間 | 約22 ms/frame（warm-up後） |
| mapper 最大処理時間 | 66.8 ms/frame（warm-up初回は131 ms） |
| 有効sample | 約14,700〜15,700 points/frame |
| 融合セル | 約420〜623 cells/frame |
| mapper CPU | 平均20.1%、最大53.0%（1 CPU core換算） |
| mapper RSS | 最大69 MiB |
| Jetson全体RAM | 最大2.57 GiB / 14.9 GiB |
| CPU温度 | 最大54.5 °C |
| GPU | GR3D 0%（現実装はCPU処理） |
| TF drop | 0（終了時の1 frameだけfuture extrapolationでdrop） |

10 Hz入力の1 frame予算は100 msです。warm-up後の約22 ms平均、約67 ms最大なので、現行Stage 2はJetson上でリアルタイム動作可能と判断できます。温度・メモリにも余裕があり、計測中にthermal throttlingは見られませんでした。

今回の測定は保守的です。`terrain_mapping_latest_localization_replay.launch.py`が旧bag用設定を重ねるため、`publish_debug_pointcloud: true`、Stage 2のdebug map publishも有効です。実機runtimeの既定設定ではPointCloud2はfalseなので、通常運用のcore mapper負荷は今回より軽くなります。

`rate_drops`は最終17、`tf_drops`は0でした。rate dropは12 Hz処理上限でのtimestamp間引きであり、過負荷によるTF待ち行列の破綻は観測されませんでした。終了時のfuture TF warningはbag player停止端の数ms差によるもので、通常区間の評価結果ではありません。

したがってPhase 3へ進んでよい状態です。ただし、slope/roughness/stepを入れた直後に同じ条件で再測定し、平均50 ms以下・最大100 ms未満・TF drop 0を当面の目標として比較するのがよいです。計測ログはJetsonの`/tmp/pm_perception_stage2_20260922_235927/`に残しています。
