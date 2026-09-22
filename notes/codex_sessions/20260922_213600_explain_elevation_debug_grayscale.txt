User prompt:
/depth_elevation_mapper/elevation_debugというトピックが2.5D mapに相当するものでしょうか？
走行の過程でrviz2上での色が黒に近い濃い灰色〜白に近い灰色で幅広く変化しています。
これは走行コスト（現在の暫定評価関数に基づく推定）が大きく変化しているということですよね？
ほぼ平坦地を走っているので、カメラと路面の相対位置はそう大きくは変わっていないのですが、オドメトリ推定のzがずれたこと等によって変動するものでしょうか？

Codex final response:
`/depth_elevation_mapper/elevation_debug`は、内部のrolling 2.5D elevation gridを確認するための`nav_msgs/OccupancyGrid`です。ただし、mapの全layerを表す正式なAPIではなく、RViz用のdebug投影です。

濃淡は走行コストでもtraversabilityでもありません。現在は各cellの推定ground elevation `z`を、parameterの次の固定範囲で0–100へ線形変換した値です。

```text
debug_elevation_min = -0.5 m  -> 0   （黒寄り）
debug_elevation_max =  0.5 m  -> 100 （白寄り）
```

未観測cellは`-1`で、RVizでは別の灰色として表示されます。observation count、variance、obstacle confidence、走行コストはこのtopicには含めていません。Nav2 costmapにもまだ接続していません。

したがって、ほぼ平坦な地面でも次の理由で濃淡が動きます。

- `odom → base_link`のzがずれると、camera→odomへ変換される全depth点のzもほぼ同じ量だけ上下します。例えばodometry zが+0.10 mずれれば、平面路面のelevation表示も概ね+0.10 m白側へ移動します。
- roll/pitchの推定誤差・揺れは、depth画像をodomへ投影する角度を変えるため、地図に勾配や波状の変化として現れます。
- stereo depthの距離誤差、4×4 sampling、cellごとの最小z ground仮説も局所的な変化要因です。
- rolling windowでは、新しく入ったcellは観測直後の1サンプルであり、複数frameを融合済みのcellより色が変化しやすいです。

このbagのlocalizationでは特にzがVIO由来で、絶対高さとして強く拘束されていません。そのため、今の表示で絶対zを評価するのは適していません。一方で、同じodom位置に観測が時系列で残るか、roll/pitch誤差で地面が不自然に傾かないかを確認する用途には使えます。

切り分けには、RVizで`Elevation points debug`を併用し、再生topicに`/odometry/local`も加えてz・roll・pitchを確認してください。

```bash
ros2 bag play /workspaces/patasmonkey_ws/bags/rosbag2_2026_07_26-09_17_38 \
  -s mcap --topics /oak/depth/image_raw /oak/color/image_raw /tf /tf_static /odometry/local

ros2 topic echo /odometry/local
```

Stage 2では、elevationの絶対値表示と、slope/roughness/stepから作るtraversability/cost表示を別topic・別colormapに分けるのが適切です。
