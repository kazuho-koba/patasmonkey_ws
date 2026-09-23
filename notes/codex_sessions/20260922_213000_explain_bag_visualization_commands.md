User prompt:
› URDFのカメラ高さ0.28 m
  これは、base_linkからのレンズ高さがそうということです。ここでいうbase_linkは車体のメインシャシを指していて、実際はシャシがタイヤによって最低地上高の分だけ浮いているため、実高さは実測していませんが大体0.4mくらいだと思います。
  実際にperceptionの様子をbagfile再生によって目視で確認できるのでしょうか？具体的なコマンドを教えてください。

Codex final response:
はい、目視確認できます。`base_link → レンズ` の0.28 mと、地面からレンズまで約0.4 mという理解で問題ありません。mapperは`odom → base_link`とURDFのstatic TFを合成するので、地面基準の実高は`base_link`のodom上zを含めて扱われます。

代表bagを使う場合、Foxyコンテナ内で次の3端末を開いてください。

端末1 — mapper（旧bag互換：CameraInfo fallbackとframe名補正を有効化）:

```bash
docker exec -it --user 1000:1000 --env HOME=/tmp patasmonkey_foxy_dev bash
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
source /workspaces/patasmonkey_ws/install/setup.bash
ros2 launch pm_perception terrain_mapping_old_bag.launch.py
```

端末2 — bag再生:

```bash
docker exec -it --user 1000:1000 --env HOME=/tmp patasmonkey_foxy_dev bash
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
source /workspaces/patasmonkey_ws/install/setup.bash

ros2 bag play /workspaces/patasmonkey_ws/bags/rosbag2_2026_08_21-23_04_01 \
  -s mcap \
  --topics /oak/depth/image_raw /tf /tf_static
```

端末3 — RViz2:

```bash
docker exec -it --user 1000:1000 --env HOME=/tmp patasmonkey_foxy_dev bash
source /opt/ros/foxy/setup.bash
source /workspaces/ros2_ws/install/setup.bash
source /workspaces/patasmonkey_ws/install/setup.bash

rviz2 -d "$(ros2 pkg prefix pm_perception)/share/pm_perception/rviz/terrain_mapping.rviz"
```

RVizではFixed Frameを`odom`にします。同梱設定には以下が入っています。

- `/depth_elevation_mapper/elevation_debug`: 高さを0–100へ色変換したOccupancyGrid
- `/depth_elevation_mapper/elevation_points_debug`: 実zを持つ、1 cellあたり1点のPointCloud2

確認したい「前方約2 mで観測した地面が、接近してFOV外になった後も残る」挙動は、主にPointCloud2で見ます。`odom`上に固定された点群が車両移動に追随して消えず、6×6 m windowの外へ出た時だけ消えるなら期待どおりです。mapperを先に起動し、bag再生開始後およそ10秒でdepthが現れます。

なお、今回の代表bagでは`/odometry/local`のzがVIO由来で大きく変動し得ます。したがって最初は「地図がodom上で時系列に連続しているか」「地面が不自然に回転・平行移動しないか」を見る検証として使い、絶対的な地面高さの評価はカメラ実装着高さとlocalizer zの妥当性確認後に行うのが安全です。
