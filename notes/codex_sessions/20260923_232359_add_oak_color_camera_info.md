User prompt:
ありがとうございます。まず、/oak/color/camera_infoを出力するようにdepthai_driverを修正することはできますか？
公式ドキュメントの確認、開発PCにもkalibr_imucam_chain_oakds2_19443010E122001300.yamlに対応するカメラが刺さっているので、開発PC側で本当に意味があるデータが取れるかも含めて確認して、問題なさそうなら修正してください。

Codex final response:
`depthai_driver`を修正し、`/oak/color/camera_info`を追加しました。開発PCのOAK-Dを実際に読み取り専用で調べたところ、接続IDは`19443010E122001300`で、ご指定のKalibrファイルの個体と一致しました。EEPROMからRGBと左右monoの校正値を取得でき、実機起動後には新topicから640×400のCameraInfoを受信できることも確認しました。

RGB EEPROM値から得たcolor previewの内部行列は、`fx=516.590, fy=573.989, cx=311.252, cy=192.830`です。歪み係数もCameraInfoの`D`へ設定しています。Kalibrファイルの左右mono intrinsicsも、実機EEPROMの左右mono値と概ね近く、校正値が対象機から有用に取得できることを裏付けています。ただし、これは値の取得と整合性の確認であり、チェッカーボード等による再投影精度の独立検証まではしていません。

実装ではDepthAIのRGB EEPROM校正値を利用し、color画像のpreview設定（640×400、aspect比を保たないstretch）に合わせて`keepAspectRatio=False`でKを計算しています。既存のdepth CameraInfoやdepth画像の挙動は変更していません。DepthAI公式資料でも、RGB-depth alignmentでは出力サイズの指定ができ、intrinsicsは出力画像サイズ・crop条件に合わせて取得するAPIが案内されています。[RGB-depth alignment](https://docs.luxonis.com/software/depthai/examples/rgb_depth_aligned)、[Camera intrinsics API](https://docs.luxonis.com/software-v3/depthai/api/cpp)

画像が両方640×400でも、現在のcolor previewはstretch、depth出力はaspect比を保つ既定resize/cropなので、それぞれのKは異なります。画素単位で色とdepthを重ねる場合は、後段で画像変換geometryを揃えるか、共通のカメラモデルへ再投影してください。既存depth出力への影響を避けるため、今回そこは変更していません。

今後のbagで利用できるよう、`pm_bag_global_localization.launch.py`など3つの標準記録topicに追加し、`bag_clock_player`でも再生可能にしました。主な変更は[driver node](/home/kazuho/ros2_ws/src/depthai_driver/depthai_driver/oakd_vio_rgbd_node.py)、[RGB-D説明](/home/kazuho/ros2_ws/src/depthai_driver/README_patasmonkey.md)、記録launch 3件、[bag_clock_player.py](/home/kazuho/Projects/patasmonkey_ws/src/pm_evaluation/pm_evaluation/cli/bag_clock_player.py)、[pm_perception README](/home/kazuho/Projects/patasmonkey_ws/src/pm_perception/README.md)です。

Foxyコンテナで`depthai_driver`と`pm_bringup`のビルドは成功し、`pm_evaluation`も隔離した`/tmp`のbuild/install先でビルド成功しました。実機topicのpublish/subscribe smoke testも成功しています。既存build先のroot所有install artifactが原因で通常の`pm_evaluation`再buildは失敗し、`depthai_driver`のcolcon testはUID 1000でUSB機器を開けずtest収集中に失敗しました。実機topic確認は別途完了しており、ソースのdiff checkは通っています。

Foxyで値を見るには次を実行し、確認後`Ctrl-C`で終了してください（Foxyの`ros2 topic echo`には`--once` optionがありません）。

```bash
ros2 topic echo /oak/color/camera_info sensor_msgs/msg/CameraInfo
```
