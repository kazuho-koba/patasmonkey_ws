# 実行時VIO adapter

`ros2 run pm_localization vio_odom_adapter_node` は、OpenVINSのIMU原点・IMU軸の
Odometryを車体原点・base_link軸へ変換します。OpenVINSノードそのものは変更しません。
poseはodom、twistはbase_link成分で出し、位置・姿勢・速度・共分散に同じ取付変換を使います。
詳細な式は[共分散監査の説明](README_vio_adapter_audit.md)を参照してください。

## 通常の起動

`pm_core.launch.py` 等が共通で使う `stack_launch.py` は新しいadapterを起動します。
既存の `/ov_msckf/odomimu` → `/vio/odometry` と型・通常frameを維持します。
TFのodom→base_link配信はEKF/composerの担当です。

- `mount_rotation_source=parameter`：`mount_rotation` の９要素を実効R_base_imuに使用。
  既定は `(base_x,base_y,base_z)=(-imu_z,-imu_y,-imu_x)`。小さな実機操作で支持された
  仮の回転であり、厳密な校正ではありません。TFの平行移動は引き続き使用します。
- `mount_rotation_source=tf`：TFの回転と平行移動を両方使用。URDFを校正したらこちらを
  選択できます。parameter回転はTFへ追加乗算しないので、二重適用しません。
- `pose_covariance_orientation_frame=imu`：現在のOpenVINS出力共分散の微小角はIMU軸。
  上流で既にglobal固定軸へ変換した場合だけ `global` へ変更します。
- `align_initial_to_tf=true`：最初の入力header時刻に近い `/odometry/local` の位置とyawへ
  一度だけ整列。そのposeはodom→base_link TFの発行元と同じ座標です。roll/pitchは
  VIOの重力方向と取付回転から得て、初期yaw整列で世界座標を傾けません。
- 初回参照の時刻差は `reference_time_tolerance_sec=0.1` 以下が必要。localの履歴は
  最大300件で有界です。整列後にlocalへ継続的に追従することはありません。
- GNSSを使う通常launchでは `require_heading_ready=true` とし、heading_initializerの
  READYを待ちます。GNSSを無効にした単体センサ起動ではfalseになります。
  heading初期化が失敗・未完了なら、VIO adapterも整列を待つことに注意してください。

## 独立したVIOの試験

local基準がないbag・センサ単体試験では、独立した世界座標を明示します。

```bash
ros2 run pm_localization vio_odom_adapter_node --ros-args \
  -p align_initial_to_tf:=false -p output_frame_id:=vio_odom \
  -p require_heading_ready:=false
```

`zero_initial_pose=true` なら最初の車体位置とyawをゼロへ合わせ、roll/pitchは保ちます。
falseならOpenVINS世界座標のままです。独立座標をodomと同名で公開することは拒否します。

## 入力検査と診断

非有限・無効quaternion、予期しないframe、非対称/負分散の共分散はpublishせず、
`/vio/adapter/diagnostics` に理由と受信・転送・棄却・待機件数を１秒程度で記録します。
実bagで非対称入力が見つかっているため、小さい非対称量を黙って対称化しません。
検査の許容範囲は数学関数内に明記しています。入力の無受信はstale診断になります。
通常の詳細consoleはOFFです。mission bagはこの診断topicも記録します。

重複・小さい順序逆転は棄却します。１秒を超えるheader逆行、あるいは整列後に
heading READYが解除された場合はreset_requiredに入り、新しい世界座標へ黙って
再配置しません。原因を確認してadapterを再起動してください。これはVIOの発散や
位置ジャンプ全般を検出する機能ではなく、既存のVIOゲートも引き続き必要です。

## 共分散の範囲

poseの位置・微小角と交差項、およびtwistの線速度・角速度と交差項をJacobianで変換します。
旧式の一律0.05 rad²の姿勢分散下限や、姿勢だけへの相対回転・符号補正は加えません。
取付・初期配置を既知定数とした**条件付き共分散**で、初期化点の推定誤差、取付校正誤差、
OpenVINSが公開していない相関を含む絶対不確実性ではありません。Odometryにはposeとtwist
間の共分散も格納できません。上流のvelocity共分散変換等の監査指摘は別途残っています。

## 旧動作の再現

監査用の旧helper関数のimport先は維持します。旧nodeを明示的に起動する場合だけ
`ros2 run pm_localization vio_odom_adapter_legacy_node` を使用してください。
通常launchには使用しません。旧姿勢反転parameterを新nodeへ渡した場合は警告し、無視します。
新nodeと旧nodeを同じtopicで同時起動しないでください。

## Foxyの2D EKFへ渡す場合

元の`/vio/odometry`とゲート通過topicの3D共分散は維持します。
2D EKFへは`planar_twist_covariance_node`が作る`/vio/odometry/twist_planar`を使用します。
Foxyの`forceTwoD`はvz/wx/wyの分散を1e-6へ変更しても交差項を残すため、
3D観測をそのまま使うと不定な共分散を生み得ます。専用入力は(vx,vy,wz)の
周辺共分散を保持し、人工的にゼロ固定する平面外成分との相関だけを除きます。
水平投影方式は既存の水平投影node内で、車輪・VIO・IMUの投影後に同じ周辺化を行います。
3D EKF・監査・VO単独表示へ渡す元の共分散は変更しません。
