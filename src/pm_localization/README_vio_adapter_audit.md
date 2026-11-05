# VIO adapterの座標系・共分散監査

`pm_localization.vio_rigid_transform` は監査後に実行時adapterでも使用する
共通の変換です。[実行時の設定](README_vio_adapter_runtime.md)も参照してください。IMU取付回転は
実測校正が未完了なので、現在の速度軸補正に由来する回転も候補として扱います。

## 記号と入力規約

`R_AB` はB成分をA成分へ写す回転、`p_AB` はB原点のA成分位置です。
G=OpenVINS global、I=IMU、B=base_link、O=odomとします。

OpenVINSがROSへ出すquaternionはHamiltonとして `R_GI` を表します。
`JPLQuat.h` では内部の誤差を `R_IG_true = Exp(-theta_I) R_IG` と定義しており、
ROS姿勢では `R_GI_true = R_GI Exp(theta_I)` になります。
一方、`ROS2Visualizer.cpp` はpose共分散の順序を並べ替えるだけなので、
その角度誤差成分は依然としてIMU軸です。位置誤差はglobal軸、角度誤差はIMU軸
という混合座標の共分散を、通常のROS固定軸pose共分散と混同しないでください。

`Propagator.cpp` の出力twistはIMU原点の速度・角速度をIMU成分で表します。
共分散は `[v_I, w_I]` の順序です。入力twistを地図方向へ回してEKFへ渡す必要はありません。
出力child_frame_idがbase_linkなら、twistの成分もbase_linkであることが必要です。

## 共通の剛体変換

取付変換 `T_BI=(C,t)`、初期配置 `T_OG=(A,b)` を既知定数として扱います。
`l=-C^T t` はbase原点のIMU成分位置です。

```
p_OB = A (p_GI + R_GI l) + b
R_OB = A R_GI C^T
v_B  = C v_I + skew(t) C w_I
w_B  = C w_I
```

姿勢だけを後から反転したり、原点補正後に別の軸回転を追加したりせず、
同じCとtでposeとtwistを変換します。角速度による速度の原点補正を
`skew(t) C w_I` とし、tと角速度の座標系を揃えます。

OpenVINS入力pose共分散へのJacobianとtwistへのJacobianは次です。

```
J_pose = [ A   -A R_GI skew(l) ]
         [ 0    A R_GI         ]

J_twist = [ C   skew(t) C ]
          [ 0   C         ]

P_out = J P_in J^T
```

出力poseの微小角誤差はodomの固定軸で表します。交差共分散も変換します。
入力pose共分散が既にglobal固定軸へ変換済みの場合のみ
`orientation_error_frame='global'` を指定します。二重変換はしません。
非有限、非対称、半正定値でない入力を共通関数は拒否します。実行時nodeもこの検査を
使い、棄却理由を診断topicへ出します。数値誤差の許容範囲は共通関数に明記しています。

## 残る不確実性

- TF取付位置・回転の校正誤差は含んでいません。
- 初期配置は固定の既知量と仮定します。最初のVIO推定と初期化位置・姿勢の
  共分散、および後続推定との相関は追加していません。最初のサンプルをゼロ化
  してもその誤差がゼロになるわけではありません。
- OpenVINS upstreamのvelocity共分散変換には、姿勢誤差が局所速度へ伝わる
  Jacobian項と交差項が不足しています。角速度共分散にもバイアス等の制約が
  ソースのTODOとして残ります。Odometryで公開されていない相関はadapterだけでは復元できません。
- ROS Odometryはposeとtwist間の共分散を格納できません。両方を同じEKFへ融合する
  ときの相関は、この変換で解消されるわけではありません。
- 非対称共分散が実bagで観測されました。数値誤差か並行処理等の影響かの原因は未確定です。

## 再現方法

Foxyコンテナでoverlayをsourceして実行します。

```bash
python3 -m pytest src/pm_localization/test/test_vio_rigid_transform.py -q
ros2 run pm_evaluation audit_vio_adapter BAG_DIRECTORY \
  --output-dir OUTPUT_DIRECTORY --sample-hz 5
```

監査CLIはraw OpenVINS、adapter実出力（あれば）、static TFを読み、
旧adapter処理の再現と剛体変換を比較します。初期化がbag以前ならその状態は復元できず、
bag内の先頭を代用します。候補は初期yawだけをゼロにして重力方向を保ちます。
bag内の原始観測は変えません。サンプル間引きは解析負荷の削減だけに使います。
姿勢・位置の中心差分とtwistの差は、画像更新による状態修正やサンプリングの
影響も含む内部整合性の指標であり、独立した真値に対する精度ではありません。

実行時nodeは初回local整列と時刻逆行・heading再初期化時の保護を実装しています。
初期配置の不確実性と未公開相関は上記の制約として残ります。
