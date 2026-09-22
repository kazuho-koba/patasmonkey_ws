User prompt

ありがとうございます、このやり方でglobal ekfの出力に機体の姿勢（roll, pitch, yaw）もある程度は入りますか？

Codex final response

はい、入ります。global EKFの出力`/odometry/global`は引き続き3D姿勢を持たせられます。ただし精度の意味は軸ごとに分けて扱います。

| 出力姿勢 | 入力・意味 |
|---|---|
| roll / pitch | Wit IMUのgravity基準orientationをglobal EKFへ直接融合します。坂・段差での姿勢変化を反映できます。 |
| yawの相対変化 | Wit `wz`を直接融合して伝播します。旋回の連続性を保てます。 |
| yawの地球基準の絶対値 | 実機で校正済みheading初期化が成功したときだけ与えます。それまでは「初期値からの相対yaw」です。 |

したがって、globalの姿勢は次のようになります。

```text
roll/pitch: その時点の車体傾斜として利用可能
yaw:       旋回量・向きの連続性は利用可能
yawの北/東に対する絶対方位: 校正完了まで未保証
```

重要なのは、global EKFへ`/odometry/local`のposeを丸ごと入れないことです。代わりにWit IMUをglobal EKFへ直接入れ、`roll`、`pitch`、`wz`を選択して使います。これならlocal EKFを経由した相関済み姿勢や、VIO破綻がglobalの姿勢へ複製されません。

VIOはglobalでも健全時の`vx/vy`だけを使う予定なので、VIO orientationはglobal姿勢へ使いません。VIOのz/roll/pitch/yawが壊れてもglobal姿勢を壊さない設計です。

校正済みheadingが利用可能になった後は、ステップ3の初期化値をglobal EKFの初期yawにも同じ一回限りの初期値として整合させます。これは連続的な磁気yaw融合ではありません。
