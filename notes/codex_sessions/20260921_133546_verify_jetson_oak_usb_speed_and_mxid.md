Codex session record
Timestamp: 2026-09-21 13:35:46 JST

User prompt
===========
oak-d s2のUSB接続速度を改めて確認してください。また接続されているカメラのIDは、ホストPCで確認したのとは違うHWを使っているので、それも確認してください。

Codex final response
====================
確認結果は次のとおりです。

- Jetson側のOAK-D S2は現在もUSB 2.0、480 Mbps接続です。
- Jetson側のカメラMXIDは`19443010C10EF91200`です。
- ホストPCで確認した`19443010E122001300`とは別個体です。

USB速度は次の3箇所すべてで一致しました。

```text
lsusb -t: 480M
sysfs speed: 480
USB descriptor bcdUSB: 2.00
```

接続位置はJetsonのUSB 2.0 root hub側です。

```text
Bus 001, Port 4, Device 10
03e7:2485 Intel Movidius MyriadX
/dev/bus/usb/001/010
```

一方、USB 3側にはVIA Labsのハブが`5000M`で存在しますが、その配下にOAKは列挙されていません。この状態ではOAKの転送帯域は480 Mbpsに制限されます。

DepthAI 2.30.0の列挙結果は以下です。

```text
count=1
mxid=19443010C10EF91200
name=1.4
state=X_LINK_UNBOOTED
protocol=X_LINK_USB_VSC
```

ストリームやパイプラインは起動しておらず、確認後も`03e7:2485`、`X_LINK_UNBOOTED`のままです。USBノードは`root:root 0666`なので、`nvidia`ユーザーからアクセス可能です。

校正設定も照合しました。Jetsonの`pm_bag_global_localization.launch.py`は`estimator_config1.yaml`を選択しており、そこから次を参照しています。

```text
kalibr_imucam_chain_oakds2_19443010C10EF91200.yaml
```

したがって、Jetsonで現在接続されているC10個体については、選択中の校正ファイルとMXIDが一致しています。前回報告した「E122個体との校正不一致」は、JetsonにもホストPCと同じE122個体が接続されているという仮定に基づくもので、今回の実機ID確認により訂正します。

Jetson側には何も変更していません。今回の記録は[20260921_133546_verify_jetson_oak_usb_speed_and_mxid.txt](/home/kazuho/Projects/patasmonkey_ws/notes/codex_sessions/20260921_133546_verify_jetson_oak_usb_speed_and_mxid.txt)に保存しました。
