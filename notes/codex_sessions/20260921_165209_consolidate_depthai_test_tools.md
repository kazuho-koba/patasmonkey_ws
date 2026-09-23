User prompt
===========

開発PCコンテナ内での作業です。jetsonへの反映は自分でやります。
いま、oak-d s2のテストスクリプト（画像のプレビューなど）が、patasmonkey_ws/tools/depthai_testsと、depthai_driver/tools/oak_openvinsに別れてしまっているんですよね。
前者に入っているスクリプトを後者に移した上で、移し元のディレクトリを削除してもらえないでしょうか。

4点付記事項があって、
まず1点目、移し先で同じ機能を持つpythonファイルが既に存在している可能性がかなりあるので、その場合は移さず削除でいいです。または非常に類似している場合は統合も視野に入れてください。
2点目、開発PCコンテナのdepthaiのバージョンを確認して、それと整合しないスクリプトがある場合は整合するように書き換えてください。
3点目、depthai_driver/tools/oak_openvinsはネストが深すぎる、かつopenvinsは正直そこまで関係ないので、移動したファイルも含めて全て1階層上に移動させてください（validate_oak_diagnostics.pyと競合するようなら私の指示を仰いでください）
最後に、USBの接続速度（USB3.0, 5000MB/s出ているか）を確認するpythonスクリプトを、最終移動先のdepthai_driver/toolsに追加してほしいです。

Codex final response
====================

開発PC側で整理・検証まで完了しました。Jetsonには接続も変更もしていません。

- `patasmonkey_ws/tools/depthai_tests` は削除しました。
- `depthai_driver/tools/oak_openvins` 内の18スクリプトを [depthai_driver/tools](/home/kazuho/ros2_ws/src/depthai_driver/tools) 直下へ移し、旧ディレクトリを削除しました。
- 重複していたRGB・Depth表示スクリプトは、移動先にあったDepthAI 2.x対応版を採用しました。
- 元側にだけあったデバイス列挙、IMU、RGBプレビュー、RGB+Depth+IMU負荷試験の4本を移管しました。DepthAI 3系APIだったため、コンテナの `depthai 2.30.0.0` に合わせて書き換えています。
- 非推奨の `RGB/LEFT/RIGHT` ソケット指定も `CAM_A/CAM_B/CAM_C` に更新しました。
- 簡易版 `check_usbconnection.py` は、判定と終了コードを備えた [check_usb_speed.py](/home/kazuho/ros2_ws/src/depthai_driver/tools/check_usb_speed.py) に置き換えました。複数台接続時は `--mxid` で選択できます。
- `validate_oak_diagnostics.py` との競合はありませんでした。

検証結果:

- 全 `tools/*.py` の構文チェック: 成功
- DepthAI 2.30でRGB+Depth+IMUパイプライン構築: 成功
- RGBプレビュー30フレーム取得: 成功
- IMUを約10秒、979パケット取得: 成功
- OAK-D S2 MX ID: `19443010E122001300`
- USB判定: `UsbSpeed.SUPER`、USB 3.xとして合格

なおUSB 3.0の公称信号速度は「5000 MB/s」ではなく「5000 Mb/s = 5 Gb/s」です。実効データ転送量はさらに低くなります。また、現在のコンテナではUID/GID 1000だとOAKのudev権限不足になり、rootでは正常に判定できました。今回は設定変更を行っていません。

ビルド対象のROSコードは変更していないためcolcon buildは省略し、Python構文・DepthAI API・実機ストリームで検証しました。既存の `AGENTS.md` と `notes/misc.txt` の変更には触れていません。
