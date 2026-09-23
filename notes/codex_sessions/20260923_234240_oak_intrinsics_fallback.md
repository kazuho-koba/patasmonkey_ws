# セッション記録

## User prompt

カメラパラメータが同じ値なら、改めて検証する必要はないです。
むしろ前回の検証ではどうやってカメラの各種パラメータを入手していたのですか？fallback値とはなんでしょうか？

## Codex final response

前回の7月bag検証では、カメラから値を取得したのではなく、bagに`/oak/depth/camera_info`が入っていなかったため、設定ファイルの静的fallback値を使っていました。mapperはCameraInfoが画像サイズと一致すればそちらを優先し、ない場合にfallbackが有効で解像度も一致すると、設定済みの`fx, fy, cx, cy`を使います。つまりfallbackは、CameraInfoを持たない古いbag向けの代替値で、実行中にカメラへ問い合わせる仕組みではありません。

今回Jetsonの対象OAK-D S2（`19443010C10EF91200`）からEEPROM値を読み出したところ、mapperが使った640×400 RGB intrinsics 4値と一致しました。そのため、bagの再検証は行っていません。なお、前回のmapperは歪み係数Dを使わず、4つのK値だけで投影していました。fallback値が当初EEPROMから記録されたことは設定コメントと履歴にありますが、当時の具体的な読み出し手順は記録されていません。

照合内容を[レポート](../reports/20260923_234211_jetson_oak_intrinsics_fallback_check.md)に保存しました。
