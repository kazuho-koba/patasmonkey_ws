# GNSS軌跡の地図描画

`plot_gnss_bags`はMCAP bagの`/fix`だけから移動軌跡を作り、`/navpvt`の
RTK状態とともに、指定したOpenStreetMapキャッシュ画像へ重ねる。各bagの最初と
最後の有効fixをSTART・GOALで示し、時間順の矢印を数か所表示する。
START・GOALは記録区間端のfixであり、実際の周回開始・終了の保証ではない。

```bash
ros2 run pm_evaluation plot_gnss_bags <bagディレクトリ> [<bagディレクトリ> ...] \
  --basemap <imageとextentを含むEPSG:3857のnpz> --summary <集計json>
```

画像は各bag直下の`gnss_trajectory_osm.png`に保存し、既存画像の上書きには
`--overwrite`が必要。GNSS座標が地図キャッシュ範囲外なら誤解を防ぐため中止する。
測位状態はbag記録時刻で近傍2秒以内のNAV-PVTから求め、それ以外はUNKNOWN。
無効fixは除外し、3秒超の欠測間を線で結ばない。距離はWGS84測地線に基づく
参考値で、GNSSの誤差や停止中のばらつきを含む。背景地図にOSMを使う場合は
利用元の規約を守り、クレジットを表示する。bag・画像には位置情報が含まれる。

## 記録済みlocal/global odometryの重畳

```bash
ros2 run pm_evaluation plot_recorded_local_global <bagディレクトリ> [<bagディレクトリ> ...] \
  --basemap <OSMキャッシュnpz> --summary <集計json>
```

入力bagの`/odometry/local`、`/odometry/global`、`/odometry/gps`、`/fix`、
`/tf`を使い、記録済み出力を変更せず描画する。開始時点のGNSSとnavsat変換の
同時刻ペアでmapを局所東北座標へ配置し、localは開始時のmap→odom TFを固定して
一度だけmapへ変換する。GNSSへの軌跡fitは行わない。標準図と全軌跡図、両odometry
の地図座標CSVを各bagへ保存する。背景地図の範囲外には背景がない。GNSSとglobalは
独立した観測ではないため、距離差を真値誤差とみなせない。既存の同名出力は上書きする。

## 記録済みzとGNSS高度

`ros2 run pm_evaluation plot_recorded_height <bag...> --summary <json>`でlocal・globalの
記録済みzとGNSS高度を各bagに描画する。異なる基準の比較のため、初めのGNSS高度を
同時刻のlocal zへ一致させ、別パネルに未補正の高度も表示する。これは真値校正ではない。
`--overwrite`指定時のみ既存画像を上書きする。
