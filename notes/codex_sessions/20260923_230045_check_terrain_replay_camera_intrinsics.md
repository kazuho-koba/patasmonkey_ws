User prompt:
検証に際してカメラintrinsincsは何を使っていますか？
src/pm_config/config/oak_d_s2/kalibr_imucam_chain_oakds2_19443010C10EF91200.yaml
にデータがあります。このファイル名末尾の1944----1200がUGVに搭載されているOAKD s2のS/Nで、これはcalibrを使って得たカメラパラメータです。
これを使ったら検証結果が変化する可能性はありますか？

Codex final response:
今回の7月bag検証では、ご提示のKalibrファイルではなく、mapper設定のEEPROM fallback intrinsicsを使っています。旧bag用launchは通常設定に旧bag設定を重ねて読み込みます。7月bagには`/oak/depth/camera_info`がないため、640×400画像に対して次の値が選ばれます。

- `fx=574.288`, `fy=574.288`, `cx=354.751`, `cy=215.233`

ご提示のファイルの`cam0`は`/oak/stereo/left/image_raw`用で、値は`fx=406.225`, `fy=406.484`, `cx=326.284`, `cy=205.634`です。`cam1`は右ステレオカメラ用です。したがって、このファイルの値は検証で実際に使われていません。

intrinsicsを変えれば結果が変わる可能性は十分あります。画素から作る光線の向きが変わり、点のodom上のXY・高さ・セル割り当てが変化するため、同じセルへのground融合やhazard判定も変わり得ます。特に今回の旧・新候補は画像中心から離れた画素を含むので、無視できるとは言えません。

ただし、Kalibrの値をそのままdepth画像に適用するのはまだ適切とは限りません。リポジトリの説明では`/oak/depth/image_raw`はRGBへ整列済みですが、Kalibrファイルの`cam0`はステレオ左カメラです。またmapperは`fx/fy/cx/cy`だけを使い、Kalibrの歪み係数による補正はしていません。まずdepth画像がどのカメラモデル・画像処理後の座標系に対応するかを確認し、その画像に対応したCameraInfoまたは校正値を選ぶ必要があります。

結論として、「校正値へ差し替えれば原因が解決する」とはまだ言えませんが、intrinsicsは今回の原因切り分けに影響し得る未検証要因です。正しいdepth画像用intrinsicsを確定したうえで、他条件を固定したA/B再生を行うのが次の検証になります。
