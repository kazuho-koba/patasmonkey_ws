User prompt:
いまperceptionの幅（点群やelevationが表示される幅）はいくつですか？

Codex final response:
現在のrolling elevation mapの幅は、前後 6.0 m × 左右 6.0 mです。

`base_link`を中心に、`odom`上で概ね前後±3 m・左右±3 mのwindowになります。車両が動くとこのwindowも追従します。

ただし実際に点やelevationが埋まる範囲は、OAK-Dの画角とdepthの使用範囲 0.4〜4.0 mに制限されます。そのため、RViz上で前方は最大約4 mまで、左右はその距離でのカメラ画角相当の範囲が主に観測されます。
