User prompt:
rvizの設定で、カメラがロボットを追従するようにできないでしょうか？

Codex final response:
できます。RVizのOrbit viewの`Target Frame`を`base_link`へ変更し、`odom`をFixed Frameのまま保持する設定にしました。

これで地図・点群はodom座標に固定されたまま、カメラの注視点だけがロボットへ追従します。`pm_perception`はFoxyコンテナ内で再build済みです。

起動後に追従を止めたり別frameへ変える場合は、左側の`Views → Current View → Target Frame`を変更できます。
