User prompt
ありがとうございます、それが先程サジェストしてくれた設計分離ということですよね？

Codex final response
はい、方向性は同じです。ただし「VIO twistのみを使う」は分離設計の水平系を安全にする部分で、設計分離そのものはさらに一段あります。

```text
水平EKF: wheel vx + Wit wz + gated VIO vx/vy  -> x, y, yaw
姿勢・高さ系: Wit roll/pitch + 必要に応じて gated VIO z/vz -> roll, pitch, z
合成: 水平系のx/y/yawと、姿勢・高さ系のroll/pitch/zを /odometry/local として出力
```

この構成では、VIO pose resetを水平EKFへ一切入れず、VIO速度も異常時だけ遮断・回復後に再受理できます。さらにVIO zが破綻しても、水平位置が巻き込まれません。

先に提案した単一3D EKFへVIO zを入れる案は、7月・8月の再生で水平状態まで汚すことが分かったため、採用しません。次は上記の分離構成として実装・再検証するのが適切です。
