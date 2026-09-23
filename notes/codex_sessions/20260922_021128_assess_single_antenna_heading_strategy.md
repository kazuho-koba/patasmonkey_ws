User prompt

ステップ３に進みたいです。

ただ懸念点として、GNSSアンテナは1⃣つしか載せていないのでGNSSコンパスのような使い方はできないし、基地局からUGV位置までの方角がわかったところでUGVのheadingはわかりませんよね。
むしろ、オドメトリ推定にはwzしか使わないにしても、初期向きだけ（あるいはそれ以外で向きをリセットする必要が生じたときだけ）IMUのyawを使う、かつ実際に使い始める前にIMUの地磁気校正やソフトウェア補正等を実施するというのが一案かと思うのですが、どう思いますか？

Codex final response

その理解で正しいです。単独アンテナGNSSはUGVの車体headingを直接観測できません。基地局への方位は測位幾何であって車体方位ではなく、`/navheading`も、単独アンテナ構成では「dual-antenna heading」と仮定して使ってはいけません。少なくともメッセージ種別・受信機設定・有効フラグを確認できるまでは、移動方向（course over ground）候補として扱うべきです。

提案の「通常は`wz`積分、絶対yawは初期化または明示的リセット時だけ」が、この構成では最も安全です。私はステップ3を次の方針で進めることを勧めます。

1. local odometryは今の分離構成を維持します。水平yawはWit `wz`で連続伝播し、磁気yawを逐次融合しません。
2. navsat_transformのdatum確立時だけ、補正・検証済みのWit yawを地球基準headingとして使います。現在は`use_odometry_yaw: false`で`/wit/imu` orientationを読む一方、[navsat_transform.yaml](/home/kazuho/Projects/patasmonkey_ws/src/pm_config/config/navsat_transform.yaml:6)の`magnetic_declination_radians`と`yaw_offset`がともに0なので、未補正の地磁気yawを使う状態です。ここを校正済み値へ置き換えます。
3. headingリセットは自動・連続ではなく、停止中かつ操作者が要求したときだけに限定します。リセットではdatum/地図座標系を再設定するため、走行中に実施するとglobal軌跡が不連続になります。

校正は「Wit内部の磁気校正だけ」で完結と考えない方がよいです。少なくとも、次の3成分を分離して記録・評価します。

- hard/soft iron: 車体に搭載した状態での地磁気センサ校正。ODrive、配線、電源、鉄材の磁気影響を含めます。
- 磁北→真北: 観測地点・時刻に応じた磁気偏角を`magnetic_declination_radians`として与えます。
- センサ座標→車体座標: Wit取付けのyawずれを`yaw_offset`または前処理ノードの固定補正として与えます。

特にモータ電流によって磁場が変わるなら、静的な校正だけでは補償できません。その場合は「走行中の磁気yawは信用しない」という今回の設計が有効です。校正後も、同じ場所・同じ向きで複数回再起動して初期yawが十分再現すること、直進走行時のGNSS courseと符号・概ねの角度が整合することを受入条件にします。courseは停止・低速・後退・横滑り時にはheadingとして使いませんが、校正の検証材料にはなります。

実装は、補正済みheadingを生成する小さなノードを追加し、(a) 起動直後に停止状態の複数IMUサンプルを平均、(b) 有効判定を通ったらnavsat_transformのdatum初期化にだけ渡す、(c) 明示的なリセット要求時だけ同じ手順を再実行、という形がよいです。`wheel_odometry`の初期yawへ同じ補正値を与えるかは、local odomの軸も真北基準に揃えたいかという別の設計判断なので、navsat_transform用headingと混ぜずに扱います。

したがってステップ3では、まず`/navheading`の型・有効条件を静的/実bagで確認し、Witの補正経路・校正値の保存形式・停止時初期化／手動リセットのROSインターフェースを実装します。その後、実機で校正・再現性試験をしてから、navsat_transformの実入力を切り替える順序が適切です。
