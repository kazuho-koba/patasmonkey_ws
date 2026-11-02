# WGS84 latitude/longitude（degree）。高さやmap frame座標は含めない。
uint8 PASS=0
uint8 START=1
uint8 GOAL=2
uint8 PAUSE=3
uint8 CONFIRM=4
# 必ず到達半径内へ寄る。GOALと異なり停止は要求しない。
uint8 VISIT=5
string waypoint_id
float64 latitude
float64 longitude
uint8 attribute
# PAUSE属性の待機時間（秒）。走行executorによる実行は後続実装。
float64 pause_sec
string note
