builtin_interfaces/Time stamp

float32 left_cmd_rps
float32 right_cmd_rps

float32 left_pos_turns
float32 right_pos_turns

float32 left_vel_rps
float32 right_vel_rps

# ODriveボード全体のDC入力電流 [A]。正は消費、負は電源への回生。
# 左右モーターとブレーキ抵抗を含むODriveの推定値。他の電池負荷は含まない。
float32 ibus_a

float32 vbus_voltage
