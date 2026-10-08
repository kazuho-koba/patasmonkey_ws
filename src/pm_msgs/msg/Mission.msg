# 経路の転送・保存用。受領は走行開始や走行可能性の保証を意味しない。
uint32 schema_version
string mission_id
string name
uint64 revision
pm_msgs/MissionWaypoint[] waypoints
