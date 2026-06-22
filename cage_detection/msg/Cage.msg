std_msgs/Header header

# カゴの色 (0=GREEN, 1=BLUE)
uint8 GREEN=0
uint8 BLUE=1
uint8 color

# カメラ座標系での3D位置 [m]
# x: 右方向, y: 下方向, z: 前方向（射出距離）
geometry_msgs/Point position

# 射出距離（position.z の便宜コピー）[m]
float64 distance

# 優先度スコア（高いほど優先、占有済みは-999）
int32 priority

# 占有フラグ（自チームが既に占有している場合 true）
bool occupied
