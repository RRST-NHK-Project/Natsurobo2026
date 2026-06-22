std_msgs/Header header

# 仰角コマンド [rad]
# 0.0 = 水平、正値 = 上向き
float64 elevation_rad

# フライホイール目標回転数 [RPM]
# 0 = 停止
float64 flywheel_rpm

# 射出トリガー (true で発射)
bool fire

# 発射先カゴID（デバッグ用）
uint8 target_cage_color   # 0=GREEN, 1=BLUE
float64 target_distance_m
