#v1.7.0 
#机器人云台及其底盘控制 
bool stop_gimbal_scan #是否停止云台扫描 0->扫描 1->停止扫描 
float32 chassis_spin_vel # 底盘小陀螺旋转速度（rad/s）
float32 gimbal_big_yaw_vel # 大云台旋转速度（rad/s）
float32 gimbal_big_yaw_angle #大云台旋转角度 （rad  -π到π）
bool follow_gimbal_big  # 底盘跟随 false 不跟随   true 跟随
float32 chassis_vel_multi # 底盘倍率 对于速度，仅需要0~1倍率
bool track_status # 是否启动履带 false 不启动 true 启动