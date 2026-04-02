# ============================================
# GimbalCmd.msg
# 云台控制指令消息（结构化版本）
# ============================================

# ---- 控制类型枚举 ----
int8 ABSOLUTE_ANGLE = 0   # 绝对角度控制（position 模式）
int8 VELOCITY       = 1   # 速度控制（velocity 模式）

# ---- 控制模式 ----
int8 pitch_type     # pitch 控制模式（ABSOLUTE_ANGLE 或 VELOCITY）
int8 yaw_type       # yaw 控制模式（ABSOLUTE_ANGLE 或 VELOCITY）

# ---- 云台位置 ----
# 当 pitch_type 或 yaw_type 为 ABSOLUTE_ANGLE 时使用
GimbalPosition position

# ---- 云台速度 ----
# 当 pitch_type 或 yaw_type 为 VELOCITY 时使用
GimbalVelocity velocity

