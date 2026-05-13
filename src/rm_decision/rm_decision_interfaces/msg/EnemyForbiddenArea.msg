# 敌方禁区检测结果
#
# armors_num: 当前识别到的敌方装甲板数量（来自 /target_tracking）
#   - 0 表示无目标
#   - > 0 表示检测到目标
#
# is_forbidden: 敌人是否在 forbidden 禁区内
#   - true: 敌人已进入禁区（经连续检测滤波确认）
#   - false: 敌人不在禁区 / 无目标

int32 armors_num
bool is_forbidden
