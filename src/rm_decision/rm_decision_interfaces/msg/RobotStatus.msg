# v1.7.0 机器人性能体系数据
uint8 robot_id # 机器人ID （1~7->红， 101~107->蓝）
uint16 current_hp # 机器人此时血量

# v1.7.0 实时功率热量数据
uint16 shooter_heat # 枪口热量

# 经下位机处理后的数据
bool team_color # 队伍颜色 0-> 红 1->蓝
bool is_attacked # 是否收到攻击 0->未受到攻击 1->受到攻击

# 哨兵姿态
# uint8 reported_posture # 来自裁判系统 0x020D
# uint8 posture # 0x0120

# 17mm 弹丸剩余量
uint16 shot_allowance

