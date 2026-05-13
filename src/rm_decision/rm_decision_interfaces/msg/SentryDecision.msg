# 哨兵 Utility AI 战术决策消息
#
# mode 可取值:
#   IDLE         - 空闲（比赛未开始或其他非战斗状态）
#   ATTACK       - 攻击（追踪并攻击敌人）
#   RETREAT      - 撤退（血量低或受到严重威胁时回撤）
#   SUPPLY       - 补给（前往补给区恢复血量/弹药）
#   PATROL       - 巡逻（无敌人时在巡逻区游走）
#   DEFEND_BASE  - 防守基地（敌人接近己方基地时回防）
#
# 各项 score 表示 Utility AI 对每个模式的评分，分数越高越倾向于选择该模式
# reason 记录本次决策的关键原因，方便调试
# target_pose 为当前决策对应的目标点（map 坐标系）

std_msgs/Header header

string mode                     # 当前战术模式

float64 attack_score            # 攻击评分
float64 retreat_score           # 撤退评分
float64 supply_score            # 补给评分
float64 patrol_score            # 巡逻评分
float64 defend_base_score       # 防守基地评分

string reason                   # 决策原因说明

geometry_msgs/PoseStamped target_pose   # 目标点位置
