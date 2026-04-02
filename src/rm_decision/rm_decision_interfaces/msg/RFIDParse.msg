# 解析后的 rfid 数据
# ---- 基础点 ----
bool base_self           # bit0 己方基地增益点
bool highland_self       # bit1 己方中央高地增益点     
bool highland_enemy      # bit2 对方中央高地增益点
bool slope_self          # bit3 己方梯形高地增益点
bool slope_enemy         # bit4 对方梯形高地增益点

# ---- 飞坡 ----
bool fly_self_front      # bit5 己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前）
bool fly_self_back       # bit6 己方地形跨越增益点（飞坡）（靠近己方一侧飞坡后）
bool fly_enemy_front     # bit7 对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前）
bool fly_enemy_back      # bit8 对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后）

# ---- 中央高地地形跨越 ----
bool center_low_self     # bit9  己方地形跨越增益点（中央高地下方）
bool center_high_self    # bit10 己方地形跨越增益点（中央高地上方）
bool center_low_enemy    # bit11 对方地形跨越增益点（中央高地下方）
bool center_high_enemy   # bit12 对方地形跨越增益点（中央高地上方）

# ---- 公路 ----
bool road_low_self       # bit13 己方地形跨越增益点（公路下方）
bool road_high_self      # bit14 己方地形跨越增益点（公路上方）
bool road_low_enemy      # bit15 对方地形跨越增益点（公路下方）
bool road_high_enemy     # bit16 对方地形跨越增益点（公路上方）

# ---- 战略点 ----
bool fortress_self       # bit17 己方堡垒增益点
bool outpost_self        # bit18 己方前哨站增益点
bool resource_isolated   # bit19 己方与资源区不重叠的补给区/RMUL 补给区
bool resource_overlap    # bit20 己方与资源区重叠的补给区
bool supply_self         # bit21 己方装配增益点
bool supply_enemy        # bit22 对方装配增益点
bool center_bonus        # bit23 中心增益点（仅 RMUL 适用）

# ---- 敌方点 ----
bool fortress_enemy      # bit24 对方堡垒增益点
bool outpost_enemy       # bit25 对方前哨站增益点

# ---- 隧道（己方）----
bool tunnel_self_1       # bit26 己方地形跨越增益点（隧道）（靠近己方一侧公路区下方）
bool tunnel_self_2       # bit27 己方地形跨越增益点（隧道）（靠近己方一侧公路区中间）
bool tunnel_self_3       # bit28 己方地形跨越增益点（隧道）（靠近己方一侧公路区上方）
bool tunnel_self_4       # bit29 己方地形跨越增益点（隧道）（靠近己方梯形高地较低处）
bool tunnel_self_5       # bit30 己方地形跨越增益点（隧道）（靠近己方梯形高地较中间）
bool tunnel_self_6       # bit31 己方地形跨越增益点（隧道）（靠近己方梯形高地较高处）

# ---- 隧道（敌方）----
bool tunnel_enemy_1      # bit32 对方地形跨越增益点（隧道）（靠近对方公路一侧下方）
bool tunnel_enemy_2      # bit33 对方地形跨越增益点（隧道）（靠近对方公路一侧中间）
bool tunnel_enemy_3      # bit34 对方地形跨越增益点（隧道）（靠近对方公路一侧上方）
bool tunnel_enemy_4      # bit35 对方地形跨越增益点（隧道）（靠近对方梯形高地较低处）
bool tunnel_enemy_5      # bit36 对方地形跨越增益点（隧道）（靠近对方梯形高地较中间）
bool tunnel_enemy_6      # bit37 对方地形跨越增益点（隧道）（靠近对方梯形高地较高处）
