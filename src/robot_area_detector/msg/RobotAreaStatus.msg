# 机器人区域检测结果
#
# is_in_area: 机器人是否在任意匹配区域内
# area_name: 第一个命中的区域名字（无命中则为空）
# area_type: 第一个命中的区域类型（无命中则为空）
# robot_x/y: 机器人在 map 坐标系下的位置
# matched_area_count: 同时命中的区域数量
# matched_area_names/types: 所有命中区域的名字和类型

std_msgs/Header header

bool is_in_area

string area_name
string area_type

float64 robot_x
float64 robot_y

int32 matched_area_count
string[] matched_area_names
string[] matched_area_types
