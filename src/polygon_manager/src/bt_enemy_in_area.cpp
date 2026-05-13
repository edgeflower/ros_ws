#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "polygon_manager/polygon_manager.hpp"

namespace polygon_manager {

/**
 * @class EnemyInAreaCondition
 * @brief BehaviorTree 条件节点：检测敌方是否在指定区域内
 * 
 * BehaviorTree.CPP v4 集成说明：
 * ============================
 * 
 * BehaviorTree 是一个用于机器人任务规划的框架
 * 典型的行为树结构：
 * 
 *         Root
 *        /    \
 *      Seq   Parallel
 *     / | \
 *   C1 A1 C2
 * 
 * 其中：
 * - C1, C2: 条件节点（ConditionNode）
 * - A1: 动作节点（ActionNode）
 * - Seq: 顺序节点（SelectorNode）
 * 
 * 节点状态：
 * - SUCCESS: 条件满足/动作完成
 * - FAILURE: 条件不满足/动作失败
 * - RUNNING: 动作正在进行
 * 
 * 本节点是 ConditionNode，用于检测敌方位置
 * 
 * 使用示例：
 * =======
 * 在 BehaviorTree XML 中：
 * 
 * <root main_tree_to_execute="MainTree">
 *   <BehaviorTree ID="MainTree">
 *     <Sequence>
 *       <EnemyInArea enemy_pose="${enemy_pose}"
 *                    area_name="forbidden_zone_1"/>
 *       <AlertAction/>
 *     </Sequence>
 *   </BehaviorTree>
 * </root>
 * 
 * 编译和注册：
 * 需要使用 BehaviorTreeFactory 来注册该节点
 * @code
 * BT::BehaviorTreeFactory factory;
 * factory.registerNodeType<EnemyInAreaCondition>("EnemyInArea");
 * @endcode
 */
class EnemyInAreaCondition : public BT::ConditionNode {
 public:
  /**
   * @brief 构造节点
   * 
   * @param name 节点名称
   * @param config 节点配置（BehaviorTree 框架提供）
   */
  EnemyInAreaCondition(const std::string& name,
                       const BT::NodeConfig& config)
      : BT::ConditionNode(name, config),
        manager_(nullptr),
        node_(nullptr) {
    
    // 尝试获取全局 PolygonManager 实例
    // 这需要在 main 函数中通过全局变量或单例模式提供
    RCLCPP_DEBUG(rclcpp::get_logger("EnemyInAreaCondition"),
                 "EnemyInAreaCondition node created: %s", name.c_str());
  }

  /**
   * @brief 设置 PolygonManager 实例
   * 
   * 在创建行为树之前，需要调用此函数来设置管理器
   * 
   * @param manager 指向 PolygonManager 的指针
   */
  static void setManager(PolygonManager* manager) {
    global_manager_ = manager;
  }

  /**
   * @brief 获取节点的输入端口描述
   * 
   * 返回此节点期望的输入参数
   * BehaviorTree 框架会自动验证和转换参数类型
   */
  static BT::PortsList providedPorts() {
    return {
        // 敌方位置（PoseStamped 格式）
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
            "enemy_pose",
            "Enemy position as PoseStamped message"),

        // 要检查的区域名称（字符串）
        BT::InputPort<std::string>(
            "area_name",
            "Name of the area to check"),

        // 可选：检查的区域类型而不是名称
        BT::InputPort<std::string>(
            "area_type",
            "{forbidden,protect,patrol,attack}")
    };
  }

  /**
   * @brief 节点执行函数（由 BehaviorTree 框架调用）
   * 
   * 执行流程：
   * 1. 获取敌方位置参数
   * 2. 获取要检查的区域名称
   * 3. 调用 PolygonManager 检测敌方
   * 4. 返回 SUCCESS/FAILURE
   * 
   * @return BT::NodeStatus
   *   - SUCCESS: 敌方在指定区域内
   *   - FAILURE: 敌方不在指定区域内
   */
  BT::NodeStatus tick() override {
    // 检查管理器是否已初始化
    if (!global_manager_) {
      RCLCPP_ERROR(rclcpp::get_logger("EnemyInAreaCondition"),
                   "PolygonManager not initialized!");
      return BT::NodeStatus::FAILURE;
    }

    // 尝试获取敌方位置参数
    auto enemy_pose_result = getInput<geometry_msgs::msg::PoseStamped>(
        "enemy_pose");

    if (!enemy_pose_result) {
      RCLCPP_ERROR(rclcpp::get_logger("EnemyInAreaCondition"),
                   "Missing enemy_pose input");
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped enemy_pose = enemy_pose_result.value();

    // 尝试按区域名称检查
    auto area_name_result = getInput<std::string>("area_name");
    if (area_name_result) {
      std::string area_name = area_name_result.value();

      // 获取敌方所在的所有区域
      auto enemy_areas = global_manager_->getEnemyAreaNames(enemy_pose);

      // 检查是否在指定区域内
      for (const auto& name : enemy_areas) {
        if (name == area_name) {
          RCLCPP_DEBUG(rclcpp::get_logger("EnemyInAreaCondition"),
                       "Enemy in area: %s", area_name.c_str());
          return BT::NodeStatus::SUCCESS;
        }
      }

      RCLCPP_DEBUG(rclcpp::get_logger("EnemyInAreaCondition"),
                   "Enemy NOT in area: %s", area_name.c_str());
      return BT::NodeStatus::FAILURE;
    }

    // 如果没有提供区域名称，尝试按类型检查
    auto area_type_result = getInput<std::string>("area_type");
    if (area_type_result) {
      std::string type_str = area_type_result.value();

      // 将字符串转换为 AreaType
      AreaType check_type;
      if (type_str == "forbidden") {
        check_type = AreaType::FORBIDDEN_AREA;
      } else if (type_str == "protect") {
        check_type = AreaType::PROTECT_AREA;
      } else if (type_str == "patrol") {
        check_type = AreaType::PATROL_AREA;
      } else if (type_str == "attack") {
        check_type = AreaType::ATTACK_AREA;
      } else {
        RCLCPP_WARN(rclcpp::get_logger("EnemyInAreaCondition"),
                    "Unknown area type: %s", type_str.c_str());
        return BT::NodeStatus::FAILURE;
      }

      // 检查敌方是否在此类型的区域内
      auto enemy_area_types = global_manager_->getEnemyAreas(enemy_pose);
      for (const auto& type : enemy_area_types) {
        if (type == check_type) {
          RCLCPP_DEBUG(rclcpp::get_logger("EnemyInAreaCondition"),
                       "Enemy in area type: %s", type_str.c_str());
          return BT::NodeStatus::SUCCESS;
        }
      }

      RCLCPP_DEBUG(rclcpp::get_logger("EnemyInAreaCondition"),
                   "Enemy NOT in area type: %s", type_str.c_str());
      return BT::NodeStatus::FAILURE;
    }

    // 既没有提供区域名称也没有提供区域类型
    RCLCPP_ERROR(rclcpp::get_logger("EnemyInAreaCondition"),
                 "Must provide either 'area_name' or 'area_type'");
    return BT::NodeStatus::FAILURE;
  }

 private:
  /// 全局 PolygonManager 实例指针（所有节点共用）
  static PolygonManager* global_manager_;

  PolygonManager* manager_;
  rclcpp::Node* node_;
};

/// 静态成员初始化
PolygonManager* EnemyInAreaCondition::global_manager_ = nullptr;

}  // namespace polygon_manager

/**
 * @brief BehaviorTree 插件导出函数
 * 
 * BehaviorTree 框架会调用此函数来注册插件
 * 使用 REGISTER_BT_FACTORY 宏来自动导出
 */
extern "C" {

void registerNodes(BT::BehaviorTreeFactory& factory) {
  factory.registerNodeType<polygon_manager::EnemyInAreaCondition>(
      "EnemyInArea");
}

}
