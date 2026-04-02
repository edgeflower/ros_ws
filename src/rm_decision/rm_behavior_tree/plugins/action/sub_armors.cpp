// 包含当前节点插件对应的头文件，该头文件中包含了类声明以及必要的依赖
#include "rm_behavior_tree/plugins/action/sub_armors.hpp"
#include <armor_interfaces/msg/detail/target__struct.hpp>

namespace rm_behavior_tree
{

// ------------------------------------------------------------------
// SubArmorsAction 构造函数的实现
// ------------------------------------------------------------------
/*
  该构造函数用于创建一个 SubArmorsAction 节点实例。
  参数说明：
    - name：节点的名称，用于在行为树中标识该节点。
    - conf：节点的配置（NodeConfig），通常包含输入/输出端口等配置信息。
    - params：ROS 节点参数（RosNodeParams），用于传递 ROS2 节点相关的配置或句柄。
  
  构造函数使用初始化列表调用基类构造函数，
  这里基类是 BT::RosTopicSubNode<auto_aim_interfaces::msg::Target>，
  该基类封装了基于 ROS2 话题订阅的行为树节点，
  模板参数 auto_aim_interfaces::msg::Target 表示该节点订阅的数据消息类型。
*/
SubArmorsAction::SubArmorsAction(
    const std::string & name, 
    const BT::NodeConfig & conf, 
    const BT::RosNodeParams & params)
: BT::RosTopicSubNode<armor_interfaces::msg::Target>(name, conf, params)
{
    // 构造函数体为空，所有初始化工作均在基类构造函数中完成
}

// ------------------------------------------------------------------
// onTick() 方法的实现
// ------------------------------------------------------------------
/*
  onTick() 是当节点被 tick 时调用的回调函数，用于处理从 ROS2 话题接收到的最新消息。
  参数：
    - last_msg：一个指向 auto_aim_interfaces::msg::Target 消息的共享指针，
               表示当前订阅到的最新消息。如果没有接收到消息，该指针可能为空。

  函数逻辑：
    1. 判断 last_msg 是否有效（非空），如果有效：
         调用 setOutput("armors", *last_msg) 将消息内容设置到黑板中，
         其中 "armors" 是对应的黑板键名，供后续节点使用。
    2. 最后，无论消息是否有效，都返回 BT::NodeStatus::SUCCESS，
       表示节点执行成功。
*/
BT::NodeStatus SubArmorsAction::onTick(
    const std::shared_ptr<armor_interfaces::msg::Target> & last_msg)
{
    if(last_msg){
        // 将收到的消息解引用后，写入黑板的 "armors" 键中
        setOutput("armors", *last_msg);
    }
    // 返回 SUCCESS 状态
    return BT::NodeStatus::SUCCESS;
}

} // namespace rm_behavior_tree

// ------------------------------------------------------------------
// 插件导出部分
// ------------------------------------------------------------------

// 包含 BehaviorTree.Ros2 插件宏定义头文件，该文件定义了用于注册和导出插件的宏
#include "behaviortree_ros2/plugins.hpp"

// 通过 CreateRosNodePlugin 宏将 SubArmorsAction 插件导出
// 第一个参数：插件的完整类名（含命名空间）
// 第二个参数："SubArmors" 是插件在 XML 配置文件中使用的名称
CreateRosNodePlugin(rm_behavior_tree::SubArmorsAction, "SubArmors");
