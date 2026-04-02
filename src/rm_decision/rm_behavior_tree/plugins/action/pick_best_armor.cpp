#include "rm_behavior_tree/plugins/action/pick_best_armor.hpp"
#include <algorithm>
#include <armor_interfaces/msg/detail/armors__struct.hpp>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
// 这个用不了，传给的 x y点不精确
namespace rm_behavior_tree {
PickBestArmorAction::PickBestArmorAction(const std::string& name, const BT::NodeConfig& confing)
    : BT::SyncActionNode(name, confing)
{

}

BT::NodeStatus PickBestArmorAction::tick()
{
    if (getInput<armor_interfaces::msg::Armors>("armors_message", armors)) {
        return BT::NodeStatus::FAILURE;
    }

    auto best = std::min_element(armors.armors.begin(), armors.armors.end(),
        [](auto& a, auto& b) { return a.distance_to_image_center < b.distance_to_image_center;});
    setOutput("armor_message", *best);
    return BT::NodeStatus::SUCCESS;
}

}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::PickBestArmorAction>("PickBestArmor");
}