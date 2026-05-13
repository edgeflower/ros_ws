#include "polygon_manager/polygon_edit_panel.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace polygon_manager {

PolygonEditPanel::PolygonEditPanel(QWidget* parent)
    : rviz_common::Panel(parent) {
  setupUI();
}

void PolygonEditPanel::onInitialize() {
  node_ = std::make_shared<rclcpp::Node>("polygon_edit_panel");

  area_type_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/polygon_manager/set_area_type", 10);

  close_client_ = node_->create_client<std_srvs::srv::Trigger>(
      "/polygon_manager/close_polygon");

  cancel_client_ = node_->create_client<std_srvs::srv::Trigger>(
      "/polygon_manager/cancel_polygon");

  status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/polygon_manager/status", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        // 从 ROS 回调线程安全地发射信号到 Qt 主线程
        Q_EMIT statusChanged(QString::fromStdString(msg->data));
      });

  // 连接跨线程信号（QueuedConnection 确保 Qt 主线程处理）
  connect(this, &PolygonEditPanel::statusChanged,
          this, &PolygonEditPanel::updateStatusLabel,
          Qt::QueuedConnection);

  // 发布默认类型
  onAreaTypeChanged(0);

  RCLCPP_INFO(node_->get_logger(), "PolygonEditPanel initialized");
}

void PolygonEditPanel::setupUI() {
  auto* main_layout = new QVBoxLayout;

  // === 区域类型选择 ===
  auto* type_group = new QGroupBox("区域类型");
  auto* type_layout = new QVBoxLayout;

  area_type_combo_ = new QComboBox;
  area_type_combo_->addItem("禁区 (forbidden)", "forbidden");
  area_type_combo_->addItem("防守区 (protect)", "protect");
  area_type_combo_->addItem("巡逻区 (patrol)", "patrol");
  area_type_combo_->addItem("攻击区 (attack)", "attack");
  type_layout->addWidget(area_type_combo_);

  type_group->setLayout(type_layout);
  main_layout->addWidget(type_group);

  // === 操作按钮 ===
  auto* action_group = new QGroupBox("操作");
  auto* action_layout = new QVBoxLayout;

  close_btn_ = new QPushButton("闭合多边形");
  close_btn_->setStyleSheet(
      "QPushButton { background-color: #4CAF50; color: white; "
      "font-size: 13pt; padding: 8px; border-radius: 4px; }"
      "QPushButton:hover { background-color: #45a049; }");
  action_layout->addWidget(close_btn_);

  cancel_btn_ = new QPushButton("取消绘制");
  cancel_btn_->setStyleSheet(
      "QPushButton { background-color: #f44336; color: white; "
      "font-size: 13pt; padding: 8px; border-radius: 4px; }"
      "QPushButton:hover { background-color: #da190b; }");
  action_layout->addWidget(cancel_btn_);

  action_group->setLayout(action_layout);
  main_layout->addWidget(action_group);

  // === 状态显示 ===
  auto* status_group = new QGroupBox("状态");
  auto* status_layout = new QVBoxLayout;

  status_label_ = new QLabel("等待连接...");
  status_label_->setStyleSheet(
      "QLabel { background-color: #f0f0f0; padding: 6px; "
      "border: 1px solid #ccc; border-radius: 4px; }");
  status_label_->setWordWrap(true);
  status_layout->addWidget(status_label_);

  vertex_label_ = new QLabel("提示: 在地图上点击添加顶点");
  vertex_label_->setStyleSheet("QLabel { color: #666; }");
  status_layout->addWidget(vertex_label_);

  status_group->setLayout(status_layout);
  main_layout->addWidget(status_group);

  main_layout->addStretch();
  setLayout(main_layout);

  // 连接信号
  connect(area_type_combo_,
          static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
          this, &PolygonEditPanel::onAreaTypeChanged);
  connect(close_btn_, &QPushButton::clicked, this, &PolygonEditPanel::onClosePolygon);
  connect(cancel_btn_, &QPushButton::clicked, this, &PolygonEditPanel::onCancelPolygon);
}

void PolygonEditPanel::onAreaTypeChanged(int index) {
  if (!node_) return;
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = area_type_combo_->itemData(index).toString().toStdString();
  area_type_pub_->publish(std::move(msg));
}

void PolygonEditPanel::onClosePolygon() {
  if (!node_) return;

  if (!close_client_->wait_for_service(std::chrono::seconds(1))) {
    status_label_->setText("错误: close_polygon 服务不可用");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = close_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future,
          std::chrono::seconds(3)) == rclcpp::FutureReturnCode::SUCCESS) {
    auto result = future.get();
    status_label_->setText(result->success
        ? QString::fromStdString(result->message)
        : "失败: " + QString::fromStdString(result->message));
  } else {
    status_label_->setText("错误: 服务调用超时");
  }
}

void PolygonEditPanel::onCancelPolygon() {
  if (!node_) return;

  if (!cancel_client_->wait_for_service(std::chrono::seconds(1))) {
    status_label_->setText("错误: cancel_polygon 服务不可用");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = cancel_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future,
          std::chrono::seconds(3)) == rclcpp::FutureReturnCode::SUCCESS) {
    auto result = future.get();
    status_label_->setText(result->success
        ? QString::fromStdString(result->message)
        : "提示: " + QString::fromStdString(result->message));
  } else {
    status_label_->setText("错误: 服务调用超时");
  }
}

void PolygonEditPanel::updateStatusLabel(QString text) {
  status_label_->setText(text);
}

}  // namespace polygon_manager

PLUGINLIB_EXPORT_CLASS(polygon_manager::PolygonEditPanel, rviz_common::Panel)
