#ifndef POLYGON_MANAGER__POLYGON_EDIT_PANEL_HPP_
#define POLYGON_MANAGER__POLYGON_EDIT_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <QWidget>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QComboBox>
#include <QGroupBox>
#include <QString>

namespace polygon_manager {

class PolygonEditPanel : public rviz_common::Panel {
  Q_OBJECT

 public:
  explicit PolygonEditPanel(QWidget* parent = nullptr);
  ~PolygonEditPanel() override = default;

  void onInitialize() override;

 Q_SIGNALS:
  void statusChanged(QString text);

 private Q_SLOTS:
  void onAreaTypeChanged(int index);
  void onClosePolygon();
  void onCancelPolygon();
  void updateStatusLabel(QString text);

 private:
  void setupUI();

  // ROS
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr area_type_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr close_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cancel_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;

  // UI
  QComboBox* area_type_combo_;
  QPushButton* close_btn_;
  QPushButton* cancel_btn_;
  QLabel* status_label_;
  QLabel* vertex_label_;
};

}  // namespace polygon_manager

#endif  // POLYGON_MANAGER__POLYGON_EDIT_PANEL_HPP_
