// Copyright (c) 2026 Mohamed Abdelkader
// SPDX-License-Identifier: MIT
//
// RViz panel: drive mav_controllers_ros' trajectory_test_node from the
// ground station -- pick a shape, set speed, start and stop.
//
// The panel is a remote control for that node and nothing more. It calls the
// node's existing ~/start and ~/stop Trigger services and writes the node's
// own parameters; it never publishes a setpoint and never touches the
// controller's gains. Every safety decision stays where it already lives: the
// node's engagement gating, closed-form feasibility derating, geofence
// pre-check and runtime abort. A start this panel requests is accepted or
// refused on exactly the same terms as one typed at a terminal.

#ifndef GEO_TUNER__RVIZ__TRAJECTORY_TEST_PANEL_HPP_
#define GEO_TUNER__RVIZ__TRAJECTORY_TEST_PANEL_HPP_

#include <rviz_common/panel.hpp>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QWidget>

#include <vector>

#include <chrono>
#include <map>
#include <mutex>
#include <string>

class QTimer;

namespace geo_tuner::panels
{

class NamespaceSelector;

class TrajectoryTestPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TrajectoryTestPanel(QWidget * parent = nullptr);
  ~TrajectoryTestPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

protected:
  /// One column when the dock is narrow, two when it is wide, so the
  /// panel can be dragged thin without stealing the 3D view.
  void resizeEvent(QResizeEvent * event) override;

private Q_SLOTS:
  void applyNamespace();
  void refresh();
  void onStart();
  void onStop();
  void onApplyPlanParams();
  /// Show only the plan fields the selected shape actually uses.
  void updatePlanFieldVisibility();

private:
  void connectNode();
  QString prefix() const;
  void relayout(int columns);
  void setResult(const QString & text, bool ok);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr health_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_, stop_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;

  // Filled from ROS callbacks (possibly off the Qt thread), read in refresh().
  std::mutex mutex_;
  bool health_received_{false};
  std::chrono::steady_clock::time_point health_arrival_;
  uint8_t level_{0};
  std::string message_;
  std::map<std::string, std::string> values_;
  QString pending_result_;
  bool pending_result_ok_{false};
  bool have_pending_result_{false};

  NamespaceSelector * ns_selector_{nullptr};
  QLabel * banner_{nullptr};
  QLabel * blocked_{nullptr};
  QLabel * result_{nullptr};
  QLabel * plan_report_{nullptr};
  QLabel * frame_hint_{nullptr};

  QComboBox * type_box_{nullptr};
  QComboBox * yaw_box_{nullptr};
  QDoubleSpinBox * speed_spin_{nullptr};
  QDoubleSpinBox * radius_spin_{nullptr};
  QDoubleSpinBox * width_spin_{nullptr};
  QDoubleSpinBox * cx_spin_{nullptr};
  QDoubleSpinBox * cy_spin_{nullptr};
  QDoubleSpinBox * cz_spin_{nullptr};
  QDoubleSpinBox * sp_x_spin_{nullptr};
  QDoubleSpinBox * sp_y_spin_{nullptr};
  QDoubleSpinBox * sp_z_spin_{nullptr};
  QDoubleSpinBox * yaw_spin_{nullptr};
  QDoubleSpinBox * goto_speed_spin_{nullptr};
  QDoubleSpinBox * goto_accel_spin_{nullptr};
  QCheckBox * relative_check_{nullptr};

  /// Plan rows as (label, field), so a row can be hidden whole.
  std::vector<std::pair<QLabel *, QWidget *>> plan_rows_;
  std::map<std::string, size_t> plan_row_index_;
  QPushButton * apply_params_button_{nullptr};
  QPushButton * start_button_{nullptr};
  QPushButton * stop_button_{nullptr};

  std::map<std::string, QLabel *> fields_;
  std::vector<QWidget *> groups_;
  QGridLayout * grid_{nullptr};
  int group_row0_{0};
  int columns_{0};

  QTimer * timer_{nullptr};
};

}  // namespace geo_tuner::panels

#endif  // GEO_TUNER__RVIZ__TRAJECTORY_TEST_PANEL_HPP_
