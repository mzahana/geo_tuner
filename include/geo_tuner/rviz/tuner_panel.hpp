// Copyright (c) 2026 Mohamed Abdelkader
// SPDX-License-Identifier: MIT
//
// RViz panel: run a geo_tuner auto-tuning session from the ground.
//
// The conductor flies the vehicle -- step injections, gain updates, its own
// safety monitor. This panel starts it, watches it, and stops it. It holds no
// tuning logic of its own: every decision, and every abort, stays in the node
// that is actually flying.

#ifndef GEO_TUNER__RVIZ__TUNER_PANEL_HPP_
#define GEO_TUNER__RVIZ__TUNER_PANEL_HPP_

#include <rviz_common/panel.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QTextEdit>

#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <vector>

class QTimer;

namespace geo_tuner::panels
{

class NamespaceSelector;

class TunerPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TunerPanel(QWidget * parent = nullptr);
  ~TunerPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

protected:
  void resizeEvent(QResizeEvent * event) override;

private Q_SLOTS:
  void applyNamespace();
  void refresh();
  /// Push the step amplitudes to the conductor (validated node-side).
  void applySteps();

private:
  void connectNode();
  QString prefix() const;
  void relayout(int columns);
  /// Call one of the conductor's Trigger services, with an optional confirm.
  void callService(const QString & name,
                   rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
                   const QString & confirm);
  void log(const QString & text, bool ok = true);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr health_sub_;
  // trajectory_test_node streams setpoints continuously, including while
  // holding. The conductor streams its own. Both reach the controller, on
  // different topics, and it follows whichever arrived last -- so a session
  // must not start while that node is running.
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr traj_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_, abort_client_,
    accept_client_, restore_client_, reset_client_;
  // Step amplitudes are the one thing this panel changes on the conductor.
  // Every value is validated by the node (against safety.max_pos_error and
  // max_step_size) and may be refused -- the panel reports, never overrides.
  std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;

  std::mutex mutex_;
  bool received_{false};
  bool traj_seen_{false};
  std::chrono::steady_clock::time_point traj_arrival_;
  std::chrono::steady_clock::time_point arrival_;
  uint8_t level_{0};
  std::string message_;
  std::map<std::string, std::string> values_;
  QString pending_log_;
  bool pending_log_ok_{true};
  bool have_pending_log_{false};

  NamespaceSelector * ns_selector_{nullptr};
  QLabel * banner_{nullptr};
  QPushButton * start_button_{nullptr};
  QPushButton * abort_button_{nullptr};
  QPushButton * accept_button_{nullptr};
  QPushButton * restore_button_{nullptr};
  QPushButton * reset_button_{nullptr};
  QDoubleSpinBox * step_xy_spin_{nullptr};
  QDoubleSpinBox * step_z_spin_{nullptr};
  QDoubleSpinBox * step_yaw_spin_{nullptr};
  QPushButton * apply_steps_button_{nullptr};
  QLabel * envelope_label_{nullptr};
  /// True while the operator has typed values not yet applied: the live
  /// values from the conductor must not overwrite an edit in progress.
  bool steps_dirty_{false};
  std::map<std::string, QLabel *> fields_;
  QTextEdit * log_view_{nullptr};

  std::vector<QWidget *> groups_;
  QGridLayout * grid_{nullptr};
  int group_row0_{0};
  int columns_{0};
  QTimer * timer_{nullptr};
};

}  // namespace geo_tuner::panels

#endif  // GEO_TUNER__RVIZ__TUNER_PANEL_HPP_
