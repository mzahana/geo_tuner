// Copyright (c) 2026 Mohamed Abdelkader
// SPDX-License-Identifier: MIT
//
// RViz panel: run a geo_tuner auto-tuning session from the ground.
//
// The conductor flies the vehicle -- step injections, gain updates, its own
// safety monitor. This panel starts it, watches it, and stops it. It holds no
// tuning logic of its own: every decision, and every abort, stays in the node
// that is actually flying.
//
// The field workflow is start -> watch -> decide, and the layout follows it:
// banner, START/ABORT, one status line, then (once buckets finish) an
// old -> new result table with exactly two decisions -- Keep & save, or
// Revert. Everything an operator touches rarely (step envelope, the session
// log) lives behind an Advanced disclosure.

#ifndef GEO_TUNER__RVIZ__TUNER_PANEL_HPP_
#define GEO_TUNER__RVIZ__TUNER_PANEL_HPP_

#include <rviz_common/panel.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QTextEdit>
#include <QToolButton>

#include <array>
#include <chrono>
#include <map>
#include <mutex>
#include <string>

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

  QString currentNamespace() const;

Q_SIGNALS:
  /// The operator applied a new namespace; an embedding panel can forward it
  /// to sibling panels so one box drives them all.
  void namespaceApplied(const QString & ns);

private Q_SLOTS:
  void applyNamespace();
  void refresh();
  /// Push the step amplitudes to the conductor (validated node-side).
  void applySteps();
  /// Push the working altitude to the conductor, after a confirmation that
  /// spells out what the vehicle will and will not do with it.
  void applyAltitude();

private:
  void connectNode();
  QString prefix() const;
  /// Call one of the conductor's Trigger services, with an optional confirm.
  void callService(const QString & name,
                   rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
                   const QString & confirm);
  void onStart();
  void onKeepAndSave();
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
  /// gain_saver on the vehicle: Keep & save persists the accepted gains to
  /// the override YAML in the same request flow the Gains tab uses.
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr save_client_;
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
  /// Last state string seen in health; START after ABORT/DONE chains a
  /// reset first, so the operator never needs a separate Reset button.
  std::string last_state_;

  NamespaceSelector * ns_selector_{nullptr};
  QLabel * banner_{nullptr};
  QPushButton * start_button_{nullptr};
  QPushButton * abort_button_{nullptr};
  /// Session completion, fed by the conductor's steps_done/steps_total/
  /// progress_pct health keys -- the at-a-glance answer to "how much
  /// longer", for an operator who has not read the tuner's internals.
  QProgressBar * progress_bar_{nullptr};
  QLabel * progress_line_{nullptr};
  QLabel * vehicle_line_{nullptr};
  QLabel * waiting_line_{nullptr};

  QGroupBox * result_box_{nullptr};
  // Rows x, y, z, yaw: old | -> | new | outcome.
  std::array<QLabel *, 4> result_old_{};
  std::array<QLabel *, 4> result_new_{};
  std::array<QLabel *, 4> result_note_{};
  QPushButton * keep_button_{nullptr};
  QPushButton * revert_button_{nullptr};

  QToolButton * advanced_toggle_{nullptr};
  QWidget * advanced_box_{nullptr};
  // The working altitude: the height above ground the session refuses to
  // start below. The conductor never climbs to reach it -- it is a gate on
  // where the pilot hands the vehicle over, not a commanded altitude.
  QDoubleSpinBox * min_alt_spin_{nullptr};
  QPushButton * apply_alt_button_{nullptr};
  QLabel * altitude_line_{nullptr};
  bool alt_dirty_{false};
  QDoubleSpinBox * step_xy_spin_{nullptr};
  QDoubleSpinBox * step_z_spin_{nullptr};
  QDoubleSpinBox * step_yaw_spin_{nullptr};
  QPushButton * apply_steps_button_{nullptr};
  QLabel * envelope_label_{nullptr};
  QTextEdit * log_view_{nullptr};
  /// True while the operator has typed values not yet applied: the live
  /// values from the conductor must not overwrite an edit in progress.
  bool steps_dirty_{false};

  QTimer * timer_{nullptr};
};

}  // namespace geo_tuner::panels

#endif  // GEO_TUNER__RVIZ__TUNER_PANEL_HPP_
