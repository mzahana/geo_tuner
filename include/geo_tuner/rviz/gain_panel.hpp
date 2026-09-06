// Copyright (c) 2026 Mohamed Abdelkader
// SPDX-License-Identifier: MIT
//
// RViz panel: read and change the geometric controller's gains in flight,
// expressed as wn and zeta rather than raw kx/kv.
//
// This panel writes. Everything about it is built around that:
//   - edits are behind an interlock that resets to OFF every time the panel
//     is loaded, so a saved RViz config never comes up armed;
//   - a value outside the design caps (attitude-loop separation, latency
//     margin) cannot be applied at all;
//   - a single apply cannot change a gain by more than a bounded factor;
//   - the previous and the baseline gain sets are kept, so a bad change is
//     one click from being undone;
//   - it refuses to write while a geo_tuner session is running, so the two
//     never fight over the same parameters.
//
// A parameter write is runtime-only. Persisting it is a separate, explicit
// act: the Save button calls gain_saver ON THE VEHICLE, which writes the
// override YAML the launch files read at the next boot.

#ifndef GEO_TUNER__RVIZ__GAIN_PANEL_HPP_
#define GEO_TUNER__RVIZ__GAIN_PANEL_HPP_

#include <rviz_common/panel.hpp>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QTextEdit>

#include <array>
#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <vector>

class QTimer;

namespace geo_tuner::panels
{

class NamespaceSelector;

/// One gain set, as the controller holds it.
struct GainSet
{
  std::array<double, 3> kx{{0.0, 0.0, 0.0}};
  std::array<double, 3> kv{{0.0, 0.0, 0.0}};
  double attctrl_tau{0.0};
  bool valid{false};
};

class GainPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit GainPanel(QWidget * parent = nullptr);
  ~GainPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

  /// Embedded inside another panel (the Geo Field Tune tab): hide this
  /// panel's own namespace row -- the host's selector drives it instead.
  void setEmbedded(bool embedded);
  /// Take over the namespace from the host panel's selector.
  void adoptNamespace(const QString & ns);

protected:
  void resizeEvent(QResizeEvent * event) override;

private Q_SLOTS:
  void applyNamespace();
  void refresh();
  void onApply();
  void onRevert();
  void onSaveToVehicle();
  void onInterlockToggled(bool on);
  void onEditsChanged();

private:
  void connectNode();
  QString prefix() const;
  void relayout(int columns);
  void applyGains(const GainSet & gains, const QString & what);
  void log(const QString & text, bool ok = true);
  /// Upper bound on wn from the design rules, given the live attctrl_tau.
  double wnCap(double attctrl_tau, bool vertical) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr ctrl_sub_, mavros_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tuner_sub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr save_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;

  std::mutex mutex_;
  bool ctrl_received_{false}, mavros_received_{false};
  std::chrono::steady_clock::time_point ctrl_arrival_, mavros_arrival_, tuner_arrival_;
  bool tuner_seen_{false};
  std::map<std::string, std::string> ctrl_values_, mavros_values_;
  QString pending_log_;
  bool pending_log_ok_{true};
  bool have_pending_log_{false};

  GainSet live_;       // what the controller reports right now
  GainSet baseline_;   // captured when the panel first saw the controller
  GainSet previous_;   // the set in force before the last apply
  bool edits_dirty_{false};

  NamespaceSelector * ns_selector_{nullptr};
  QLabel * banner_{nullptr};
  QLabel * caps_label_{nullptr};
  QCheckBox * interlock_{nullptr};
  QCheckBox * thrust_correct_{nullptr};
  QPushButton * apply_button_{nullptr};
  QPushButton * revert_button_{nullptr};
  QPushButton * save_button_{nullptr};
  QPushButton * reload_button_{nullptr};
  std::array<QDoubleSpinBox *, 3> wn_spin_{{nullptr, nullptr, nullptr}};
  std::array<QDoubleSpinBox *, 3> zeta_spin_{{nullptr, nullptr, nullptr}};
  std::array<QLabel *, 3> live_label_{{nullptr, nullptr, nullptr}};
  std::array<QLabel *, 3> raw_label_{{nullptr, nullptr, nullptr}};
  std::map<std::string, QLabel *> fields_;
  QTextEdit * log_view_{nullptr};

  std::vector<QWidget *> groups_;
  QGridLayout * grid_{nullptr};
  int group_row0_{0};
  int columns_{0};

  QTimer * timer_{nullptr};
};

}  // namespace geo_tuner::panels

#endif  // GEO_TUNER__RVIZ__GAIN_PANEL_HPP_
