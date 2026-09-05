// Copyright (c) 2026 Mohamed Abdelkader
// SPDX-License-Identifier: MIT
//
// RViz panel: geometric controller health at a glance, for field flying.
//
// Read-only. It subscribes to the two low-rate DiagnosticStatus topics the
// controller and its mavros bridge publish (geometric_controller/status and
// geometric_mavros/status) plus mavros/state, and never publishes, calls a
// service, or writes a parameter -- nothing in this panel can reach the
// flight controller.

#ifndef GEO_TUNER__RVIZ__CONTROLLER_HEALTH_PANEL_HPP_
#define GEO_TUNER__RVIZ__CONTROLLER_HEALTH_PANEL_HPP_

#include <rviz_common/panel.hpp>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

// mavros_msgs is optional: it only supplies the PX4 flight-mode readout.
// Everything else comes from the two DiagnosticStatus topics, so the panel
// still builds and runs on a ground station without mavros installed.
#ifdef GEO_TUNER_HAVE_MAVROS_MSGS
#include <mavros_msgs/msg/state.hpp>
#endif

#include <QGridLayout>
#include <QLabel>

class QTimer;

#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace geo_tuner::panels
{

class SparklineWidget;
class NamespaceSelector;

/// One received DiagnosticStatus plus its arrival time, so the panel can
/// tell "publisher says OK" from "publisher stopped talking".
struct StatusSnapshot
{
  bool received{false};
  std::chrono::steady_clock::time_point arrival;
  uint8_t level{0};
  std::string message;
  std::map<std::string, std::string> values;

  double age_s() const;
  /// Value for a key, or the fallback when the key is absent/unparsable.
  double num(const std::string & key, double fallback) const;
  std::string str(const std::string & key, const std::string & fallback = "-") const;
  bool flag(const std::string & key, bool fallback = false) const;
};

class ControllerHealthPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ControllerHealthPanel(QWidget * parent = nullptr);
  ~ControllerHealthPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

protected:
  /// Panels are docked next to the 3D view, so they must be able to get
  /// out of the way: narrow means one column of groups, wide means two.
  void resizeEvent(QResizeEvent * event) override;

private Q_SLOTS:
  /// (Re)create subscriptions under the namespace in the text box.
  void applyNamespace();
  /// Repaint from the latest snapshots; runs in the Qt thread.
  void refresh();

private:
  void subscribe();
  QString topicPrefix() const;
  /// Re-flow the group boxes into `columns` columns.
  void relayout(int columns);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr ctrl_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr mavros_sub_;
#ifdef GEO_TUNER_HAVE_MAVROS_MSGS
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
#endif

  // Subscription callbacks may run off the Qt thread, so they only fill
  // these under the mutex; every widget touch happens in refresh().
  std::mutex mutex_;
  StatusSnapshot ctrl_, mavros_;
  bool state_received_{false};
  std::chrono::steady_clock::time_point state_arrival_;
  std::string px4_mode_;
  bool px4_armed_{false};

  NamespaceSelector * ns_selector_{nullptr};

  QLabel * banner_{nullptr};
  QLabel * link_label_{nullptr};

  std::map<std::string, QLabel *> fields_;
  std::vector<QWidget *> groups_;
  QGridLayout * grid_{nullptr};
  int group_row0_{0};
  int columns_{0};
  SparklineWidget * pos_err_plot_{nullptr};

  QTimer * timer_{nullptr};
};

}  // namespace geo_tuner::panels

#endif  // GEO_TUNER__RVIZ__CONTROLLER_HEALTH_PANEL_HPP_
