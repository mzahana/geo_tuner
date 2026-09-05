#include "geo_tuner/rviz/controller_health_panel.hpp"
#include "geo_tuner/rviz/namespace_selector.hpp"
#include "geo_tuner/rviz/sparkline_widget.hpp"

#include <rviz_common/display_context.hpp>

#include <QFont>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QResizeEvent>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <vector>

namespace geo_tuner::panels
{

namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// A status topic that stops arriving is a failure in its own right: the
// panel must not keep showing the last frame as if it were live.
constexpr double kStaleAfter = 2.0;

// Colours read on both light and dark RViz themes.
const char * kOkStyle = "background-color:#1f7a34; color:white; padding:6px; border-radius:3px;";
const char * kWarnStyle = "background-color:#b37400; color:white; padding:6px; border-radius:3px;";
const char * kErrStyle = "background-color:#a11d1d; color:white; padding:6px; border-radius:3px;";
const char * kStaleStyle = "background-color:#4a4a4a; color:#dddddd; padding:6px; border-radius:3px;";

std::vector<double> parseTriple(const std::string & s)
{
  std::vector<double> out;
  std::stringstream ss(s);
  std::string item;
  while(std::getline(ss, item, ','))
  {
    try {
      out.push_back(std::stod(item));
    } catch(const std::exception &) {
      out.push_back(kNaN);
    }
  }
  return out;
}

QString fmt(double v, int prec = 2, const QString & suffix = QString())
{
  if(!std::isfinite(v))
    return QString("-");
  return QString::number(v, 'f', prec) + suffix;
}

}  // namespace

double StatusSnapshot::age_s() const
{
  if(!received)
    return std::numeric_limits<double>::infinity();
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - arrival).count();
}

double StatusSnapshot::num(const std::string & key, double fallback) const
{
  const auto it = values.find(key);
  if(it == values.end())
    return fallback;
  if(it->second == "inf")
    return std::numeric_limits<double>::infinity();
  try {
    return std::stod(it->second);
  } catch(const std::exception &) {
    return fallback;
  }
}

std::string StatusSnapshot::str(const std::string & key, const std::string & fallback) const
{
  const auto it = values.find(key);
  return (it == values.end()) ? fallback : it->second;
}

bool StatusSnapshot::flag(const std::string & key, bool fallback) const
{
  const auto it = values.find(key);
  return (it == values.end()) ? fallback : (it->second == "true");
}

ControllerHealthPanel::ControllerHealthPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  // Two columns: a single tall stack of groups turns the panel into a
  // scroll, which defeats the point of a glance-at-it field display.
  auto * root = new QGridLayout(this);
  grid_ = root;
  root->setContentsMargins(4, 4, 4, 4);
  root->setSpacing(4);
  root->setColumnStretch(0, 1);
  root->setColumnStretch(1, 1);
  int grid_row = 0;

  // The same stack runs bare in the field and under /interceptor in the
  // d2dtracker SITL, so the prefix has to be settable and to persist in
  // the saved RViz config.
  ns_selector_ = new NamespaceSelector(this);
  root->addWidget(ns_selector_, grid_row++, 0, 1, 2);

  banner_ = new QLabel("waiting for controller status", this);
  QFont bf = banner_->font();
  bf.setPointSizeF(bf.pointSizeF() + 3.0);
  bf.setBold(true);
  banner_->setFont(bf);
  banner_->setAlignment(Qt::AlignCenter);
  banner_->setStyleSheet(kStaleStyle);
  banner_->setWordWrap(true);
  root->addWidget(banner_, grid_row++, 0, 1, 2);

  link_label_ = new QLabel("-", this);
  link_label_->setAlignment(Qt::AlignCenter);
  root->addWidget(link_label_, grid_row++, 0, 1, 2);

  auto make_group = [&](const char * title, const std::vector<std::pair<const char *, const char *>> & rows) {
    auto * box = new QGroupBox(title, this);
    auto * grid = new QGridLayout(box);
    grid->setContentsMargins(6, 4, 6, 4);
    grid->setVerticalSpacing(2);
    int r = 0;
    for(const auto & kv : rows)
    {
      auto * name = new QLabel(kv.second, box);
      auto * value = new QLabel("-", box);
      QFont vf = value->font();
      vf.setFamily("Monospace");
      value->setFont(vf);
      value->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
      grid->addWidget(name, r, 0);
      grid->addWidget(value, r, 1);
      fields_[kv.first] = value;
      ++r;
    }
    grid->setColumnStretch(1, 1);
    return box;
  };

  // Deliberately few rows. Everything here answers one of two questions --
  // is it alive and engaged, is it flying well -- and anything that answered
  // neither was removed: it competes for the same screen as the 3D view.
  auto * status_box = make_group("Status", {
    {"px4_mode", "PX4 / armed"},
    {"streams", "odom / setpoint"},
    {"flags", "motors / hold / sat"},
  });

  auto * tracking_box = make_group("Tracking", {
    {"pos_err", "position error"},
    {"pos_err_axes", "err x / y / z"},
    {"tilt", "tilt"},
    {"throttle", "throttle (pred hover)"},
    {"thrust_scale", "thrust scale est"},
  });

  pos_err_plot_ = new SparklineWidget(this);
  pos_err_plot_->setLabel("position error [m]");
  pos_err_plot_->setColor(QColor(70, 160, 230));
  pos_err_plot_->setMinSpan(0.2);

  groups_ = {status_box, tracking_box, pos_err_plot_};
  group_row0_ = grid_row;
  relayout(2);

  connect(ns_selector_, &NamespaceSelector::applied, this, &ControllerHealthPanel::applyNamespace);

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &ControllerHealthPanel::refresh);
  timer_->start(200);
}

ControllerHealthPanel::~ControllerHealthPanel() = default;

void ControllerHealthPanel::relayout(int columns)
{
  if(columns == columns_ || !grid_)
    return;
  columns_ = columns;

  for(auto * w : groups_)
    grid_->removeWidget(w);

  int i = 0;
  for(auto * w : groups_)
  {
    const int row = group_row0_ + i / columns;
    const int col = i % columns;
    grid_->addWidget(w, row, col, 1, (columns == 1) ? 2 : 1);
    ++i;
  }
  grid_->setColumnStretch(1, (columns == 1) ? 0 : 1);
  grid_->setRowStretch(group_row0_ + (static_cast<int>(groups_.size()) + columns - 1) / columns, 1);
}

void ControllerHealthPanel::resizeEvent(QResizeEvent * event)
{
  rviz_common::Panel::resizeEvent(event);
  // Hysteresis around the switch point, so dragging the dock edge across
  // the threshold does not make the panel flicker between layouts.
  const int w = width();
  if(columns_ == 2 && w < 400)
    relayout(1);
  else if(columns_ != 2 && w > 440)
    relayout(2);
}

void ControllerHealthPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  subscribe();
}

QString ControllerHealthPanel::topicPrefix() const
{
  return ns_selector_->prefix(node_);
}

void ControllerHealthPanel::subscribe()
{
  if(!node_)
    return;

  const std::string prefix = topicPrefix().toStdString();

  // Best effort with a shallow depth: this is a live gauge on a field
  // datalink, where the newest sample matters and a backlog does not.
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

  ctrl_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
    prefix + "geometric_controller/status", qos,
    [this](diagnostic_msgs::msg::DiagnosticStatus::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      ctrl_.received = true;
      ctrl_.arrival = std::chrono::steady_clock::now();
      ctrl_.level = msg->level;
      ctrl_.message = msg->message;
      ctrl_.values.clear();
      for(const auto & kv : msg->values)
        ctrl_.values[kv.key] = kv.value;
    });

  mavros_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
    prefix + "geometric_mavros/status", qos,
    [this](diagnostic_msgs::msg::DiagnosticStatus::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      mavros_.received = true;
      mavros_.arrival = std::chrono::steady_clock::now();
      mavros_.level = msg->level;
      mavros_.message = msg->message;
      mavros_.values.clear();
      for(const auto & kv : msg->values)
        mavros_.values[kv.key] = kv.value;
    });

#ifdef GEO_TUNER_HAVE_MAVROS_MSGS
  state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
    prefix + "mavros/state", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
    [this](mavros_msgs::msg::State::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      state_received_ = true;
      state_arrival_ = std::chrono::steady_clock::now();
      px4_mode_ = msg->mode;
      px4_armed_ = msg->armed;
    });
#endif
}

void ControllerHealthPanel::applyNamespace()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ctrl_ = StatusSnapshot();
    mavros_ = StatusSnapshot();
    state_received_ = false;
  }
  pos_err_plot_->clear();
  subscribe();
  Q_EMIT configChanged();
}

void ControllerHealthPanel::refresh()
{
  StatusSnapshot ctrl, mavros;
  bool state_ok = false;
  std::string mode;
  bool armed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ctrl = ctrl_;
    mavros = mavros_;
    mode = px4_mode_;
    armed = px4_armed_;
    state_ok = state_received_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - state_arrival_).count() < kStaleAfter;
  }

  const double ctrl_age = ctrl.age_s();
  const double mavros_age = mavros.age_s();
  const bool ctrl_live = ctrl.received && ctrl_age < kStaleAfter;
  const bool mavros_live = mavros.received && mavros_age < kStaleAfter;

  // --- banner: worst of what is actually live -----------------------
  if(!ctrl_live && !mavros_live)
  {
    banner_->setStyleSheet(kStaleStyle);
    banner_->setText(ctrl.received || mavros.received ? "NO DATA - status topics stopped"
                                                      : "NO DATA - waiting for controller");
  }
  else
  {
    uint8_t level = 0;
    QString text;
    if(ctrl_live)
    {
      level = ctrl.level;
      text = QString::fromStdString(ctrl.message);
    }
    if(mavros_live && (!ctrl_live || mavros.level > level))
    {
      level = mavros.level;
      text = QString::fromStdString(mavros.message);
    }
    // A half-dead pair is itself a warning: one node talking and the
    // other silent means part of the chain is gone.
    if(ctrl_live != mavros_live)
    {
      level = std::max<uint8_t>(level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
      text += ctrl_live ? "  (+ mavros bridge silent)" : "  (+ controller silent)";
    }
    banner_->setText(text.toUpper());
    banner_->setStyleSheet(level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR ? kErrStyle
                           : level >= diagnostic_msgs::msg::DiagnosticStatus::WARN ? kWarnStyle
                                                                                   : kOkStyle);
  }

  auto age_text = [](bool live, bool received, double age) {
    if(!received)
      return QString("--");
    return live ? QString("%1s").arg(age, 0, 'f', 1) : QString("STALE %1s").arg(age, 0, 'f', 0);
  };
  link_label_->setText(QString("status age: controller %1 | mavros %2 | topics: %3")
                         .arg(age_text(ctrl_live, ctrl.received, ctrl_age))
                         .arg(age_text(mavros_live, mavros.received, mavros_age))
                         .arg(topicPrefix() + "geometric_*/status"));

  auto set = [this](const char * key, const QString & text) {
    const auto it = fields_.find(key);
    if(it != fields_.end())
      it->second->setText(text);
  };
  auto set_flag = [this](const char * key, bool value, bool bad_when_true) {
    const auto it = fields_.find(key);
    if(it == fields_.end())
      return;
    it->second->setText(value ? "YES" : "no");
    const bool bad = (value == bad_when_true);
    it->second->setStyleSheet(bad ? "color:#d05050; font-weight:bold;" : "");
  };

  if(!ctrl_live)
  {
    for(const char * k : {"streams", "flags", "pos_err", "pos_err_axes", "tilt"})
    {
      set(k, "-");
      fields_[k]->setStyleSheet("");
    }
  }
  else
  {
    set("streams", QString("%1 / %2 Hz")
          .arg(fmt(ctrl.num("odom_rate_hz", kNaN), 0))
          .arg(fmt(ctrl.num("setpoint_rate_hz", kNaN), 0)));

    // Three booleans on one line, and only coloured when one is bad.
    const bool motors = ctrl.flag("motors_enabled");
    const bool hold = ctrl.flag("hold_active");
    const bool sat = ctrl.flag("saturated");
    const auto flags_it = fields_.find("flags");
    if(flags_it != fields_.end())
    {
      flags_it->second->setText(QString("%1 / %2 / %3")
                                  .arg(motors ? "on" : "OFF")
                                  .arg(hold ? "HOLD" : "-")
                                  .arg(sat ? "SAT" : "-"));
      flags_it->second->setStyleSheet((hold || sat || !motors)
                                        ? "color:#d05050; font-weight:bold;" : "");
    }

    const double odom_age = ctrl.num("odom_age_s", kNaN);
    const double odom_timeout = ctrl.num("odom_timeout_s", 0.3);
    const bool tracking_frozen = !ctrl.flag("have_odom", true) ||
                                 !std::isfinite(odom_age) || odom_age > odom_timeout;
    const char * tracking_style = tracking_frozen ? "color:#808080; font-style:italic;" : "";
    for(const char * k : {"pos_err", "pos_err_axes", "tilt"})
      fields_[k]->setStyleSheet(tracking_style);

    const double pos_err = ctrl.num("pos_err_norm_m", kNaN);
    set("pos_err", fmt(pos_err, 3, " m"));
    set("pos_err_axes", QString("%1 / %2 / %3")
          .arg(fmt(ctrl.num("pos_err_x_m", kNaN), 2))
          .arg(fmt(ctrl.num("pos_err_y_m", kNaN), 2))
          .arg(fmt(ctrl.num("pos_err_z_m", kNaN), 2)));
    set("tilt", fmt(ctrl.num("tilt_deg", kNaN), 1, " deg"));

    if(!tracking_frozen)
      pos_err_plot_->push(pos_err);
  }

  if(!mavros_live)
  {
    for(const char * k : {"throttle", "thrust_scale"})
      set(k, "-");
  }
  else
  {
    const double throttle = mavros.num("throttle", kNaN);
    // Actual against predicted hover throttle: the difference IS the
    // thrust-map error, and it is the number that explains a tuner abort
    // for "steady-state offset exceeds trim authority".
    const double hover = mavros.num("hover_throttle_pred", kNaN);
    QString throttle_text = fmt(100.0 * throttle, 0, " %");
    if(std::isfinite(hover))
      throttle_text += QString(" (hover %1 %)").arg(fmt(100.0 * hover, 0));
    if(mavros.flag("setpoints_stopped"))
      throttle_text += " (stopped)";
    else if(mavros.flag("cmd_timeout_active"))
      throttle_text += " (failsafe)";
    set("throttle", throttle_text);

    // Pinned at a clamp means max_thrust itself is wrong by more than the
    // estimator may correct.
    const double scale = mavros.num("thrust_scale_est", kNaN);
    const auto scale_it = fields_.find("thrust_scale");
    if(scale_it != fields_.end())
    {
      // "not updating" matters as much as the number: outside OFFBOARD the
      // estimator is frozen, and a value read then describes an earlier
      // flight, not this one.
      const bool estimating = mavros.flag("thrust_estimator_active");
      scale_it->second->setText(fmt(scale, 3) + (estimating ? "" : "  (frozen)"));
      const bool pinned = std::isfinite(scale) && (scale <= 0.801 || scale >= 1.249);
      scale_it->second->setStyleSheet(pinned ? "color:#d05050; font-weight:bold;"
                                             : (estimating ? "" : "color:#909090;"));
    }
  }

  QString mode_text = state_ok ? QString::fromStdString(mode) : QString("-");
  const bool armed_known = state_ok || mavros_live;
  const bool armed_value = state_ok ? armed : (mavros_live && mavros.flag("armed"));
  const auto mode_it = fields_.find("px4_mode");
  if(mode_it != fields_.end())
  {
    mode_it->second->setText(QString("%1 / %2")
                               .arg(mode_text)
                               .arg(armed_known ? (armed_value ? "ARMED" : "disarmed") : "-"));
    mode_it->second->setStyleSheet(armed_value ? "color:#d05050; font-weight:bold;" : "");
  }
}

void ControllerHealthPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString ns;
  if(config.mapGetString("Namespace", &ns))
  {
    ns_selector_->setNs(ns);
    subscribe();
  }
}

void ControllerHealthPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Namespace", ns_selector_->ns());
}

}  // namespace geo_tuner::panels

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(geo_tuner::panels::ControllerHealthPanel, rviz_common::Panel)
