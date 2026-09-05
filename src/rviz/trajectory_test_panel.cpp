#include "geo_tuner/rviz/trajectory_test_panel.hpp"
#include "geo_tuner/rviz/namespace_selector.hpp"

#include <rviz_common/display_context.hpp>

#include <QFont>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QResizeEvent>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <vector>

namespace geo_tuner::panels
{

namespace
{

constexpr double kStaleAfter = 2.0;

const char * kOkStyle = "background-color:#1f7a34; color:white; padding:6px; border-radius:3px;";
const char * kWarnStyle = "background-color:#b37400; color:white; padding:6px; border-radius:3px;";
const char * kErrStyle = "background-color:#a11d1d; color:white; padding:6px; border-radius:3px;";
const char * kStaleStyle = "background-color:#4a4a4a; color:#dddddd; padding:6px; border-radius:3px;";

}  // namespace

TrajectoryTestPanel::TrajectoryTestPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  // Two columns, for the same reason as the health panel: a single tall
  // stack scrolls, and a control you have to scroll to is a control you
  // cannot reach in a hurry.
  auto * root = new QGridLayout(this);
  grid_ = root;
  root->setContentsMargins(4, 4, 4, 4);
  root->setSpacing(4);
  root->setColumnStretch(0, 1);
  root->setColumnStretch(1, 1);
  int grid_row = 0;

  ns_selector_ = new NamespaceSelector(this);
  root->addWidget(ns_selector_, grid_row++, 0, 1, 2);

  banner_ = new QLabel("waiting for trajectory_test_node", this);
  QFont bf = banner_->font();
  bf.setPointSizeF(bf.pointSizeF() + 3.0);
  bf.setBold(true);
  banner_->setFont(bf);
  banner_->setAlignment(Qt::AlignCenter);
  banner_->setStyleSheet(kStaleStyle);
  banner_->setWordWrap(true);
  root->addWidget(banner_, grid_row++, 0, 1, 2);

  blocked_ = new QLabel("-", this);
  blocked_->setWordWrap(true);
  blocked_->setAlignment(Qt::AlignCenter);
  root->addWidget(blocked_, grid_row++, 0, 1, 2);

  // No preconditions grid: the "blocked" line above already names what is
  // missing, and the Health tab shows mode and arming. Four more rows here
  // were four rows of duplication.

  // --- plan ----------------------------------------------------------
  // Only the fields the chosen shape actually consumes are shown: a
  // "speed" box next to a setpoint move (which is governed by goto_speed)
  // or a radius next to a lemniscate is an invitation to set a number
  // that silently does nothing.
  {
    auto * box = new QGroupBox("Plan", this);
    auto * grid = new QGridLayout(box);
    grid->setContentsMargins(6, 4, 6, 4);
    grid->setVerticalSpacing(3);
    grid->setColumnStretch(1, 1);
    int r = 0;

    auto add_row = [&](const char * key, const QString & text, QWidget * field) {
      auto * label = new QLabel(text, box);
      grid->addWidget(label, r, 0);
      grid->addWidget(field, r, 1);
      plan_row_index_[key] = plan_rows_.size();
      plan_rows_.emplace_back(label, field);
      ++r;
    };
    auto make_spin = [&](double lo, double hi, double step, double value,
                         const QString & suffix, int decimals = 2) {
      auto * spin = new QDoubleSpinBox(box);
      spin->setRange(lo, hi);
      spin->setSingleStep(step);
      spin->setDecimals(decimals);
      spin->setValue(value);
      spin->setSuffix(suffix);
      // Do not let a spin box set the floor on the dock width.
      spin->setMinimumWidth(70);
      spin->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
      return spin;
    };

    type_box_ = new QComboBox(box);
    type_box_->addItems({"setpoint", "circle", "lemniscate"});
    type_box_->setCurrentText("circle");
    add_row("type", "shape", type_box_);

    yaw_box_ = new QComboBox(box);
    yaw_box_->addItems({"hold", "fixed", "tangent", "center"});
    yaw_box_->setCurrentText("tangent");
    add_row("yaw_mode", "yaw mode", yaw_box_);

    // Centre of the orbit. With "relative" ticked these are offsets from
    // the pose at START (0,0,0 = centred on the vehicle, current altitude);
    // unticked they are absolute local-frame coordinates, and z = 0 is the
    // ground, which the geofence floor rejects.
    cx_spin_ = make_spin(-500.0, 500.0, 0.5, 0.0, " m");
    add_row("cx", "centre x", cx_spin_);
    cy_spin_ = make_spin(-500.0, 500.0, 0.5, 0.0, " m");
    add_row("cy", "centre y", cy_spin_);
    cz_spin_ = make_spin(-500.0, 500.0, 0.5, 0.0, " m");
    add_row("cz", "centre z", cz_spin_);

    sp_x_spin_ = make_spin(-500.0, 500.0, 0.5, 0.0, " m");
    add_row("sp_x", "setpoint x", sp_x_spin_);
    sp_y_spin_ = make_spin(-500.0, 500.0, 0.5, 0.0, " m");
    add_row("sp_y", "setpoint y", sp_y_spin_);
    sp_z_spin_ = make_spin(-500.0, 500.0, 0.5, 0.0, " m");
    add_row("sp_z", "setpoint z", sp_z_spin_);

    // Entered in degrees and sent in radians: a yaw box in radians is a
    // reliable way to fly 57 degrees when you meant one.
    yaw_spin_ = make_spin(-180.0, 180.0, 5.0, 0.0, " deg", 1);
    add_row("yaw", "yaw (fixed)", yaw_spin_);

    relative_check_ = new QCheckBox("relative to pose at START", box);
    relative_check_->setChecked(true);
    add_row("relative", "frame", relative_check_);

    frame_hint_ = new QLabel(box);
    frame_hint_->setWordWrap(true);
    frame_hint_->setStyleSheet("color:#909090;");
    grid->addWidget(frame_hint_, r++, 0, 1, 2);

    speed_spin_ = make_spin(0.1, 20.0, 0.25, 2.0, " m/s");
    add_row("speed", "path speed", speed_spin_);

    radius_spin_ = make_spin(0.5, 100.0, 0.5, 3.0, " m");
    add_row("radius", "circle radius", radius_spin_);

    width_spin_ = make_spin(0.5, 100.0, 0.5, 4.0, " m");
    add_row("width", "lemniscate width", width_spin_);

    goto_speed_spin_ = make_spin(0.1, 10.0, 0.25, 1.0, " m/s");
    add_row("goto_speed", "move speed", goto_speed_spin_);

    goto_accel_spin_ = make_spin(0.1, 10.0, 0.25, 1.0, " m/s2");
    add_row("goto_accel", "move accel", goto_accel_spin_);

    apply_params_button_ = new QPushButton("Send plan parameters", box);
    grid->addWidget(apply_params_button_, r++, 0, 1, 2);

    groups_.push_back(box);
  }

  // --- start / stop --------------------------------------------------
  {
    auto * row = new QHBoxLayout();
    start_button_ = new QPushButton("START", this);
    start_button_->setStyleSheet("background-color:#1f7a34; color:white; font-weight:bold; padding:8px;");
    stop_button_ = new QPushButton("STOP", this);
    stop_button_->setStyleSheet("background-color:#a11d1d; color:white; font-weight:bold; padding:8px;");
    row->addWidget(start_button_, 1);
    row->addWidget(stop_button_, 1);
    root->addLayout(row, grid_row++, 0, 1, 2);
  }

  result_ = new QLabel("-", this);
  result_->setWordWrap(true);
  root->addWidget(result_, grid_row++, 0, 1, 2);

  // --- readback ------------------------------------------------------
  {
    auto * box = new QGroupBox("Flying", this);
    auto * grid = new QGridLayout(box);
    grid->setContentsMargins(6, 4, 6, 4);
    grid->setVerticalSpacing(2);
    const std::vector<std::pair<const char *, const char *>> rows = {
      {"phase", "phase"},
      {"speed", "speed done / asked"},
      {"fence", "fence radius / alt"},
    };
    int r = 0;
    for(const auto & kv : rows)
    {
      auto * value = new QLabel("-", box);
      value->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
      value->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
      grid->addWidget(new QLabel(kv.second, box), r, 0);
      grid->addWidget(value, r, 1);
      fields_[kv.first] = value;
      ++r;
    }
    grid->setColumnStretch(1, 1);
    groups_.push_back(box);
  }

  plan_report_ = new QLabel("-", this);
  plan_report_->setWordWrap(true);
  plan_report_->setStyleSheet("color:#909090;");
  plan_report_->setAlignment(Qt::AlignTop);
  groups_.push_back(plan_report_);

  // START/STOP sit directly under the banner, above the flowing groups, so
  // they never end up below the fold in a short dock.
  group_row0_ = grid_row;
  relayout(2);

  connect(ns_selector_, &NamespaceSelector::applied, this, &TrajectoryTestPanel::applyNamespace);
  connect(start_button_, &QPushButton::clicked, this, &TrajectoryTestPanel::onStart);
  connect(stop_button_, &QPushButton::clicked, this, &TrajectoryTestPanel::onStop);
  connect(apply_params_button_, &QPushButton::clicked, this, &TrajectoryTestPanel::onApplyPlanParams);
  connect(type_box_, &QComboBox::currentTextChanged, this,
          &TrajectoryTestPanel::updatePlanFieldVisibility);
  connect(yaw_box_, &QComboBox::currentTextChanged, this,
          &TrajectoryTestPanel::updatePlanFieldVisibility);
  connect(relative_check_, &QCheckBox::toggled, this,
          &TrajectoryTestPanel::updatePlanFieldVisibility);
  updatePlanFieldVisibility();

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &TrajectoryTestPanel::refresh);
  timer_->start(200);
}

TrajectoryTestPanel::~TrajectoryTestPanel() = default;

void TrajectoryTestPanel::relayout(int columns)
{
  if(columns == columns_ || !grid_)
    return;
  columns_ = columns;

  for(auto * w : groups_)
    grid_->removeWidget(w);

  int i = 0;
  for(auto * w : groups_)
  {
    grid_->addWidget(w, group_row0_ + i / columns, i % columns, 1, (columns == 1) ? 2 : 1);
    ++i;
  }
  grid_->setColumnStretch(1, (columns == 1) ? 0 : 1);
  grid_->setRowStretch(group_row0_ + (static_cast<int>(groups_.size()) + columns - 1) / columns, 1);
}

void TrajectoryTestPanel::resizeEvent(QResizeEvent * event)
{
  rviz_common::Panel::resizeEvent(event);
  const int w = width();
  if(columns_ == 2 && w < 400)
    relayout(1);
  else if(columns_ != 2 && w > 440)
    relayout(2);
}

void TrajectoryTestPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  connectNode();
}

QString TrajectoryTestPanel::prefix() const
{
  return ns_selector_->prefix(node_);
}

void TrajectoryTestPanel::connectNode()
{
  if(!node_)
    return;

  const std::string p = prefix().toStdString();

  health_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
    p + "trajectory_test/health", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
    [this](diagnostic_msgs::msg::DiagnosticStatus::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      health_received_ = true;
      health_arrival_ = std::chrono::steady_clock::now();
      level_ = msg->level;
      message_ = msg->message;
      values_.clear();
      for(const auto & kv : msg->values)
        values_[kv.key] = kv.value;
    });

  start_client_ = node_->create_client<std_srvs::srv::Trigger>(p + "trajectory_test/start");
  stop_client_ = node_->create_client<std_srvs::srv::Trigger>(p + "trajectory_test/stop");
  param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    node_, p + "trajectory_test_node");
}

void TrajectoryTestPanel::applyNamespace()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    health_received_ = false;
    values_.clear();
  }
  connectNode();
  setResult("-", true);
  Q_EMIT configChanged();
}

void TrajectoryTestPanel::setResult(const QString & text, bool ok)
{
  result_->setText(text);
  result_->setStyleSheet(ok ? "color:#3fa34d;" : "color:#d05050; font-weight:bold;");
}

void TrajectoryTestPanel::onStart()
{
  if(!start_client_)
    return;
  if(!start_client_->service_is_ready())
  {
    setResult("start service not available at " + prefix() + "trajectory_test/start", false);
    return;
  }

  // The node re-checks everything, but this is a command that makes the
  // vehicle fly a pattern: confirm what is about to be flown, at the speed
  // that is about to be used.
  const QString type = type_box_->currentText();
  QString what;
  if(type == "setpoint")
  {
    what = QString("Move to (%1, %2, %3) %4 at up to %5 m/s?")
             .arg(sp_x_spin_->value(), 0, 'f', 1)
             .arg(sp_y_spin_->value(), 0, 'f', 1)
             .arg(sp_z_spin_->value(), 0, 'f', 1)
             .arg(relative_check_->isChecked() ? "relative to the current pose" : "(absolute)")
             .arg(goto_speed_spin_->value(), 0, 'f', 2);
  }
  else
  {
    what = QString("Fly a %1 at %2 m/s, centred on (%3, %4, %5) %6?")
             .arg(type)
             .arg(speed_spin_->value(), 0, 'f', 2)
             .arg(cx_spin_->value(), 0, 'f', 1)
             .arg(cy_spin_->value(), 0, 'f', 1)
             .arg(cz_spin_->value(), 0, 'f', 1)
             .arg(relative_check_->isChecked() ? "relative to the current pose" : "(absolute)");
  }
  what += QString("\n\nThe node re-plans from the current pose and will refuse if it is "
                  "not engaged, the path is infeasible, or it leaves the geofence.");
  if(QMessageBox::question(this, "Start trajectory test", what,
                           QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
    return;

  setResult("start requested...", true);
  start_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto res = future.get();
      std::lock_guard<std::mutex> lock(mutex_);
      pending_result_ = QString::fromStdString(res->message);
      pending_result_ok_ = res->success;
      have_pending_result_ = true;
    });
}

void TrajectoryTestPanel::onStop()
{
  // Never confirmed and never disabled: stop must always be one click.
  if(!stop_client_ || !stop_client_->service_is_ready())
  {
    setResult("stop service not available at " + prefix() + "trajectory_test/stop", false);
    return;
  }
  setResult("stop requested...", true);
  stop_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto res = future.get();
      std::lock_guard<std::mutex> lock(mutex_);
      pending_result_ = QString::fromStdString(res->message);
      pending_result_ok_ = res->success;
      have_pending_result_ = true;
    });
}

void TrajectoryTestPanel::updatePlanFieldVisibility()
{
  const QString type = type_box_->currentText();
  const bool is_setpoint = (type == "setpoint");
  const bool is_circle = (type == "circle");
  const bool is_lemniscate = (type == "lemniscate");

  auto show = [this](const char * key, bool visible) {
    const auto it = plan_row_index_.find(key);
    if(it == plan_row_index_.end())
      return;
    auto & row = plan_rows_[it->second];
    row.first->setVisible(visible);
    row.second->setVisible(visible);
  };

  const bool periodic = is_circle || is_lemniscate;
  show("cx", periodic);
  show("cy", periodic);
  show("cz", periodic);
  show("sp_x", is_setpoint);
  show("sp_y", is_setpoint);
  show("sp_z", is_setpoint);
  // setpoint.yaw is only read in "fixed" yaw mode -- for the periodic
  // shapes it is the fixed heading, for a setpoint move it is the
  // heading held at the target.
  show("yaw", yaw_box_->currentText() == "fixed");
  show("speed", is_circle || is_lemniscate);
  show("radius", is_circle);
  show("width", is_lemniscate);

  // The single most common way to get a plan rejected: absolute
  // coordinates with z left at 0, which is the ground.
  const QString what = is_setpoint ? "setpoint x/y/z" : "centre x/y/z";
  if(relative_check_->isChecked())
  {
    frame_hint_->setText(QString("%1 are offsets from the pose at START; z = 0 keeps "
                                 "the current altitude.").arg(what));
  }
  else
  {
    frame_hint_->setText(QString("%1 are ABSOLUTE local-frame coordinates; z = 0 is the "
                                 "ground and will be rejected by geofence.min_z.").arg(what));
  }
}

void TrajectoryTestPanel::onApplyPlanParams()
{
  if(!param_client_)
    return;
  if(!param_client_->service_is_ready())
  {
    setResult("parameter service not available at " + prefix() + "trajectory_test_node", false);
    return;
  }

  // Parameters are only read when a plan is built, i.e. at the next start,
  // so sending them mid-flight would silently do nothing until the next
  // run. The button is disabled unless the node is holding.
  const std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("trajectory_type", type_box_->currentText().toStdString()),
    rclcpp::Parameter("yaw_mode", yaw_box_->currentText().toStdString()),
    rclcpp::Parameter("relative_to_start", relative_check_->isChecked()),
    rclcpp::Parameter("setpoint.x", sp_x_spin_->value()),
    rclcpp::Parameter("setpoint.y", sp_y_spin_->value()),
    rclcpp::Parameter("setpoint.z", sp_z_spin_->value()),
    rclcpp::Parameter("setpoint.yaw", yaw_spin_->value() * M_PI / 180.0),
    rclcpp::Parameter("speed", speed_spin_->value()),
    rclcpp::Parameter("circle.radius", radius_spin_->value()),
    rclcpp::Parameter("lemniscate.width", width_spin_->value()),
    // One set of centre boxes drives both shapes' parameters, so switching
    // shape cannot leave a stale centre behind from the other one.
    rclcpp::Parameter("circle.center_x", cx_spin_->value()),
    rclcpp::Parameter("circle.center_y", cy_spin_->value()),
    rclcpp::Parameter("circle.z", cz_spin_->value()),
    rclcpp::Parameter("lemniscate.center_x", cx_spin_->value()),
    rclcpp::Parameter("lemniscate.center_y", cy_spin_->value()),
    rclcpp::Parameter("lemniscate.z", cz_spin_->value()),
    rclcpp::Parameter("goto_speed", goto_speed_spin_->value()),
    rclcpp::Parameter("goto_accel", goto_accel_spin_->value()),
  };

  setResult("sending plan parameters...", true);
  param_client_->set_parameters(
    params,
    [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      const auto results = future.get();
      QString failures;
      for(const auto & r : results)
      {
        if(!r.successful)
          failures += QString::fromStdString(r.reason) + " ";
      }
      std::lock_guard<std::mutex> lock(mutex_);
      have_pending_result_ = true;
      pending_result_ok_ = failures.isEmpty();
      pending_result_ = failures.isEmpty()
                          ? QString("plan parameters sent; they take effect at the next START")
                          : QString("parameter rejected: ") + failures;
    });
}

void TrajectoryTestPanel::refresh()
{
  bool received;
  double age;
  uint8_t level;
  std::string message;
  std::map<std::string, std::string> values;
  bool have_result = false;
  QString result_text;
  bool result_ok = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    received = health_received_;
    age = received
            ? std::chrono::duration<double>(std::chrono::steady_clock::now() - health_arrival_).count()
            : std::numeric_limits<double>::infinity();
    level = level_;
    message = message_;
    values = values_;
    if(have_pending_result_)
    {
      have_result = true;
      result_text = pending_result_;
      result_ok = pending_result_ok_;
      have_pending_result_ = false;
    }
  }

  if(have_result)
    setResult(result_text, result_ok);

  const bool live = received && age < kStaleAfter;

  auto str = [&values](const char * key, const char * fallback = "-") {
    const auto it = values.find(key);
    return (it == values.end()) ? std::string(fallback) : it->second;
  };
  auto flag = [&values](const char * key) {
    const auto it = values.find(key);
    return it != values.end() && it->second == "true";
  };
  auto set = [this](const char * key, const QString & text) {
    const auto it = fields_.find(key);
    if(it != fields_.end())
      it->second->setText(text);
  };

  if(!live)
  {
    banner_->setStyleSheet(kStaleStyle);
    banner_->setText(received ? "NO DATA - trajectory_test_node stopped publishing"
                              : "NO DATA - trajectory_test_node not running");
    blocked_->setText(QString("expected at %1trajectory_test/health").arg(prefix()));
    for(const auto & kv : fields_)
      kv.second->setText("-");
    // Start is meaningless without the node; stop stays enabled, since a
    // node that is flying but whose health topic died is exactly when you
    // most want the button to work.
    start_button_->setEnabled(false);
    apply_params_button_->setEnabled(false);
    return;
  }

  banner_->setText(QString::fromStdString(message).toUpper());
  banner_->setStyleSheet(level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR ? kErrStyle
                         : level >= diagnostic_msgs::msg::DiagnosticStatus::WARN ? kWarnStyle
                                                                                 : kOkStyle);

  const std::string blocked = str("blocked_reason", "");
  blocked_->setText(blocked.empty() ? QString("engaged") : QString("blocked: %1").arg(blocked.c_str()));
  blocked_->setStyleSheet(blocked.empty() ? "color:#3fa34d;" : "color:#c08040;");

  const std::string phase = str("phase");
  const bool holding = (phase == "HOLD");
  set("phase", QString("%1   %2 s").arg(phase.c_str()).arg(str("phase_time_s").c_str()));
  set("speed", QString("%1 / %2 m/s").arg(str("speed_achieved").c_str())
                                      .arg(str("speed_requested").c_str()));
  set("fence", QString("%1 / %2 m   alt %3")
        .arg(str("dist_from_origin_xy").c_str())
        .arg(str("geofence_max_radius_xy").c_str())
        .arg(str("altitude_m").c_str()));

  // A requested speed the node had to derate is worth seeing at a glance:
  // it means the shape, not the speed box, is the binding constraint.
  const auto sp_it = fields_.find("speed");
  if(sp_it != fields_.end())
  {
    bool ok_a = false, ok_r = false;
    const double achieved = QString::fromStdString(str("speed_achieved")).toDouble(&ok_a);
    const double requested = QString::fromStdString(str("speed_requested")).toDouble(&ok_r);
    const bool derated = ok_a && ok_r && achieved > 0.0 && achieved < requested - 0.05;
    sp_it->second->setStyleSheet(derated ? "color:#c08040; font-weight:bold;" : "");
  }

  plan_report_->setText(QString("last plan: %1").arg(str("last_plan_report", "none").c_str()));

  start_button_->setEnabled(holding);
  // Parameters are consumed when the plan is built, so changing them while
  // a trajectory is running would appear to do nothing until the next run.
  apply_params_button_->setEnabled(holding);
}

void TrajectoryTestPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString ns;
  if(config.mapGetString("Namespace", &ns))
  {
    ns_selector_->setNs(ns);
    connectNode();
  }
}

void TrajectoryTestPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Namespace", ns_selector_->ns());
}

}  // namespace geo_tuner::panels

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(geo_tuner::panels::TrajectoryTestPanel, rviz_common::Panel)
