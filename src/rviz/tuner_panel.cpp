#include "geo_tuner/rviz/tuner_panel.hpp"
#include "geo_tuner/rviz/namespace_selector.hpp"

#include <rviz_common/display_context.hpp>

#include <QDateTime>
#include <QFont>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QResizeEvent>
#include <QTimer>
#include <QVBoxLayout>

#include <limits>

namespace geo_tuner::panels
{

namespace
{
constexpr double kStaleAfter = 2.0;
const char * kOkStyle = "background-color:#1f7a34; color:white; padding:5px; border-radius:3px;";
const char * kWarnStyle = "background-color:#b37400; color:white; padding:5px; border-radius:3px;";
const char * kErrStyle = "background-color:#a11d1d; color:white; padding:5px; border-radius:3px;";
const char * kStaleStyle = "background-color:#4a4a4a; color:#dddddd; padding:5px; border-radius:3px;";
}  // namespace

TunerPanel::TunerPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * root = new QGridLayout(this);
  grid_ = root;
  root->setContentsMargins(4, 4, 4, 4);
  root->setSpacing(3);
  root->setColumnStretch(0, 1);
  root->setColumnStretch(1, 1);
  int grid_row = 0;

  ns_selector_ = new NamespaceSelector(this);
  root->addWidget(ns_selector_, grid_row++, 0, 1, 2);

  banner_ = new QLabel("waiting for geo_tuner", this);
  QFont bf = banner_->font();
  bf.setPointSizeF(bf.pointSizeF() + 2.0);
  bf.setBold(true);
  banner_->setFont(bf);
  banner_->setAlignment(Qt::AlignCenter);
  banner_->setStyleSheet(kStaleStyle);
  banner_->setWordWrap(true);
  root->addWidget(banner_, grid_row++, 0, 1, 2);

  // Controls sit directly under the banner: an abort you have to scroll to
  // is not an abort.
  {
    auto * row = new QHBoxLayout();
    start_button_ = new QPushButton("START", this);
    start_button_->setStyleSheet("background-color:#1f7a34; color:white; font-weight:bold; padding:7px;");
    abort_button_ = new QPushButton("ABORT", this);
    abort_button_->setStyleSheet("background-color:#a11d1d; color:white; font-weight:bold; padding:7px;");
    row->addWidget(start_button_, 1);
    row->addWidget(abort_button_, 1);
    root->addLayout(row, grid_row++, 0, 1, 2);

    auto * row2 = new QHBoxLayout();
    accept_button_ = new QPushButton("Accept result", this);
    restore_button_ = new QPushButton("Restore safe gains", this);
    // Without this, recovering from an abort meant restarting the node --
    // an SSH session, in flight, to undo something the tuner did on purpose.
    reset_button_ = new QPushButton("Reset session", this);
    row2->addWidget(accept_button_, 1);
    row2->addWidget(restore_button_, 1);
    row2->addWidget(reset_button_, 1);
    root->addLayout(row2, grid_row++, 0, 1, 2);
  }

  // Manoeuvre envelope. The conductor steps the setpoint by these amounts
  // about the hover point and comes straight back, so the vehicle stays
  // within +/- these values of it -- which is what a confined space
  // actually constrains. Editable in flight; the node validates and
  // applies from the next episode.
  {
    auto * box = new QGroupBox("Step envelope (about the hover point)", this);
    auto * grid = new QGridLayout(box);
    grid->setContentsMargins(6, 3, 6, 3);
    grid->setVerticalSpacing(2);

    auto make_spin = [&](double lo, double hi, double step, const char * suffix) {
      auto * sp = new QDoubleSpinBox(box);
      sp->setRange(lo, hi);
      sp->setSingleStep(step);
      sp->setDecimals(2);
      sp->setSuffix(suffix);
      sp->setKeyboardTracking(false);
      connect(sp, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
              this, [this]() { steps_dirty_ = true; });
      return sp;
    };
    // The upper bounds are placeholders until the conductor reports its
    // own ceiling (health key "step_max"), which depends on
    // safety.max_pos_error and is authoritative.
    step_xy_spin_ = make_spin(0.05, 2.0, 0.05, " m");
    step_z_spin_ = make_spin(0.05, 2.0, 0.05, " m");
    step_yaw_spin_ = make_spin(0.05, 1.0, 0.05, " rad");

    grid->addWidget(new QLabel("lateral (x, y)", box), 0, 0);
    grid->addWidget(step_xy_spin_, 0, 1);
    grid->addWidget(new QLabel("vertical (z)", box), 1, 0);
    grid->addWidget(step_z_spin_, 1, 1);
    grid->addWidget(new QLabel("yaw", box), 2, 0);
    grid->addWidget(step_yaw_spin_, 2, 1);

    apply_steps_button_ = new QPushButton("Apply step sizes", box);
    grid->addWidget(apply_steps_button_, 3, 0, 1, 2);

    envelope_label_ = new QLabel("-", box);
    envelope_label_->setWordWrap(true);
    envelope_label_->setStyleSheet("color:#aaaaaa;");
    grid->addWidget(envelope_label_, 4, 0, 1, 2);

    grid->setColumnStretch(1, 1);
    groups_.push_back(box);
  }

  auto make_group = [&](const char * title,
                        const std::vector<std::pair<const char *, const char *>> & rows) {
    auto * box = new QGroupBox(title, this);
    auto * grid = new QGridLayout(box);
    grid->setContentsMargins(6, 3, 6, 3);
    grid->setVerticalSpacing(1);
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
    return box;
  };

  // Only what tells you whether to let the session continue.
  groups_.push_back(make_group("Session", {
    {"progress", "axis / rung"},
    {"target", "wn target"},
    {"episode", "episode"},
    {"blocked", "waiting on"},
  }));

  groups_.push_back(make_group("Vehicle", {
    {"gains", "gains (wn/zeta)"},
    {"pos_err", "position error"},
    {"altitude", "altitude"},
  }));

  log_view_ = new QTextEdit(this);
  log_view_->setReadOnly(true);
  log_view_->setMinimumHeight(60);
  log_view_->setMaximumHeight(110);
  groups_.push_back(log_view_);

  group_row0_ = grid_row;
  relayout(2);

  connect(ns_selector_, &NamespaceSelector::applied, this, &TunerPanel::applyNamespace);
  connect(start_button_, &QPushButton::clicked, this, [this]() {
    callService("start", start_client_,
                "Start an auto-tuning session?\n\nPress START FIRST, while still "
                "in LOITER or POSCTL: the conductor then streams hold setpoints "
                "at the current pose, which is what lets PX4 accept the switch "
                "to OFFBOARD. Switch to OFFBOARD after this, and the session "
                "begins.\n\nIt will fly step inputs and change gains in flight. "
                "Leaving OFFBOARD pauses it; ABORT restores the safe gains.");
  });
  connect(abort_button_, &QPushButton::clicked, this, [this]() {
    callService("abort", abort_client_, QString());  // never confirmed
  });
  connect(accept_button_, &QPushButton::clicked, this, [this]() {
    callService("accept", accept_client_,
                "Accept the current gains as the safe set?\n\nThey stay live on "
                "the controller. Use Save on the Gains tab to keep them past a "
                "restart.");
  });
  connect(restore_button_, &QPushButton::clicked, this, [this]() {
    callService("restore_safe", restore_client_, QString());
  });
  connect(reset_button_, &QPushButton::clicked, this, [this]() {
    callService("reset", reset_client_,
                "Arm another session?\n\nThis only clears the aborted state. "
                "Fix what caused the abort first -- the gains in force are "
                "left exactly as they are.");
  });

  connect(apply_steps_button_, &QPushButton::clicked, this, &TunerPanel::applySteps);

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &TunerPanel::refresh);
  timer_->start(200);
}

TunerPanel::~TunerPanel() = default;

void TunerPanel::relayout(int columns)
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

void TunerPanel::resizeEvent(QResizeEvent * event)
{
  rviz_common::Panel::resizeEvent(event);
  const int w = width();
  if(columns_ == 2 && w < 400)
    relayout(1);
  else if(columns_ != 2 && w > 440)
    relayout(2);
}

void TunerPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  connectNode();
}

QString TunerPanel::prefix() const
{
  return ns_selector_->prefix(node_);
}

void TunerPanel::connectNode()
{
  if(!node_)
    return;
  const std::string p = prefix().toStdString();

  health_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
    p + "geo_tuner/health", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
    [this](diagnostic_msgs::msg::DiagnosticStatus::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      received_ = true;
      arrival_ = std::chrono::steady_clock::now();
      level_ = msg->level;
      message_ = msg->message;
      values_.clear();
      for(const auto & kv : msg->values)
        values_[kv.key] = kv.value;
    });

  traj_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
    p + "trajectory_test/health", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
    [this](diagnostic_msgs::msg::DiagnosticStatus::ConstSharedPtr) {
      std::lock_guard<std::mutex> lock(mutex_);
      traj_seen_ = true;
      traj_arrival_ = std::chrono::steady_clock::now();
    });

  start_client_ = node_->create_client<std_srvs::srv::Trigger>(p + "tuning_conductor/start");
  abort_client_ = node_->create_client<std_srvs::srv::Trigger>(p + "tuning_conductor/abort");
  accept_client_ = node_->create_client<std_srvs::srv::Trigger>(p + "tuning_conductor/accept");
  restore_client_ = node_->create_client<std_srvs::srv::Trigger>(p + "tuning_conductor/restore_safe");
  reset_client_ = node_->create_client<std_srvs::srv::Trigger>(p + "tuning_conductor/reset");
  param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    node_, p + "tuning_conductor");
  steps_dirty_ = false;
}

void TunerPanel::applySteps()
{
  if(!param_client_ || !param_client_->service_is_ready())
  {
    log("step sizes: tuning_conductor not running at " + prefix(), false);
    return;
  }
  const std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("step_size", step_xy_spin_->value()),
    rclcpp::Parameter("step_size_z", step_z_spin_->value()),
    rclcpp::Parameter("yaw_step", step_yaw_spin_->value()),
  };
  const QString what = QString("step sizes -> %1 m lat, %2 m vert, %3 rad yaw")
                         .arg(step_xy_spin_->value(), 0, 'f', 2)
                         .arg(step_z_spin_->value(), 0, 'f', 2)
                         .arg(step_yaw_spin_->value(), 0, 'f', 2);
  param_client_->set_parameters(
    params,
    [this, what](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      const auto results = future.get();
      QString failures;
      for(const auto & r : results)
        if(!r.successful)
          failures += QString::fromStdString(r.reason) + " ";
      std::lock_guard<std::mutex> lock(mutex_);
      have_pending_log_ = true;
      pending_log_ok_ = failures.isEmpty();
      // A refusal carries the node's reason (which limit, and what it is)
      // -- show it verbatim rather than a generic failure.
      pending_log_ = failures.isEmpty()
                       ? (what + " (applies from the next episode)")
                       : ("REJECTED: " + failures);
    });
  steps_dirty_ = false;
}

void TunerPanel::applyNamespace()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    received_ = false;
    traj_seen_ = false;
    values_.clear();
  }
  connectNode();
  Q_EMIT configChanged();
}

void TunerPanel::log(const QString & text, bool ok)
{
  const QString stamp = QDateTime::currentDateTime().toString("HH:mm:ss");
  log_view_->append(ok ? QString("%1  %2").arg(stamp, text)
                       : QString("<span style='color:#d05050;'>%1  %2</span>").arg(stamp, text));
}

void TunerPanel::callService(const QString & name,
                             rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
                             const QString & confirm)
{
  if(!client)
    return;
  if(!client->service_is_ready())
  {
    log(name + ": tuning_conductor not running at " + prefix(), false);
    return;
  }
  if(!confirm.isEmpty() &&
     QMessageBox::question(this, name, confirm,
                           QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
    return;

  client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto res = future.get();
      std::lock_guard<std::mutex> lock(mutex_);
      have_pending_log_ = true;
      pending_log_ok_ = res->success;
      pending_log_ = QString::fromStdString(res->message);
    });
}

void TunerPanel::refresh()
{
  std::map<std::string, std::string> values;
  bool live = false, traj_live = false;
  uint8_t level = 0;
  std::string message;
  bool have_log = false;
  QString log_text;
  bool log_ok = true;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now_t = std::chrono::steady_clock::now();
    live = received_ &&
      std::chrono::duration<double>(now_t - arrival_).count() < kStaleAfter;
    traj_live = traj_seen_ &&
      std::chrono::duration<double>(now_t - traj_arrival_).count() < kStaleAfter;
    level = level_;
    message = message_;
    values = values_;
    if(have_pending_log_)
    {
      have_log = true;
      log_text = pending_log_;
      log_ok = pending_log_ok_;
      have_pending_log_ = false;
    }
  }
  if(have_log)
    log(log_text, log_ok);

  auto str = [&values](const char * key, const char * fallback = "-") {
    const auto it = values.find(key);
    return (it == values.end() || it->second.empty()) ? std::string(fallback) : it->second;
  };
  auto set = [this](const char * key, const QString & text) {
    const auto it = fields_.find(key);
    if(it != fields_.end())
      it->second->setText(text);
  };

  if(!live)
  {
    banner_->setStyleSheet(kStaleStyle);
    banner_->setText(received_ ? "NO DATA - tuning_conductor stopped"
                               : "tuning_conductor not running");
    for(auto & kv : fields_)
      kv.second->setText("-");
    start_button_->setEnabled(false);
    accept_button_->setEnabled(false);
    restore_button_->setEnabled(false);
    reset_button_->setEnabled(false);
    // Abort stays live: a conductor whose health topic died is exactly when
    // you want to be able to stop it.
    abort_button_->setEnabled(true);
    apply_steps_button_->setEnabled(false);
    envelope_label_->setText("-");
    return;
  }

  banner_->setText(QString::fromStdString(message).toUpper());
  banner_->setStyleSheet(level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR ? kErrStyle
                         : level >= diagnostic_msgs::msg::DiagnosticStatus::WARN ? kWarnStyle
                                                                                 : kOkStyle);

  const std::string state = str("state");
  set("progress", QString("%1  %2 / rung %3")
        .arg(str("axis").c_str()).arg(str("axis_index").c_str()).arg(str("rung").c_str()));
  set("target", QString("wn %1, zeta %2")
        .arg(str("wn_target").c_str()).arg(str("zeta_target").c_str()));
  set("episode", str("episode").c_str());

  // One line for why it is not progressing -- the question you actually ask
  // while watching a session sit still.
  QString blocked = "-";
  if(state == "WAIT_ODOM")
    blocked = "odometry";
  else if(state == "WAIT_ENABLE")
    blocked = (str("start_requested") == "true")
                ? "baseline gains"
                : "START (press it before switching to OFFBOARD)";
  else if(state == "WAIT_OFFBOARD")
    blocked = QString("switch to OFFBOARD now - setpoints are streaming (mode: %1)")
                .arg(str("px4_mode").c_str());
  else if(state == "ABORT")
  {
    // The diagnosis says what to DO about it (e.g. a wrong thrust map);
    // the reason alone only says what tripped.
    const std::string diagnosis = str("diagnosis", "");
    blocked = QString::fromStdString(diagnosis.empty() ? str("abort_reason") : diagnosis);
    fields_["blocked"]->setToolTip(QString::fromStdString(str("abort_reason")));
  }
  set("blocked", blocked);

  QStringList gains;
  for(const char * ax : {"gain_x", "gain_y", "gain_z"})
    if(values.count(ax))
      gains << QString::fromStdString(values.at(ax)).remove("wn=").remove("zeta=");
  set("gains", gains.isEmpty() ? "-" : gains.join("  "));
  set("pos_err", str("pos_err").c_str());
  set("altitude", QString("%1 m").arg(str("altitude").c_str()));

  // The conductor is the authority on both the current amplitudes and the
  // largest one it will accept; mirror them, but never stomp on an edit
  // the operator has not applied yet.
  {
    bool ok = false;
    const double step_max = QString::fromStdString(str("step_max", "")).toDouble(&ok);
    if(ok && step_max > 0.0)
    {
      step_xy_spin_->setMaximum(step_max);
      step_z_spin_->setMaximum(step_max);
    }
    if(!steps_dirty_)
    {
      const QSignalBlocker b1(step_xy_spin_), b2(step_z_spin_), b3(step_yaw_spin_);
      bool k = false;
      const double xy = QString::fromStdString(str("step_size", "")).toDouble(&k);
      if(k) step_xy_spin_->setValue(xy);
      const double z = QString::fromStdString(str("step_size_z", "")).toDouble(&k);
      if(k) step_z_spin_->setValue(z);
      const double yw = QString::fromStdString(str("yaw_step", "")).toDouble(&k);
      if(k) step_yaw_spin_->setValue(yw);
    }
    envelope_label_->setText(
      QString("vehicle stays within %1 of the hover point%2")
        .arg(QString::fromStdString(str("envelope")))
        .arg(steps_dirty_ ? "  -- edited, press Apply" : ""));
  }

  // Two setpoint sources reaching the controller is the one thing this
  // panel must not let you cause: the controller follows whichever message
  // arrived last, so the vehicle would chase the tuner's steps and the
  // trajectory node's hold pose alternately.
  if(traj_live)
  {
    banner_->setStyleSheet(kErrStyle);
    banner_->setText("TRAJECTORY TEST IS STREAMING SETPOINTS - STOP IT BEFORE TUNING");
    set("blocked", "trajectory_test_node running");
  }

  const bool finished = (state == "ABORT" || state == "DONE");
  const bool running = !finished && state != "WAIT_ODOM" && state != "WAIT_ENABLE" &&
                       state != "WAIT_OFFBOARD";
  start_button_->setEnabled(!finished && !running && !traj_live);
  abort_button_->setEnabled(true);
  reset_button_->setEnabled(finished);
  accept_button_->setEnabled(!values.count("gain_x") ? false : true);
  restore_button_->setEnabled(true);
  // Amplitudes may be retuned at any time: the change lands at the next
  // episode boundary, never mid-step.
  apply_steps_button_->setEnabled(true);
}

void TunerPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString ns;
  if(config.mapGetString("Namespace", &ns))
  {
    ns_selector_->setNs(ns);
    connectNode();
  }
}

void TunerPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Namespace", ns_selector_->ns());
}

}  // namespace geo_tuner::panels

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(geo_tuner::panels::TunerPanel, rviz_common::Panel)
