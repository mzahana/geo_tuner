#include "geo_tuner/rviz/tuner_panel.hpp"
#include "geo_tuner/rviz/namespace_selector.hpp"

#include <rviz_common/display_context.hpp>

#include <QDateTime>
#include <QFont>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QTimer>
#include <QVBoxLayout>

namespace geo_tuner::panels
{

namespace
{
constexpr double kStaleAfter = 2.0;
const char * kOkStyle = "background-color:#1f7a34; color:white; padding:5px; border-radius:3px;";
const char * kWarnStyle = "background-color:#b37400; color:white; padding:5px; border-radius:3px;";
const char * kErrStyle = "background-color:#a11d1d; color:white; padding:5px; border-radius:3px;";
const char * kStaleStyle = "background-color:#4a4a4a; color:#dddddd; padding:5px; border-radius:3px;";

const char * kAxes[4] = {"x", "y", "z", "yaw"};

// "wn=1.06 zeta=0.83" (the conductor's health encoding) -> "1.06 / 0.83".
QString gainPair(const std::string & encoded)
{
  QString s = QString::fromStdString(encoded);
  s.remove("wn=");
  s.replace(" zeta=", " / ");
  return s.isEmpty() ? QString("-") : s;
}
}  // namespace

TunerPanel::TunerPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * root = new QVBoxLayout(this);
  root->setContentsMargins(4, 4, 4, 4);
  root->setSpacing(3);

  ns_selector_ = new NamespaceSelector(this);
  root->addWidget(ns_selector_);

  banner_ = new QLabel("waiting for geo_tuner", this);
  QFont bf = banner_->font();
  bf.setPointSizeF(bf.pointSizeF() + 2.0);
  bf.setBold(true);
  banner_->setFont(bf);
  banner_->setAlignment(Qt::AlignCenter);
  banner_->setStyleSheet(kStaleStyle);
  banner_->setWordWrap(true);
  root->addWidget(banner_);

  // Controls sit directly under the banner: an abort you have to scroll to
  // is not an abort.
  {
    auto * row = new QHBoxLayout();
    start_button_ = new QPushButton("START", this);
    start_button_->setStyleSheet(
      "background-color:#1f7a34; color:white; font-weight:bold; padding:7px;");
    abort_button_ = new QPushButton("ABORT", this);
    abort_button_->setStyleSheet(
      "background-color:#a11d1d; color:white; font-weight:bold; padding:7px;");
    row->addWidget(start_button_, 1);
    row->addWidget(abort_button_, 1);
    root->addLayout(row);
  }

  // The glance lines: is it progressing, and is the vehicle where it should
  // be. One line each -- the details live in the health topic, not here.
  progress_line_ = new QLabel("-", this);
  progress_line_->setAlignment(Qt::AlignCenter);
  root->addWidget(progress_line_);
  vehicle_line_ = new QLabel("-", this);
  vehicle_line_->setAlignment(Qt::AlignCenter);
  vehicle_line_->setStyleSheet("color:#909090;");
  root->addWidget(vehicle_line_);
  waiting_line_ = new QLabel(this);
  waiting_line_->setAlignment(Qt::AlignCenter);
  waiting_line_->setWordWrap(true);
  waiting_line_->setStyleSheet("color:#b37400; font-weight:bold;");
  waiting_line_->hide();
  root->addWidget(waiting_line_);

  // Old -> new, filled in as buckets finish; the panel's answer to "what did
  // the session actually do". Hidden until there is something to show.
  {
    result_box_ = new QGroupBox("Result (old → new)", this);
    auto * grid = new QGridLayout(result_box_);
    grid->setContentsMargins(6, 3, 6, 3);
    grid->setVerticalSpacing(2);
    grid->addWidget(new QLabel("", result_box_), 0, 0);
    grid->addWidget(new QLabel("old (wn/ζ)", result_box_), 0, 1);
    grid->addWidget(new QLabel("new (wn/ζ)", result_box_), 0, 3);
    grid->addWidget(new QLabel("outcome", result_box_), 0, 4);
    for(int i = 0; i < 4; ++i)
    {
      const size_t a = static_cast<size_t>(i);
      grid->addWidget(new QLabel(kAxes[i], result_box_), i + 1, 0);
      result_old_[a] = new QLabel("-", result_box_);
      grid->addWidget(result_old_[a], i + 1, 1);
      grid->addWidget(new QLabel("→", result_box_), i + 1, 2);
      result_new_[a] = new QLabel("-", result_box_);
      grid->addWidget(result_new_[a], i + 1, 3);
      result_note_[a] = new QLabel("-", result_box_);
      result_note_[a]->setStyleSheet("color:#909090;");
      result_note_[a]->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
      grid->addWidget(result_note_[a], i + 1, 4);
    }
    grid->setColumnStretch(4, 1);

    auto * row = new QHBoxLayout();
    keep_button_ = new QPushButton("Keep && save", result_box_);
    keep_button_->setToolTip(
      "Accept the gains in force as the safe set and ask gain_saver on the "
      "vehicle to write them to the override YAML, so they survive a restart.");
    revert_button_ = new QPushButton("Revert", result_box_);
    revert_button_->setToolTip("Restore the last known-safe gains on the controller.");
    row->addWidget(keep_button_, 1);
    row->addWidget(revert_button_, 1);
    grid->addLayout(row, 5, 0, 1, 5);

    result_box_->hide();
    root->addWidget(result_box_);
  }

  // Everything an operator touches rarely: the step envelope and the log.
  advanced_toggle_ = new QToolButton(this);
  advanced_toggle_->setText("Advanced");
  advanced_toggle_->setCheckable(true);
  advanced_toggle_->setArrowType(Qt::RightArrow);
  advanced_toggle_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  advanced_toggle_->setAutoRaise(true);
  root->addWidget(advanced_toggle_);

  advanced_box_ = new QWidget(this);
  auto * adv = new QVBoxLayout(advanced_box_);
  adv->setContentsMargins(0, 0, 0, 0);
  adv->setSpacing(3);

  // Manoeuvre envelope. The conductor steps the setpoint by these amounts
  // about the hover point and comes straight back, so the vehicle stays
  // within +/- these values of it -- which is what a confined space
  // actually constrains. Editable in flight; the node validates and
  // applies from the next episode.
  {
    auto * box = new QGroupBox("Step envelope (about the hover point)", advanced_box_);
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
    adv->addWidget(box);
  }

  log_view_ = new QTextEdit(advanced_box_);
  log_view_->setReadOnly(true);
  log_view_->setMinimumHeight(60);
  log_view_->setMaximumHeight(110);
  adv->addWidget(log_view_);

  advanced_box_->hide();
  root->addWidget(advanced_box_);
  root->addStretch(1);

  connect(advanced_toggle_, &QToolButton::toggled, this, [this](bool on) {
    advanced_toggle_->setArrowType(on ? Qt::DownArrow : Qt::RightArrow);
    advanced_box_->setVisible(on);
  });

  connect(ns_selector_, &NamespaceSelector::applied, this, &TunerPanel::applyNamespace);
  connect(start_button_, &QPushButton::clicked, this, &TunerPanel::onStart);
  connect(abort_button_, &QPushButton::clicked, this, [this]() {
    callService("abort", abort_client_, QString());  // never confirmed
  });
  connect(keep_button_, &QPushButton::clicked, this, &TunerPanel::onKeepAndSave);
  connect(revert_button_, &QPushButton::clicked, this, [this]() {
    callService("restore_safe", restore_client_, QString());
  });
  connect(apply_steps_button_, &QPushButton::clicked, this, &TunerPanel::applySteps);

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &TunerPanel::refresh);
  timer_->start(200);
}

TunerPanel::~TunerPanel() = default;

void TunerPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  connectNode();
}

QString TunerPanel::prefix() const
{
  return ns_selector_->prefix(node_);
}

QString TunerPanel::currentNamespace() const
{
  return ns_selector_->ns();
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
  save_client_ = node_->create_client<std_srvs::srv::SetBool>(p + "gain_saver/save");
  param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    node_, p + "tuning_conductor");
  steps_dirty_ = false;
}

void TunerPanel::onStart()
{
  const QString confirm =
    "Start an auto-tuning session?\n\nPress START FIRST, while still "
    "in LOITER or POSCTL: the conductor then streams hold setpoints "
    "at the current pose, which is what lets PX4 accept the switch "
    "to OFFBOARD. Switch to OFFBOARD after this, and the session "
    "begins.\n\nIt will fly step inputs and change gains in flight. "
    "Leaving OFFBOARD pauses it; ABORT restores the safe gains.";
  std::string state;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state = last_state_;
  }
  if(state != "ABORT" && state != "DONE")
  {
    callService("start", start_client_, confirm);
    return;
  }
  // A finished (or aborted) conductor must be reset before another session.
  // That is one gesture for the operator, not two buttons: confirm once,
  // then chain reset -> start. After an abort the confirm carries the
  // reminder that the cause needs fixing first.
  QString text = confirm;
  if(state == "ABORT")
    text += "\n\nThe last session ABORTED. Starting again only makes sense "
            "after the cause is fixed -- the gains in force are unchanged.";
  if(!reset_client_ || !reset_client_->service_is_ready())
  {
    log("start: tuning_conductor not running at " + prefix(), false);
    return;
  }
  if(QMessageBox::question(this, "start", text,
                           QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
    return;
  reset_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto res = future.get();
      if(!res->success)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        have_pending_log_ = true;
        pending_log_ok_ = false;
        pending_log_ = QString::fromStdString(res->message);
        return;
      }
      if(start_client_)
        start_client_->async_send_request(
          std::make_shared<std_srvs::srv::Trigger::Request>(),
          [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f2) {
            const auto r2 = f2.get();
            std::lock_guard<std::mutex> lock(mutex_);
            have_pending_log_ = true;
            pending_log_ok_ = r2->success;
            pending_log_ = QString::fromStdString(r2->message);
          });
    });
}

void TunerPanel::onKeepAndSave()
{
  if(!accept_client_ || !accept_client_->service_is_ready())
  {
    log("keep: tuning_conductor not running at " + prefix(), false);
    return;
  }
  const bool saver = save_client_ && save_client_->service_is_ready();
  QString text = "Keep the gains in force?\n\nThey become the conductor's safe set";
  text += saver
            ? ", and gain_saver writes them to the vehicle's override YAML so "
              "they survive a restart."
            : ".\n\ngain_saver is NOT running, so they stay runtime-only and "
              "are lost at the next restart.";
  if(QMessageBox::question(this, "Keep & save", text,
                           QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
    return;
  accept_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this, saver](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto res = future.get();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        have_pending_log_ = true;
        pending_log_ok_ = res->success;
        pending_log_ = QString::fromStdString(res->message);
      }
      if(!res->success || !saver || !save_client_)
        return;
      auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
      req->data = false;   // never touch max_thrust from here
      save_client_->async_send_request(
        req,
        [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture f2) {
          const auto r2 = f2.get();
          std::lock_guard<std::mutex> lock(mutex_);
          have_pending_log_ = true;
          pending_log_ok_ = r2->success;
          pending_log_ = QString::fromStdString(r2->message);
        });
    });
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
  Q_EMIT namespaceApplied(ns_selector_->ns());
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

  if(!live)
  {
    banner_->setStyleSheet(kStaleStyle);
    banner_->setText(received_ ? "NO DATA - tuning_conductor stopped"
                               : "tuning_conductor not running");
    progress_line_->setText("-");
    vehicle_line_->setText("-");
    waiting_line_->hide();
    result_box_->hide();
    start_button_->setEnabled(false);
    // Abort stays live: a conductor whose health topic died is exactly when
    // you want to be able to stop it.
    abort_button_->setEnabled(true);
    keep_button_->setEnabled(false);
    revert_button_->setEnabled(false);
    apply_steps_button_->setEnabled(false);
    envelope_label_->setText("-");
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_state_.clear();
    }
    return;
  }

  banner_->setText(QString::fromStdString(message).toUpper());
  banner_->setStyleSheet(level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR ? kErrStyle
                         : level >= diagnostic_msgs::msg::DiagnosticStatus::WARN ? kWarnStyle
                                                                                 : kOkStyle);

  const std::string state = str("state");
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_state_ = state;
  }

  progress_line_->setText(
    QString("%1 %2 · rung %3 · ep %4 · wn %5 ζ %6")
      .arg(str("axis").c_str()).arg(str("axis_index").c_str()).arg(str("rung").c_str())
      .arg(str("episode").c_str()).arg(str("wn_target").c_str())
      .arg(str("zeta_target").c_str()));
  vehicle_line_->setText(
    QString("alt %1 m · err %2")
      .arg(str("altitude").c_str()).arg(str("pos_err").c_str()));

  // One line for why it is not progressing -- the question you actually ask
  // while watching a session sit still. Hidden while nothing blocks.
  QString blocked;
  if(state == "WAIT_ODOM")
    blocked = "waiting on odometry";
  else if(state == "WAIT_ENABLE")
    blocked = (str("start_requested") == "true")
                ? "waiting on baseline gains"
                : "press START before switching to OFFBOARD";
  else if(state == "WAIT_OFFBOARD")
    blocked = QString("switch to OFFBOARD now - setpoints are streaming (mode: %1)")
                .arg(str("px4_mode").c_str());
  else if(state == "ABORT")
  {
    // The diagnosis says what to DO about it (e.g. a wrong thrust map);
    // the reason alone only says what tripped.
    const std::string diagnosis = str("diagnosis", "");
    blocked = QString::fromStdString(diagnosis.empty() ? str("abort_reason") : diagnosis);
    waiting_line_->setToolTip(QString::fromStdString(str("abort_reason")));
  }
  waiting_line_->setText(blocked);
  waiting_line_->setVisible(!blocked.isEmpty());

  // Old -> new result rows, appearing as buckets finish. An axis whose
  // gains match the baseline and carries no outcome yet shows a dash.
  {
    bool any = false;
    for(int i = 0; i < 4; ++i)
    {
      const size_t a = static_cast<size_t>(i);
      const std::string ax = kAxes[i];
      QString oldv = "-", newv = "-";
      if(ax == "yaw")
      {
        oldv = QString::fromStdString(str("baseline_yaw_tau"));
        newv = QString::fromStdString(str("yaw_tau"));
        if(oldv != "-")
          oldv = "tau " + oldv;
        if(newv != "-")
          newv = "tau " + newv;
      }
      else
      {
        oldv = gainPair(str(("baseline_" + ax).c_str(), ""));
        newv = gainPair(str(("gain_" + ax).c_str(), ""));
      }
      const QString note = QString::fromStdString(str(("result_" + ax).c_str(), "-"));
      result_old_[a]->setText(oldv);
      result_new_[a]->setText(newv);
      result_note_[a]->setText(note);
      result_note_[a]->setToolTip(note);
      const bool changed = oldv != newv && oldv != "-" && newv != "-";
      result_new_[a]->setStyleSheet(changed ? "font-weight:bold;" : "");
      if(note != "-")
        any = true;
    }
    result_box_->setVisible(any || state == "DONE");
  }

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
    waiting_line_->setText("trajectory_test_node running");
    waiting_line_->show();
  }

  const bool finished = (state == "ABORT" || state == "DONE");
  const bool running = !finished && state != "WAIT_ODOM" && state != "WAIT_ENABLE" &&
                       state != "WAIT_OFFBOARD";
  start_button_->setText(finished ? "START (new session)" : "START");
  start_button_->setEnabled(!running && !traj_live);
  abort_button_->setEnabled(true);
  keep_button_->setEnabled(values.count("gain_x") > 0);
  revert_button_->setEnabled(true);
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
