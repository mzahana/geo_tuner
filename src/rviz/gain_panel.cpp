#include "geo_tuner/rviz/gain_panel.hpp"
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

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace geo_tuner::panels
{

namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kStaleAfter = 2.0;

// Design constraints, matching geo_tuner's gain_design.LoopShape. The panel
// must not accept a gain the offline design rules would refuse: the position
// loop has to stay well inside the attitude loop's bandwidth, and wn is also
// bounded by the round-trip latency of EKF + mavros + offboard.
constexpr double kTimescaleSeparation = 4.0;   // attitude BW / position BW
constexpr double kLatency = 0.08;              // s
constexpr double kLatencyMargin = 0.35;        // require wn * latency <= this
constexpr double kZGainFactor = 1.6;           // z may be stiffer

// The largest factor by which one apply may change kx, matching the tuning
// conductor's max_gain_change_factor. A gain step bigger than this is not a
// tuning tweak, it is a different controller.
constexpr double kMaxChangeFactor = 1.6;

constexpr double kZetaMin = 0.4;
constexpr double kZetaMax = 1.5;

const char * kOkStyle = "background-color:#1f7a34; color:white; padding:6px; border-radius:3px;";
const char * kWarnStyle = "background-color:#b37400; color:white; padding:6px; border-radius:3px;";
const char * kStaleStyle = "background-color:#4a4a4a; color:#dddddd; padding:6px; border-radius:3px;";
const char * kArmedStyle = "background-color:#a11d1d; color:white; padding:6px; border-radius:3px;";

std::array<double, 3> parseTriple(const std::string & s)
{
  std::array<double, 3> out{{kNaN, kNaN, kNaN}};
  std::stringstream ss(s);
  std::string item;
  for(int i = 0; i < 3 && std::getline(ss, item, ','); ++i)
  {
    try {
      out[static_cast<size_t>(i)] = std::stod(item);
    } catch(const std::exception &) {
    }
  }
  return out;
}

double num(const std::map<std::string, std::string> & values, const char * key)
{
  const auto it = values.find(key);
  if(it == values.end())
    return kNaN;
  try {
    return std::stod(it->second);
  } catch(const std::exception &) {
    return kNaN;
  }
}

QString fmt(double v, int prec = 2)
{
  return std::isfinite(v) ? QString::number(v, 'f', prec) : QString("-");
}

const char * kAxisNames[3] = {"x", "y", "z"};

}  // namespace

GainPanel::GainPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * root = new QGridLayout(this);
  grid_ = root;
  root->setContentsMargins(4, 4, 4, 4);
  root->setSpacing(4);
  root->setColumnStretch(0, 1);
  root->setColumnStretch(1, 1);
  int grid_row = 0;

  ns_selector_ = new NamespaceSelector(this);
  root->addWidget(ns_selector_, grid_row++, 0, 1, 2);

  banner_ = new QLabel("waiting for controller", this);
  QFont bf = banner_->font();
  bf.setPointSizeF(bf.pointSizeF() + 3.0);
  bf.setBold(true);
  banner_->setFont(bf);
  banner_->setAlignment(Qt::AlignCenter);
  banner_->setStyleSheet(kStaleStyle);
  banner_->setWordWrap(true);
  root->addWidget(banner_, grid_row++, 0, 1, 2);

  // The interlock is deliberately the first thing under the banner, and is
  // never restored from the saved config: a panel that came back armed
  // because of something you did last week is exactly the accident this is
  // meant to prevent.
  interlock_ = new QCheckBox("Enable edits (resets on reload)", this);
  interlock_->setStyleSheet("font-weight:bold;");
  root->addWidget(interlock_, grid_row++, 0, 1, 2);

  // --- gains -------------------------------------------------------
  {
    auto * box = new QGroupBox("Gains  (yaml: gains.pos.* / gains.vel.*)", this);
    auto * grid = new QGridLayout(box);
    grid->setContentsMargins(6, 4, 6, 4);
    grid->setVerticalSpacing(3);

    // Both representations, side by side. wn/zeta is what you reason about;
    // gains.pos / gains.vel is what is actually in the yaml, and a panel
    // that shows only the first leaves you unable to match it to the file.
    grid->addWidget(new QLabel("axis", box), 0, 0);
    grid->addWidget(new QLabel("wn / zeta", box), 0, 1);
    grid->addWidget(new QLabel("pos / vel", box), 0, 2);
    grid->addWidget(new QLabel("set wn", box), 0, 3);
    grid->addWidget(new QLabel("set zeta", box), 0, 4);

    for(int i = 0; i < 3; ++i)
    {
      grid->addWidget(new QLabel(kAxisNames[i], box), i + 1, 0);

      live_label_[static_cast<size_t>(i)] = new QLabel("-", box);
      live_label_[static_cast<size_t>(i)]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
      live_label_[static_cast<size_t>(i)]->setSizePolicy(QSizePolicy::Ignored,
                                                         QSizePolicy::Preferred);
      grid->addWidget(live_label_[static_cast<size_t>(i)], i + 1, 1);

      raw_label_[static_cast<size_t>(i)] = new QLabel("-", box);
      raw_label_[static_cast<size_t>(i)]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
      raw_label_[static_cast<size_t>(i)]->setSizePolicy(QSizePolicy::Ignored,
                                                        QSizePolicy::Preferred);
      grid->addWidget(raw_label_[static_cast<size_t>(i)], i + 1, 2);

      auto * wn = new QDoubleSpinBox(box);
      wn->setRange(0.1, 10.0);
      wn->setSingleStep(0.05);
      wn->setDecimals(2);
      wn->setMinimumWidth(64);
      wn->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
      grid->addWidget(wn, i + 1, 3);
      wn_spin_[static_cast<size_t>(i)] = wn;

      auto * zeta = new QDoubleSpinBox(box);
      zeta->setRange(kZetaMin, kZetaMax);
      zeta->setSingleStep(0.05);
      zeta->setDecimals(2);
      zeta->setMinimumWidth(64);
      zeta->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
      grid->addWidget(zeta, i + 1, 4);
      zeta_spin_[static_cast<size_t>(i)] = zeta;
    }

    caps_label_ = new QLabel("-", box);
    caps_label_->setWordWrap(true);
    caps_label_->setStyleSheet("color:#909090;");
    grid->addWidget(caps_label_, 4, 0, 1, 5);

    // Restoring a tuning session's gains lives on the Tune tab (Revert);
    // here Revert last only undoes a manual Apply, so the two tabs never
    // offer the same-looking action with different meanings.
    auto * buttons = new QHBoxLayout();
    apply_button_ = new QPushButton("Apply", box);
    revert_button_ = new QPushButton("Revert last", box);
    reload_button_ = new QPushButton("Reload from vehicle", box);
    buttons->addWidget(apply_button_);
    buttons->addWidget(revert_button_);
    buttons->addWidget(reload_button_);
    grid->addLayout(buttons, 5, 0, 1, 5);

    groups_.push_back(box);
  }

  // --- thrust map (read-only) --------------------------------------
  {
    auto * box = new QGroupBox("Thrust map (read-only in flight)", this);
    auto * grid = new QGridLayout(box);
    grid->setContentsMargins(6, 4, 6, 4);
    grid->setVerticalSpacing(2);
    const std::vector<std::pair<const char *, const char *>> rows = {
      {"thrust", "max_thrust / scale"},
      {"suggested", "suggested"},
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

    thrust_correct_ = new QCheckBox("also save corrected max_thrust", box);
    thrust_correct_->setToolTip(
      "Writes max_thrust x online scale estimate. max_thrust scales every "
      "position gain, so this is refused while armed.");
    grid->addWidget(thrust_correct_, r++, 0, 1, 2);

    save_button_ = new QPushButton("Save to vehicle", box);
    save_button_->setToolTip(
      "A parameter write is runtime only and is lost on the next restart. "
      "This asks gain_saver on the vehicle to write the override YAML that "
      "the launch files load at boot.");
    grid->addWidget(save_button_, r++, 0, 1, 2);

    groups_.push_back(box);
  }

  // One status line instead of a log pane: the last thing that happened is
  // what you need, and a scrollback here costs the 3D view real estate.
  log_view_ = new QTextEdit(this);
  log_view_->setReadOnly(true);
  log_view_->setMaximumHeight(56);
  groups_.push_back(log_view_);

  group_row0_ = grid_row;
  relayout(2);

  connect(ns_selector_, &NamespaceSelector::applied, this, &GainPanel::applyNamespace);
  connect(interlock_, &QCheckBox::toggled, this, &GainPanel::onInterlockToggled);
  connect(apply_button_, &QPushButton::clicked, this, &GainPanel::onApply);
  connect(revert_button_, &QPushButton::clicked, this, &GainPanel::onRevert);
  connect(save_button_, &QPushButton::clicked, this, &GainPanel::onSaveToVehicle);
  connect(reload_button_, &QPushButton::clicked, this, [this]() {
    edits_dirty_ = false;
    log("reloaded editors from the live gains");
  });
  for(int i = 0; i < 3; ++i)
  {
    connect(wn_spin_[static_cast<size_t>(i)],
            QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &GainPanel::onEditsChanged);
    connect(zeta_spin_[static_cast<size_t>(i)],
            QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &GainPanel::onEditsChanged);
  }

  onInterlockToggled(false);

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &GainPanel::refresh);
  timer_->start(200);
}

GainPanel::~GainPanel() = default;

void GainPanel::relayout(int columns)
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

void GainPanel::resizeEvent(QResizeEvent * event)
{
  rviz_common::Panel::resizeEvent(event);
  const int w = width();
  if(columns_ == 2 && w < 430)
    relayout(1);
  else if(columns_ != 2 && w > 470)
    relayout(2);
}

void GainPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  connectNode();
}

QString GainPanel::prefix() const
{
  return ns_selector_->prefix(node_);
}

void GainPanel::connectNode()
{
  if(!node_)
    return;
  const std::string p = prefix().toStdString();
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

  ctrl_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
    p + "geometric_controller/status", qos,
    [this](diagnostic_msgs::msg::DiagnosticStatus::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      ctrl_received_ = true;
      ctrl_arrival_ = std::chrono::steady_clock::now();
      ctrl_values_.clear();
      for(const auto & kv : msg->values)
        ctrl_values_[kv.key] = kv.value;
    });

  mavros_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
    p + "geometric_mavros/status", qos,
    [this](diagnostic_msgs::msg::DiagnosticStatus::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      mavros_received_ = true;
      mavros_arrival_ = std::chrono::steady_clock::now();
      mavros_values_.clear();
      for(const auto & kv : msg->values)
        mavros_values_[kv.key] = kv.value;
    });

  // A running geo_tuner session owns these parameters. Two writers stepping
  // on each other mid-flight is the one failure this panel must not cause.
  tuner_sub_ = node_->create_subscription<std_msgs::msg::String>(
    p + "geo_tuner/status", rclcpp::QoS(rclcpp::KeepLast(5)),
    [this](std_msgs::msg::String::ConstSharedPtr) {
      std::lock_guard<std::mutex> lock(mutex_);
      tuner_seen_ = true;
      tuner_arrival_ = std::chrono::steady_clock::now();
    });

  param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    node_, p + "geometric_controller_node");
  save_client_ = node_->create_client<std_srvs::srv::SetBool>(p + "gain_saver/save");

  baseline_ = GainSet();
  previous_ = GainSet();
}

void GainPanel::applyNamespace()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ctrl_received_ = mavros_received_ = tuner_seen_ = false;
    ctrl_values_.clear();
    mavros_values_.clear();
  }
  edits_dirty_ = false;
  interlock_->setChecked(false);
  connectNode();
  Q_EMIT configChanged();
}

void GainPanel::onInterlockToggled(bool on)
{
  apply_button_->setEnabled(on);
  revert_button_->setEnabled(on);
  save_button_->setEnabled(on);
  thrust_correct_->setEnabled(on);
  for(int i = 0; i < 3; ++i)
  {
    wn_spin_[static_cast<size_t>(i)]->setEnabled(on);
    zeta_spin_[static_cast<size_t>(i)]->setEnabled(on);
  }
  if(!on)
    edits_dirty_ = false;
}

void GainPanel::onEditsChanged()
{
  if(interlock_->isChecked())
    edits_dirty_ = true;
}

double GainPanel::wnCap(double attctrl_tau, bool vertical) const
{
  // Two independent ceilings, both from geo_tuner's design rules: the
  // position loop must stay a factor below the attitude loop's bandwidth,
  // and wn is bounded by the sensing/actuation round trip.
  const double separation = (attctrl_tau > 0.0)
                              ? (2.0 / attctrl_tau) / kTimescaleSeparation
                              : std::numeric_limits<double>::infinity();
  const double latency = kLatencyMargin / kLatency;
  const double cap = std::min(separation, latency);
  return vertical ? cap * std::sqrt(kZGainFactor) : cap;
}

void GainPanel::log(const QString & text, bool ok)
{
  const QString stamp = QDateTime::currentDateTime().toString("HH:mm:ss");
  log_view_->append(ok ? QString("%1  %2").arg(stamp, text)
                       : QString("<span style='color:#d05050;'>%1  %2</span>").arg(stamp, text));
  if(node_)
  {
    if(ok)
      RCLCPP_INFO(node_->get_logger(), "[gain panel] %s", text.toStdString().c_str());
    else
      RCLCPP_WARN(node_->get_logger(), "[gain panel] %s", text.toStdString().c_str());
  }
}

void GainPanel::applyGains(const GainSet & gains, const QString & what)
{
  if(!param_client_)
    return;
  if(!param_client_->service_is_ready())
  {
    log("parameter service not available at " + prefix() + "geometric_controller_node", false);
    return;
  }

  previous_ = live_;
  const std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("gains.pos.x", gains.kx[0]),
    rclcpp::Parameter("gains.pos.y", gains.kx[1]),
    rclcpp::Parameter("gains.pos.z", gains.kx[2]),
    rclcpp::Parameter("gains.vel.x", gains.kv[0]),
    rclcpp::Parameter("gains.vel.y", gains.kv[1]),
    rclcpp::Parameter("gains.vel.z", gains.kv[2]),
  };

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
      pending_log_ = failures.isEmpty() ? what : ("REJECTED: " + failures);
    });
}

void GainPanel::onApply()
{
  if(!live_.valid)
  {
    log("no live gains yet; nothing to change", false);
    return;
  }

  GainSet target;
  target.valid = true;
  target.attctrl_tau = live_.attctrl_tau;
  QStringList summary;
  for(int i = 0; i < 3; ++i)
  {
    const size_t a = static_cast<size_t>(i);
    const double wn = wn_spin_[a]->value();
    const double zeta = zeta_spin_[a]->value();

    const double cap = wnCap(live_.attctrl_tau, i == 2);
    if(wn > cap)
    {
      log(QString("refused: wn %1 on %2 exceeds the design cap %3 rad/s "
                  "(attitude separation / latency margin)")
            .arg(fmt(wn)).arg(kAxisNames[i]).arg(fmt(cap)), false);
      return;
    }

    target.kx[a] = wn * wn;
    target.kv[a] = 2.0 * zeta * wn;

    // Bound the size of a single step. A large jump is not a tweak, and the
    // controller is known to misbehave on abrupt changes.
    if(live_.kx[a] > 0.0)
    {
      const double factor = std::max(target.kx[a] / live_.kx[a], live_.kx[a] / target.kx[a]);
      if(factor > kMaxChangeFactor)
      {
        log(QString("refused: kx on %1 would change by %2x, more than the %3x "
                    "limit for one apply; step there in stages")
              .arg(kAxisNames[i]).arg(fmt(factor)).arg(fmt(kMaxChangeFactor, 1)), false);
        return;
      }
    }

    const double wn_live = (live_.kx[a] > 0.0) ? std::sqrt(live_.kx[a]) : kNaN;
    const double zeta_live = (std::isfinite(wn_live) && wn_live > 0.0)
                               ? live_.kv[a] / (2.0 * wn_live) : kNaN;
    summary << QString("%1: wn %2->%3, zeta %4->%5   (gains.pos.%1 %6->%7, "
                       "gains.vel.%1 %8->%9)")
                 .arg(kAxisNames[i]).arg(fmt(wn_live)).arg(fmt(wn))
                 .arg(fmt(zeta_live)).arg(fmt(zeta))
                 .arg(fmt(live_.kx[a])).arg(fmt(target.kx[a]))
                 .arg(fmt(live_.kv[a])).arg(fmt(target.kv[a]));
  }

  const QString text = QString("Apply new gains?\n\n%1\n\nThis takes effect immediately on the "
                               "flying vehicle. It is runtime only -- use Save to vehicle to keep "
                               "it past the next restart.").arg(summary.join("\n"));
  if(QMessageBox::question(this, "Apply gains", text,
                           QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
    return;

  applyGains(target, "applied " + summary.join("; "));
  edits_dirty_ = false;
}

void GainPanel::onRevert()
{
  if(!previous_.valid)
  {
    log("nothing to revert to yet", false);
    return;
  }
  applyGains(previous_, "reverted to the previous gains");
}

void GainPanel::setEmbedded(bool embedded)
{
  ns_selector_->setVisible(!embedded);
}

void GainPanel::adoptNamespace(const QString & ns)
{
  ns_selector_->setNs(ns);
  applyNamespace();
}

void GainPanel::onSaveToVehicle()
{
  if(!save_client_)
    return;
  if(!save_client_->service_is_ready())
  {
    log("gain_saver not running on the vehicle (" + prefix() + "gain_saver/save)", false);
    return;
  }

  const bool correct = thrust_correct_->isChecked();
  QString text = "Write the current gains to the vehicle's override YAML?\n\n"
                 "They will be loaded at the next start, ahead of the shipped config.";
  if(correct)
    text += "\n\nmax_thrust will also be corrected by the online scale estimate. "
            "It scales every position gain, so the vehicle must be disarmed.";
  if(QMessageBox::question(this, "Save to vehicle", text,
                           QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
    return;

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = correct;
  save_client_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      const auto res = future.get();
      std::lock_guard<std::mutex> lock(mutex_);
      have_pending_log_ = true;
      pending_log_ok_ = res->success;
      pending_log_ = QString::fromStdString(res->message);
    });
}

void GainPanel::refresh()
{
  std::map<std::string, std::string> ctrl, mavros;
  bool ctrl_live = false, mavros_live = false, tuner_active = false;
  bool have_log = false;
  QString log_text;
  bool log_ok = true;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    ctrl_live = ctrl_received_ &&
      std::chrono::duration<double>(now - ctrl_arrival_).count() < kStaleAfter;
    mavros_live = mavros_received_ &&
      std::chrono::duration<double>(now - mavros_arrival_).count() < kStaleAfter;
    tuner_active = tuner_seen_ &&
      std::chrono::duration<double>(now - tuner_arrival_).count() < 5.0;
    ctrl = ctrl_values_;
    mavros = mavros_values_;
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

  const bool armed = mavros_live && mavros.count("armed") && mavros.at("armed") == "true";

  // Live gains come from the controller's status topic rather than a
  // parameter read: those are the gains actually in use, which differ from
  // the parameters whenever a setpoint message carries its own.
  if(ctrl_live)
  {
    const auto kx = parseTriple(ctrl.count("kx") ? ctrl.at("kx") : "");
    const auto kv = parseTriple(ctrl.count("kv") ? ctrl.at("kv") : "");
    if(std::isfinite(kx[0]) && std::isfinite(kv[0]))
    {
      live_.kx = kx;
      live_.kv = kv;
      live_.attctrl_tau = num(ctrl, "attctrl_tau");
      live_.valid = true;
      if(!baseline_.valid)
      {
        baseline_ = live_;
        log(QString("baseline captured: wn %1 / %2 / %3")
              .arg(fmt(std::sqrt(kx[0]))).arg(fmt(std::sqrt(kx[1]))).arg(fmt(std::sqrt(kx[2]))));
      }
    }
  }

  for(int i = 0; i < 3; ++i)
  {
    const size_t a = static_cast<size_t>(i);
    if(!live_.valid || !ctrl_live)
    {
      live_label_[a]->setText("-");
      raw_label_[a]->setText("-");
      continue;
    }
    const double wn = (live_.kx[a] > 0.0) ? std::sqrt(live_.kx[a]) : kNaN;
    const double zeta = (std::isfinite(wn) && wn > 0.0) ? live_.kv[a] / (2.0 * wn) : kNaN;
    live_label_[a]->setText(QString("%1 / %2").arg(fmt(wn)).arg(fmt(zeta)));
    // kx = wn^2 and kv = 2*zeta*wn: the same gains the yaml carries as
    // gains.pos.<axis> and gains.vel.<axis>.
    raw_label_[a]->setText(QString("%1 / %2").arg(fmt(live_.kx[a])).arg(fmt(live_.kv[a])));

    // While the user is not mid-edit, the editors track the vehicle, so
    // Apply is always relative to what is actually flying.
    if(!edits_dirty_)
    {
      const QSignalBlocker b1(wn_spin_[a]);
      const QSignalBlocker b2(zeta_spin_[a]);
      if(std::isfinite(wn))
        wn_spin_[a]->setValue(wn);
      if(std::isfinite(zeta))
        zeta_spin_[a]->setValue(zeta);
    }
  }

  if(live_.valid)
  {
    caps_label_->setText(QString("design cap: wn <= %1 rad/s (x, y), %2 (z), from "
                                 "attctrl_tau %3 s and %4 ms latency margin")
                           .arg(fmt(wnCap(live_.attctrl_tau, false)))
                           .arg(fmt(wnCap(live_.attctrl_tau, true)))
                           .arg(fmt(live_.attctrl_tau, 3))
                           .arg(fmt(1000.0 * kLatency, 0)));
  }

  auto set = [this](const char * key, const QString & text) {
    const auto it = fields_.find(key);
    if(it != fields_.end())
      it->second->setText(text);
  };

  if(mavros_live)
  {
    const double max_thrust = num(mavros, "max_thrust_n");
    const double scale = num(mavros, "thrust_scale_est");
    const auto thrust_it = fields_.find("thrust");
    if(thrust_it != fields_.end())
    {
      thrust_it->second->setText(QString("%1 N  x %2").arg(fmt(max_thrust, 1)).arg(fmt(scale, 3)));
      const bool pinned = std::isfinite(scale) && (scale <= 0.801 || scale >= 1.249);
      thrust_it->second->setStyleSheet(pinned ? "color:#d05050; font-weight:bold;" : "");
    }
    set("suggested", (std::isfinite(max_thrust) && std::isfinite(scale))
                       ? fmt(max_thrust * scale, 1) + " N" + (armed ? " (disarm to save)" : "")
                       : QString("-"));
  }
  else
  {
    for(const char * k : {"thrust", "suggested"})
      set(k, "-");
  }

  // The thrust correction is refused by the vehicle while armed; disabling
  // it here as well means the operator is told before they click, not after.
  thrust_correct_->setEnabled(interlock_->isChecked() && !armed);
  if(armed && thrust_correct_->isChecked())
    thrust_correct_->setChecked(false);

  // A running tuner session owns the gains.
  const bool blocked = tuner_active;
  if(blocked && interlock_->isChecked())
  {
    interlock_->setChecked(false);
    log("geo_tuner session detected; edits disabled so the two cannot fight "
        "over the same parameters", false);
  }
  interlock_->setEnabled(!blocked);

  if(!ctrl_live)
  {
    banner_->setStyleSheet(kStaleStyle);
    banner_->setText(ctrl_received_ ? "NO DATA - controller status stopped"
                                    : "NO DATA - waiting for controller");
  }
  else if(blocked)
  {
    banner_->setStyleSheet(kWarnStyle);
    banner_->setText("GEO_TUNER SESSION ACTIVE - EDITS LOCKED");
  }
  else if(!interlock_->isChecked())
  {
    banner_->setStyleSheet(kOkStyle);
    banner_->setText("READ ONLY");
  }
  else if(armed)
  {
    banner_->setStyleSheet(kArmedStyle);
    banner_->setText("EDITS ENABLED - VEHICLE ARMED");
  }
  else
  {
    banner_->setStyleSheet(kWarnStyle);
    banner_->setText("EDITS ENABLED");
  }
}

void GainPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString ns;
  if(config.mapGetString("Namespace", &ns))
  {
    ns_selector_->setNs(ns);
    connectNode();
  }
  // The interlock is intentionally not restored.
  interlock_->setChecked(false);
}

void GainPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Namespace", ns_selector_->ns());
}

}  // namespace geo_tuner::panels

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(geo_tuner::panels::GainPanel, rviz_common::Panel)
