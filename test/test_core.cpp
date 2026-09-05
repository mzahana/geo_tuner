// Unit tests for the ROS-free identification and design math.
// Ported 1:1 from the package's original pytest suite.
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "geo_tuner/core/aggregate.hpp"
#include "geo_tuner/core/first_order_fit.hpp"
#include "geo_tuner/core/gain_design.hpp"
#include "geo_tuner/core/gain_yaml.hpp"
#include "geo_tuner/core/safety.hpp"
#include "geo_tuner/core/step_fit.hpp"
#include "geo_tuner/core/yaml_double.hpp"

using namespace geo_tuner;  // NOLINT(build/namespaces)

namespace
{

Eigen::VectorXd arange(double start, double stop, double step)
{
  const auto n = static_cast<Eigen::Index>(std::ceil((stop - start) / step));
  Eigen::VectorXd v(n);
  for (Eigen::Index i = 0; i < n; ++i) {v[i] = start + static_cast<double>(i) * step;}
  return v;
}

}  // namespace

// ---------------------------------------------------------------- gain design

TEST(GainDesign, PdRoundtrip)
{
  const auto [kx, kv] = pd_from_wn_zeta(2.0, 0.9);
  const auto [wn, zeta] = wn_zeta_from_pd(kx, kv);
  EXPECT_NEAR(wn, 2.0, 1e-9);
  EXPECT_NEAR(zeta, 0.9, 1e-9);
}

TEST(GainDesign, RepoDefaultsMapToSaneWn)
{
  // kx=7.4, kv=4.8 from the node defaults
  const auto [wn, zeta] = wn_zeta_from_pd(7.4, 4.8);
  EXPECT_GT(wn, 2.5);
  EXPECT_LT(wn, 3.0);
  EXPECT_GT(zeta, 0.8);
  EXPECT_LT(zeta, 1.0);
}

TEST(GainDesign, MaxThrustFromHover)
{
  VehicleParams v{2.5, 0.5, 0.52};
  EXPECT_NEAR(v.max_thrust(), 2.5 * 9.81 / 0.5, 1e-9);
}

TEST(GainDesign, RespectsSeparationCap)
{
  VehicleParams v{2.5, 0.45, 0.52};
  LoopShape s;
  s.attctrl_tau = 0.3;
  s.timescale_separation = 4.0;
  s.latency = 0.01;  // latency not binding
  const auto g = design_gains(v, s);
  EXPECT_NEAR(g.wn_xy, (2.0 / 0.3) / 4.0, 1e-9);
  EXPECT_NEAR(g.kx[0], g.wn_xy * g.wn_xy, 1e-2 * g.wn_xy * g.wn_xy);
  EXPECT_NEAR(g.kv[0], 2 * s.zeta * g.wn_xy, 1e-2 * 2 * s.zeta * g.wn_xy);
}

TEST(GainDesign, RespectsLatencyCap)
{
  VehicleParams v{2.5, 0.45, 0.52};
  LoopShape s;
  s.latency = 0.2;
  s.latency_margin = 0.3;  // cap at 1.5 rad/s
  const auto g = design_gains(v, s);
  EXPECT_NEAR(g.wn_xy, 1.5, 1e-9);
  bool found = false;
  for (const auto & n : g.notes) {
    if (n.find("latency") != std::string::npos) {found = true;}
  }
  EXPECT_TRUE(found);
}

TEST(GainDesign, RequestClipping)
{
  VehicleParams v{2.5, 0.45, 0.52};
  const auto g = design_gains(v, LoopShape{}, 100.0);
  EXPECT_LT(g.wn_xy, 100.0);
  bool found = false;
  for (const auto & n : g.notes) {
    if (n.find("clipped") != std::string::npos) {found = true;}
  }
  EXPECT_TRUE(found);
}

TEST(GainDesign, ZStifferThanXy)
{
  VehicleParams v{2.5, 0.45, 0.52};
  const auto g = design_gains(v, LoopShape{});
  EXPECT_GT(g.kx[2], g.kx[0]);
}

TEST(GainDesign, IdentificationCorrection)
{
  // Plant has 20% low effective gain (alpha=0.8): measured wn lower
  const double kx_applied = 4.0;
  const double alpha_true = 0.8;
  const double wn_meas = std::sqrt(alpha_true * kx_applied);
  const auto c = correct_gains_from_identification(kx_applied, wn_meas, 2.0, 0.9);
  EXPECT_NEAR(c.alpha, alpha_true, 1e-9);
  // With corrected gains the effective wn hits the target
  EXPECT_NEAR(std::sqrt(alpha_true * c.kx_new), 2.0, 1e-9);
  EXPECT_NEAR(alpha_true * c.kv_new, 2 * 0.9 * 2.0, 1e-9);
}

TEST(GainDesign, YamlGenerationComplete)
{
  VehicleParams v{2.5, 0.45, 0.52};
  const auto g = design_gains(v, LoopShape{});
  const auto y = geometric_controller_yaml(g, 2.5);
  const auto p = y["geometric_controller_node"]["ros__parameters"];
  EXPECT_DOUBLE_EQ(p["mass"].as<double>(), 2.5);
  EXPECT_DOUBLE_EQ(p["gains"]["pos"]["x"].as<double>(), g.kx[0]);
  EXPECT_DOUBLE_EQ(p["gains"]["ki"]["x"].as<double>(), 0.0);
  const auto m = geometric_mavros_yaml(g);
  EXPECT_DOUBLE_EQ(
    m["geometric_mavros_node"]["ros__parameters"]["max_thrust"].as<double>(),
    g.max_thrust);
}

TEST(GainYaml, DoublesEmitAsFloatsNotInts)
{
  // A gain that lands on a whole number must still emit as "2.0". Emitted
  // as bare "2" it is an INTEGER ROS parameter, and geometric_controller
  // declares gains.pos.x as a double -- the node then refuses to start on
  // its own generated config.
  EXPECT_EQ(format_double(2.0), "2.0");
  EXPECT_EQ(format_double(0.0), "0.0");
  EXPECT_EQ(format_double(-3.0), "-3.0");
  // ...and the shortest round-tripping form, as Python's yaml.safe_dump
  // wrote, rather than yaml-cpp's 17 significant digits.
  EXPECT_EQ(format_double(2.7), "2.7");
  EXPECT_EQ(format_double(0.1), "0.1");
  EXPECT_EQ(format_double(50.0), "50.0");     // not "5e+01"
  EXPECT_EQ(format_double(1e-5), "1e-05");    // but tiny values still may
  EXPECT_EQ(format_double(1234.5), "1234.5");
  EXPECT_EQ(format_double(1.0 / 3.0), "0.3333333333333333");

  GainSet g;
  g.kx = {2.0, 2.0, 3.2};
  g.kv = {2.7, 2.7, 3.5};
  g.attctrl_tau = 0.3;
  g.max_thrust = 50.0;
  YAML::Emitter out;
  out << geometric_controller_yaml(g, 2.0);
  const std::string text = out.c_str();
  EXPECT_NE(text.find("x: 2.0"), std::string::npos) << text;
  EXPECT_NE(text.find("mass: 2.0"), std::string::npos) << text;
  EXPECT_EQ(text.find("x: 2\n"), std::string::npos) << text;
  // Round-trips back to the same numbers.
  const auto back = YAML::Load(text)["geometric_controller_node"]["ros__parameters"];
  EXPECT_DOUBLE_EQ(back["gains"]["pos"]["x"].as<double>(), 2.0);
  EXPECT_DOUBLE_EQ(back["mass"].as<double>(), 2.0);

  YAML::Emitter mout;
  mout << geometric_mavros_yaml(g);
  const std::string mtext = mout.c_str();
  EXPECT_NE(mtext.find("max_thrust: 50.0"), std::string::npos) << mtext;
  EXPECT_NE(mtext.find("kf: 1.0"), std::string::npos) << mtext;
}

// ------------------------------------------------------------------ step fit

namespace
{

struct Sim
{
  Eigen::VectorXd t, y;
};

Sim simulate_second_order(
  double wn, double zeta, double delay, double step,
  double t_end = 6.0, double dt = 0.01, double noise = 0.0, uint64_t seed = 0)
{
  Sim s;
  s.t = arange(0.0, t_end, dt);
  s.y = step * second_order_step((s.t.array() - delay).matrix(), wn, zeta);
  if (noise > 0.0) {
    std::mt19937_64 rng(seed);
    std::normal_distribution<double> d(0.0, noise);
    for (Eigen::Index i = 0; i < s.y.size(); ++i) {s.y[i] += d(rng);}
  }
  return s;
}

}  // namespace

class StepFitKnownSystem : public ::testing::TestWithParam<std::pair<double, double>> {};

TEST_P(StepFitKnownSystem, RecoversKnownSystem)
{
  const auto [wn, zeta] = GetParam();
  const auto s = simulate_second_order(wn, zeta, 0.08, 0.5);
  const auto r = fit_step_response(s.t, s.y, 0.5);
  EXPECT_TRUE(r.ok());
  EXPECT_NEAR(r.wn, wn, 0.05 * wn);
  EXPECT_NEAR(r.zeta, zeta, 0.10 * zeta);
  EXPECT_NEAR(r.delay, 0.08, 0.03);
}

INSTANTIATE_TEST_SUITE_P(
  Cases, StepFitKnownSystem,
  ::testing::Values(
    std::make_pair(1.6, 0.95), std::make_pair(2.5, 0.7), std::make_pair(1.0, 1.2)));

TEST(StepFit, RobustToNoise)
{
  const auto s = simulate_second_order(1.8, 0.9, 0.06, 0.5, 6.0, 0.01, 0.02, 7);
  const auto r = fit_step_response(s.t, s.y, 0.5);
  EXPECT_TRUE(r.ok());
  EXPECT_NEAR(r.wn, 1.8, 0.10 * 1.8);
}

TEST(StepFit, BadFitFlagged)
{
  // pure noise, no response
  const Eigen::VectorXd t = arange(0.0, 5.0, 0.01);
  Eigen::VectorXd y(t.size());
  std::mt19937_64 rng(1);
  std::normal_distribution<double> d(0.0, 0.5);
  for (Eigen::Index i = 0; i < y.size(); ++i) {y[i] = d(rng);}
  const auto r = fit_step_response(t, y, 0.5);
  EXPECT_FALSE(r.ok());
}

TEST(StepFit, UnderdampedOvershootMeasured)
{
  const auto s = simulate_second_order(2.0, 0.4, 0.0, 1.0);
  const auto r = fit_step_response(s.t, s.y, 1.0);
  // zeta=0.4 -> ~25% overshoot
  EXPECT_GT(r.overshoot, 0.15);
  EXPECT_LT(r.overshoot, 0.35);
}

TEST(StepFit, RejectsShortData)
{
  EXPECT_THROW(
    fit_step_response(arange(0.0, 5.0, 1.0), Eigen::VectorXd::Zero(5), 0.5),
    std::invalid_argument);
}

TEST(StepFit, BoundPinnedFitRejected)
{
  // A ramp (pure integrator response) cannot be explained by the
  // 2nd-order model inside the bounds; params pin and ok must be false
  const Eigen::VectorXd t = arange(0.0, 6.0, 0.01);
  const Eigen::VectorXd y = 0.02 * t;  // slow ramp, never settles
  const auto r = fit_step_response(t, y, 0.5);
  EXPECT_FALSE(r.ok());
}

TEST(StepFit, AmbiguousHigherOrderResponsePrefersPrior)
{
  // True plant: 2nd order (wn=1.41, zeta=0.95) cascaded with an
  // inner-loop lag (tau=0.15 s) -- the case where a naive fit locks onto
  // "high wn, overdamped, huge delay" (alpha ~ 3.7). The multi-start
  // fitter must return wn near the truth instead.
  const double wn = 1.41, zeta = 0.95, tau = 0.15;
  const Eigen::VectorXd t = arange(0.0, 6.0, 0.01);
  // Integrate x'' + 2*zeta*wn*x' + wn^2*x = wn^2*u, u the lag output of a
  // unit step through 1/(tau*s + 1). RK-free explicit steps at 1 kHz,
  // sampled onto t.
  const double h = 1e-4;
  double u = 0.0, x = 0.0, xd = 0.0, time = 0.0;
  Eigen::VectorXd y(t.size());
  for (Eigen::Index i = 0; i < t.size(); ++i) {
    while (time < t[i] - 1e-12) {
      const double xdd = wn * wn * u - 2.0 * zeta * wn * xd - wn * wn * x;
      x += xd * h;
      xd += xdd * h;
      u += (1.0 - u) * h / tau;
      time += h;
    }
    y[i] = x;
  }
  const auto r = fit_step_response(t, 0.5 * y, 0.5, wn, zeta);
  const double alpha = r.wn * r.wn / (wn * wn);
  EXPECT_GE(alpha, 0.4) << "alpha=" << alpha << " (wn=" << r.wn << ")";
  EXPECT_LE(alpha, 2.5) << "alpha=" << alpha << " (wn=" << r.wn << ")";
}

TEST(StepFit, ZeroDelayIsLegitimate)
{
  // delay pinned at its LOWER bound (0) must not reject the fit
  const auto s = simulate_second_order(1.8, 0.9, 0.0, 0.5);
  const auto r = fit_step_response(s.t, s.y, 0.5);
  EXPECT_FALSE(r.at_bounds);
  EXPECT_TRUE(r.ok());
}

// ----------------------------------------------------------- first order fit

namespace
{

Sim simulate_first_order(
  double T, double delay, double step, double t_end = 6.0, double dt = 0.01,
  double noise = 0.0)
{
  Sim s;
  s.t = arange(0.0, t_end, dt);
  s.y = step * first_order_step((s.t.array() - delay).matrix(), T);
  if (noise > 0.0) {
    std::mt19937_64 rng(3);
    std::normal_distribution<double> d(0.0, noise);
    for (Eigen::Index i = 0; i < s.y.size(); ++i) {s.y[i] += d(rng);}
  }
  return s;
}

}  // namespace

class FirstOrderKnown : public ::testing::TestWithParam<std::pair<double, double>> {};

TEST_P(FirstOrderKnown, RecoversKnownSystem)
{
  const auto [T, delay] = GetParam();
  const auto s = simulate_first_order(T, delay, 0.5);
  const auto r = fit_first_order(s.t, s.y, 0.5);
  EXPECT_TRUE(r.ok());
  EXPECT_NEAR(r.T, T, 0.05 * T);
  EXPECT_NEAR(r.delay, delay, 0.02);
}

INSTANTIATE_TEST_SUITE_P(
  Cases, FirstOrderKnown,
  ::testing::Values(
    std::make_pair(0.15, 0.05), std::make_pair(0.4, 0.1), std::make_pair(0.25, 0.0)));

TEST(FirstOrderFit, NoiseRobust)
{
  const auto s = simulate_first_order(0.2, 0.05, 0.5, 6.0, 0.01, 0.01);
  const auto r = fit_first_order(s.t, s.y, 0.5);
  EXPECT_TRUE(r.ok());
  EXPECT_NEAR(r.T, 0.2, 0.15 * 0.2);
}

TEST(FirstOrderFit, GarbageRejected)
{
  const Eigen::VectorXd t = arange(0.0, 5.0, 0.01);
  Eigen::VectorXd y(t.size());
  std::mt19937_64 rng(9);
  std::normal_distribution<double> d(0.0, 0.5);
  for (Eigen::Index i = 0; i < y.size(); ++i) {y[i] = d(rng);}
  EXPECT_FALSE(fit_first_order(t, y, 0.5).ok());
}

TEST(FirstOrderFit, TauUpdateRule)
{
  // T scales linearly with tau through the (cancelling) efficiency:
  // tau_new = tau * T_target/T_meas reaches the target exactly.
  const double beta = 0.8;  // unknown plant efficiency
  const double tau = 0.3;
  const double T_meas = tau / (2 * beta);
  const double tau_new = tau * 0.35 / T_meas;
  EXPECT_NEAR(tau_new / (2 * beta), 0.35, 1e-12);
}

// ------------------------------------------------------------------ aggregate

TEST(Aggregate, MedianOfThree)
{
  const auto est = robust_ratio_estimate({0.9, 1.0, 1.1});
  EXPECT_TRUE(est.ok);
  EXPECT_NEAR(est.value, 1.0, 1e-12);
  EXPECT_EQ(est.n_used, 3);
}

TEST(Aggregate, OutlierResistant)
{
  // median ignores one wild episode... but the spread gate must then
  // refuse the update (2.2/0.95 >> 1.35)
  auto est = robust_ratio_estimate({0.95, 1.0, 2.2});
  EXPECT_FALSE(est.ok);
  EXPECT_NE(est.reason.find("inconsistent"), std::string::npos);
  // ...unless the gate is opened
  est = robust_ratio_estimate({0.95, 1.0, 2.2}, 2, 3.0);
  EXPECT_TRUE(est.ok);
  EXPECT_NEAR(est.value, 1.0, 1e-12);
}

TEST(Aggregate, ConsistentPairAccepted)
{
  const auto est = robust_ratio_estimate({1.1, 1.25});
  EXPECT_TRUE(est.ok);
  EXPECT_NEAR(est.value, 1.175, 1e-12);
  EXPECT_NEAR(est.spread, 1.25 / 1.1, 1e-12);
}

TEST(Aggregate, SingleEstimateRejectedByDefault)
{
  auto est = robust_ratio_estimate({1.0});
  EXPECT_FALSE(est.ok);
  EXPECT_EQ(est.n_used, 1);
  // but allowed when the session is configured for 1 episode/rung
  est = robust_ratio_estimate({1.0}, 1);
  EXPECT_TRUE(est.ok);
}

TEST(Aggregate, EmptyAndGarbage)
{
  EXPECT_FALSE(robust_ratio_estimate({}).ok);
  EXPECT_FALSE(robust_ratio_estimate({-1.0, std::nan("")}).ok);
}

TEST(Aggregate, BucketMedianDelay)
{
  EpisodeBucket b;
  b.add(1.0, 0.06);
  b.add(1.1, 0.30);
  b.add(0.9, 0.08);
  EXPECT_EQ(b.count(), 3);
  EXPECT_NEAR(b.median_delay(), 0.08, 1e-12);
}

// --------------------------------------------------------------------- safety

namespace
{

OdomSample sample(
  double t = 0.0, std::array<double, 3> pos = {0, 0, 10},
  std::array<double, 3> vel = {0, 0, 0},
  std::array<double, 4> quat = {1, 0, 0, 0},
  std::array<double, 3> rates = {0, 0, 0})
{
  OdomSample s;
  s.t = t;
  s.pos = pos;
  s.vel = vel;
  s.quat = quat;
  s.body_rates = rates;
  return s;
}

bool has(const std::vector<Violation> & v, Violation want)
{
  return std::find(v.begin(), v.end(), want) != v.end();
}

}  // namespace

TEST(Safety, NominalHoverSafe)
{
  SafetyMonitor m;
  EXPECT_TRUE(m.check(sample(), std::array<double, 3>{0, 0, 10}).empty());
}

TEST(Safety, TiltViolation)
{
  SafetyMonitor m;
  // 45 deg roll: q = (cos22.5, sin22.5, 0, 0)
  const std::array<double, 4> q{std::cos(M_PI / 8), std::sin(M_PI / 8), 0, 0};
  EXPECT_NEAR(tilt_from_quat(q), M_PI / 4, 1e-12);
  EXPECT_TRUE(has(m.check(sample(0, {0, 0, 10}, {0, 0, 0}, q), std::nullopt), Violation::TILT));
}

TEST(Safety, PositionErrorViolation)
{
  SafetyLimits l;
  l.max_pos_error = 1.0;
  SafetyMonitor m(l);
  EXPECT_TRUE(
    has(
      m.check(sample(0, {3, 0, 10}), std::array<double, 3>{0, 0, 10}),
      Violation::POS_ERROR));
}

TEST(Safety, AltitudeBounds)
{
  SafetyLimits l;
  l.min_altitude = 2.0;
  l.max_altitude = 20.0;
  SafetyMonitor m(l);
  EXPECT_TRUE(has(m.check(sample(0, {0, 0, 1}), std::nullopt), Violation::ALTITUDE_LOW));
  EXPECT_TRUE(has(m.check(sample(0, {0, 0, 30}), std::nullopt), Violation::ALTITUDE_HIGH));
}

TEST(Safety, VelocityViolation)
{
  SafetyLimits l;
  l.max_velocity = 2.0;
  SafetyMonitor m(l);
  EXPECT_TRUE(
    has(m.check(sample(0, {0, 0, 10}, {3, 0, 0}), std::nullopt), Violation::VELOCITY));
}

TEST(Safety, OscillationDetected)
{
  SafetyLimits l;
  l.osc_rate_rms = 0.5;
  l.osc_window = 2.0;
  SafetyMonitor m(l);
  std::vector<Violation> v;
  for (int i = 0; i < 200; ++i) {
    const double t = i * 0.02;
    const double w = 3.0 * std::sin(2 * M_PI * 6.0 * t);  // 6 Hz wobble, 3 rad/s
    v = m.check(sample(t, {0, 0, 10}, {0, 0, 0}, {1, 0, 0, 0}, {w, 0, 0}), std::nullopt);
  }
  EXPECT_TRUE(has(v, Violation::OSCILLATION));
}

TEST(Safety, NoOscillationOnSmoothFlight)
{
  SafetyLimits l;
  l.osc_rate_rms = 0.5;
  SafetyMonitor m(l);
  std::vector<Violation> v;
  for (int i = 0; i < 200; ++i) {
    v = m.check(
      sample(i * 0.02, {0, 0, 10}, {0, 0, 0}, {1, 0, 0, 0}, {0.05, 0.02, 0}),
      std::nullopt);
  }
  EXPECT_FALSE(has(v, Violation::OSCILLATION));
}

TEST(Safety, StaleOdom)
{
  SafetyLimits l;
  l.odom_timeout = 0.3;
  SafetyMonitor m(l);
  m.check(sample(0.0), std::nullopt);
  const auto v = m.check_stale(1.0);
  ASSERT_EQ(v.size(), 1u);
  EXPECT_EQ(v[0], Violation::ODOM_STALE);
  EXPECT_TRUE(m.check_stale(0.1).empty());
}
