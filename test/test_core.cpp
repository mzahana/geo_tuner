// Unit tests for the ROS-free identification and design math.
// Ported 1:1 from the package's original pytest suite.
#include <gtest/gtest.h>

#include <cctype>
#include <cmath>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "geo_tuner/core/aggregate.hpp"
#include "geo_tuner/core/first_order_fit.hpp"
#include "geo_tuner/core/gain_design.hpp"
#include "geo_tuner/core/gain_yaml.hpp"
#include "geo_tuner/core/safety.hpp"
#include "geo_tuner/core/loop_fit.hpp"
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

// ------------------------------------------------------------------ loop fit

namespace
{

struct Sim
{
  Eigen::VectorXd t, y;
};

/// A step flown by the closed loop the conductor actually commands:
/// gains kx/kv known, plant-gain factor alpha, in-loop lag tau.
Sim simulate_loop(
  double kx, double kv, double alpha, double tau, double step,
  double t_end = 6.0, double dt = 0.01, double noise = 0.0, uint64_t seed = 0)
{
  Sim s;
  s.t = arange(0.0, t_end, dt);
  s.y = step * closed_loop_step(s.t, kx, kv, alpha, tau);
  if (noise > 0.0) {
    std::mt19937_64 rng(seed);
    std::normal_distribution<double> d(0.0, noise);
    for (Eigen::Index i = 0; i < s.y.size(); ++i) {s.y[i] += d(rng);}
  }
  return s;
}

double kv_for(double kx, double zeta = 0.95) {return 2.0 * zeta * std::sqrt(kx);}

/// The real loop, integrated with an explicit delay buffer: actuation lag
/// inside the loop, and a sensing delay that is both fed back to the
/// controller and present on what we record. Deliberately NOT the fitted
/// model, so this exercises the approximation rather than confirming
/// algebra.
Sim simulate_with_transport_delay(
  double kx, double kv, double alpha, double tau_act, double td, double step,
  double t_end = 8.0, double dt_rec = 0.01)
{
  const double h = 1e-4;
  const int nd = static_cast<int>(std::llround(td / h));
  const int n = static_cast<int>(std::llround(t_end / h)) + nd + 10;
  std::vector<double> ph(n, 0.0), vh(n, 0.0);
  double p = 0.0, v = 0.0, a_lag = 0.0;
  const int nrec = static_cast<int>(std::llround(t_end / dt_rec));
  Sim s;
  s.t.resize(nrec);
  s.y.resize(nrec);
  int rec = 0;
  for (int i = 0; i < n; ++i) {
    ph[i] = p;
    vh[i] = v;
    const int j = i - nd;
    const double y_meas = j >= 0 ? ph[j] : 0.0;
    const double v_meas = j >= 0 ? vh[j] : 0.0;
    const double a_des = kx * (step - y_meas) - kv * v_meas;
    if (tau_act > 1e-6) {
      a_lag += (alpha * a_des - a_lag) * h / tau_act;
    } else {
      a_lag = alpha * a_des;
    }
    v += a_lag * h;
    p += v * h;
    while (rec < nrec && rec * dt_rec <= i * h) {
      s.t[rec] = rec * dt_rec;
      s.y[rec] = (i - nd) >= 0 ? ph[i - nd] : 0.0;
      ++rec;
    }
  }
  return s;
}

}  // namespace

TEST(LoopFit, ZeroLagIsTheDesignSecondOrder)
{
  // With no in-loop lag the model must collapse onto the second-order
  // system the gains were designed for.
  const double kx = 2.0, kv = kv_for(kx), alpha = 1.0;
  const Eigen::VectorXd t = arange(0.0, 5.0, 0.01);
  const Eigen::VectorXd a = closed_loop_step(t, kx, kv, alpha, 0.0);
  const Eigen::VectorXd b =
    second_order_step(t, std::sqrt(alpha * kx), alpha * kv / (2.0 * std::sqrt(alpha * kx)));
  EXPECT_LT((a - b).cwiseAbs().maxCoeff(), 1e-9);
}

TEST(LoopFit, StepResponseStartsAtZeroAndSettlesAtOne)
{
  for (double tau : {0.0, 0.05, 0.3, 0.8}) {
    const Eigen::VectorXd t = arange(0.0, 40.0, 0.01);
    const Eigen::VectorXd y = closed_loop_step(t, 2.0, kv_for(2.0), 1.0, tau);
    EXPECT_NEAR(y[0], 0.0, 1e-9) << "tau=" << tau;
    EXPECT_NEAR(y[y.size() - 1], 1.0, 1e-4) << "tau=" << tau;   // unit DC gain
  }
}

class LoopFitKnown
  : public ::testing::TestWithParam<std::tuple<double, double, double>> {};

TEST_P(LoopFitKnown, RecoversAlphaAndLag)
{
  const auto [kx, alpha, tau] = GetParam();
  const double kv = kv_for(kx);
  const auto s = simulate_loop(kx, kv, alpha, tau, 0.5, 6.0, 0.01, 0.003, 1);
  const auto r = fit_closed_loop(s.t, s.y, 0.5, kx, kv, 0.15);
  EXPECT_TRUE(r.ok()) << "at_bounds=" << r.at_bounds << " ambiguous=" << r.ambiguous;
  EXPECT_NEAR(r.alpha, alpha, 0.05 * alpha);
  EXPECT_NEAR(r.tau, tau, 0.25 * tau + 0.01);
}

INSTANTIATE_TEST_SUITE_P(
  Cases, LoopFitKnown,
  ::testing::Combine(
    ::testing::Values(1.2, 2.0, 3.0),      // kx
    ::testing::Values(0.75, 1.0, 1.4),     // alpha
    ::testing::Values(0.05, 0.15, 0.30))); // in-loop lag

TEST(LoopFit, LagDoesNotBiasAlpha)
{
  // The defect that motivated this model: a free second-order fit pays
  // for an unmodelled in-loop lag by inflating wn, and since alpha was
  // derived as wn^2/kx the error landed straight in the gain update. Here
  // alpha must stay put as the lag grows.
  const double kx = 2.0, kv = kv_for(kx), alpha = 1.0;
  double worst = 0.0;
  for (double tau : {0.05, 0.15, 0.30, 0.45}) {
    const auto s = simulate_loop(kx, kv, alpha, tau, 0.5, 8.0, 0.01, 0.003, 2);
    const auto r = fit_closed_loop(s.t, s.y, 0.5, kx, kv, 0.15);
    ASSERT_TRUE(r.ok()) << "tau=" << tau;
    worst = std::max(worst, std::abs(r.alpha - alpha) / alpha);
  }
  EXPECT_LT(worst, 0.08) << "alpha drifts with the in-loop lag";
}

TEST(LoopFit, SurvivesRealTransportDelay)
{
  // The delay is in the loop, and one first-order lag stands in for the
  // (actuation lag + delay) cascade. Data here comes from an exact delay
  // simulation, so this measures that approximation, not the algebra.
  const double kx = 2.0, kv = kv_for(kx);
  double worst = 0.0;
  for (double tau_act : {0.0, 0.06, 0.20}) {
    for (double td : {0.03, 0.06, 0.12}) {
      for (double alpha : {0.8, 1.0}) {
        const auto s =
          simulate_with_transport_delay(kx, kv, alpha, tau_act, td, 0.5);
        const auto r = fit_closed_loop(s.t, s.y, 0.5, kx, kv, 0.15);
        ASSERT_TRUE(r.ok()) << "tau_act=" << tau_act << " td=" << td;
        // The lag must account for the delay, not ignore it: that is what
        // makes the ladder's stability margin see it.
        EXPECT_GT(r.tau, 0.5 * td) << "tau_act=" << tau_act << " td=" << td;
        worst = std::max(worst, std::abs(r.alpha - alpha) / alpha);
      }
    }
  }
  EXPECT_LT(worst, 0.20) << "worst alpha error across the delay sweep";
}

TEST(LoopFit, FittingTheDelayOutsideTheLoopIsWorse)
{
  // Why `model_output_delay` defaults to false. Giving the fit a free
  // output shift lets it park phase outside the loop that physically sits
  // inside it, and alpha pays for it.
  const double kx = 2.0, kv = kv_for(kx);
  double in_loop = 0.0, outside = 0.0;
  int n = 0;
  for (double tau_act : {0.0, 0.06, 0.20}) {
    for (double td : {0.06, 0.12}) {
      const double alpha = 1.0;
      const auto s = simulate_with_transport_delay(kx, kv, alpha, tau_act, td, 0.5);
      in_loop += std::abs(fit_closed_loop(s.t, s.y, 0.5, kx, kv, 0.15, false).alpha - alpha);
      outside += std::abs(fit_closed_loop(s.t, s.y, 0.5, kx, kv, 0.15, true).alpha - alpha);
      ++n;
    }
  }
  EXPECT_LT(in_loop / n, outside / n)
    << "folding the delay into the in-loop lag should beat a free output shift";
}

TEST(LoopFit, GarbageRejected)
{
  const Eigen::VectorXd t = arange(0.0, 5.0, 0.01);
  Eigen::VectorXd y(t.size());
  std::mt19937_64 rng(1);
  std::normal_distribution<double> d(0.0, 0.5);
  for (Eigen::Index i = 0; i < y.size(); ++i) {y[i] = d(rng);}
  EXPECT_FALSE(fit_closed_loop(t, y, 0.5, 2.0, kv_for(2.0)).ok());
}

TEST(LoopFit, RampRejected)
{
  // A pure integrator response cannot be explained by the closed loop at
  // any (alpha, tau); the fit must not be trusted.
  const Eigen::VectorXd t = arange(0.0, 6.0, 0.01);
  const Eigen::VectorXd y = 0.02 * t;
  EXPECT_FALSE(fit_closed_loop(t, y, 0.5, 2.0, kv_for(2.0)).ok());
}

TEST(LoopFit, RejectsShortAndDegenerateInput)
{
  EXPECT_THROW(
    fit_closed_loop(arange(0.0, 5.0, 1.0), Eigen::VectorXd::Zero(5), 0.5, 2.0, 2.7),
    std::invalid_argument);
  const auto s = simulate_loop(2.0, kv_for(2.0), 1.0, 0.15, 0.5);
  EXPECT_THROW(fit_closed_loop(s.t, s.y, 0.0, 2.0, 2.7), std::invalid_argument);
  EXPECT_THROW(fit_closed_loop(s.t, s.y, 0.5, 0.0, 2.7), std::invalid_argument);
}

TEST(LoopFit, EffectiveDampingScalesAsSqrtAlpha)
{
  // Worth stating explicitly, because it is the opposite of the intuition
  // that "stiffer plant = more overshoot": alpha multiplies kx and kv
  // together, so wn_eff = sqrt(alpha*kx) but zeta_eff = sqrt(alpha)*zeta.
  // A plant that is stronger than modelled is therefore MORE damped, and
  // it is a weak plant (alpha < 1) that rings.
  const double kx = 2.0, kv = kv_for(kx, 0.95);
  for (double alpha : {0.4, 1.0, 2.2}) {
    LoopFitResult r;
    r.alpha = alpha;
    EXPECT_NEAR(r.zeta_effective(kx, kv), 0.95 * std::sqrt(alpha), 1e-12);
  }
}

TEST(LoopFit, OvershootMeasuredFromData)
{
  // alpha = 0.4 gives zeta_eff = 0.95*sqrt(0.4) = 0.60, so the response
  // overshoots by roughly 9%.
  const double kx = 2.0, kv = kv_for(kx);
  const auto s = simulate_loop(kx, kv, 0.4, 0.02, 1.0, 10.0);
  const auto r = fit_closed_loop(s.t, s.y, 1.0, kx, kv);
  ASSERT_TRUE(r.ok());
  EXPECT_NEAR(r.alpha, 0.4, 0.05 * 0.4);
  EXPECT_GT(r.overshoot, 0.04);
  EXPECT_LT(r.overshoot, 0.20);
}

TEST(LoopFit, StabilityCapMatchesRouthBoundary)
{
  // The ladder cap wn <= 2*zeta/(margin*tau) is the Routh-Hurwitz
  // condition kv > tau*kx for tau*s^3 + s^2 + alpha*kv*s + alpha*kx.
  // At margin 1 the capped design must sit exactly on the boundary.
  const double zeta = 0.95, tau = 0.25;
  const double wn = 2.0 * zeta / (1.0 * tau);          // margin = 1
  const double kx = wn * wn, kv = 2.0 * zeta * wn;
  EXPECT_NEAR(kv / (tau * kx), 1.0, 1e-9);
  // ...and the default 4x margin is close to the wn*delay <= 0.45 rule
  // it replaced.
  EXPECT_NEAR((2.0 * zeta / (4.0 * tau)) * tau, 0.475, 1e-9);
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
  EXPECT_NEAR(b.median_lag(), 0.08, 1e-12);
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

TEST(Safety, AltitudeUsesAglWhenAvailable)
{
  // The field failure: the local frame's origin sat 6.6 m below ground, so
  // odometry z read -3.8 while the vehicle hovered at 2.8 m AGL. Judged on
  // z the floor fires on a perfectly safe hover; judged on AGL it does not.
  SafetyLimits l;
  l.min_altitude = 2.0;
  l.max_altitude = 20.0;
  SafetyMonitor m(l);
  auto s = sample(0, {0, 0, -3.8});
  s.agl = 2.8;
  EXPECT_TRUE(m.check(s, std::nullopt).empty());
  s.agl = 1.2;
  EXPECT_TRUE(has(m.check(s, std::nullopt), Violation::ALTITUDE_LOW));
  s.agl = 25.0;
  EXPECT_TRUE(has(m.check(s, std::nullopt), Violation::ALTITUDE_HIGH));
}

TEST(Safety, MinAltitudeCanBeSuppressed)
{
  // While the conductor holds position after refusing to start too low it
  // is knowingly below the floor; every other limit still applies.
  SafetyLimits l;
  l.min_altitude = 5.0;
  l.max_velocity = 2.0;
  SafetyMonitor m(l);
  auto s = sample(0, {0, 0, 2.0}, {3, 0, 0});
  const auto v = m.check(s, std::nullopt, /*check_min_altitude=*/false);
  EXPECT_FALSE(has(v, Violation::ALTITUDE_LOW));
  EXPECT_TRUE(has(v, Violation::VELOCITY));
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

// ---------------------------------------------------------------------
// Field-session regression: the 2026-09-10 tuning flight, replayed from
// the conductor's episode dumps (test/data/field_2026-09-10/).
//
// That session accepted three episodes whose fitted in-loop lag rested on
// the solver's 5 ms floor -- z rung1 reps 0 and 1, x rung1 rep 0 -- while
// the lag measured from the same flight's attitude telemetry is
// 160-170 ms. Their alphas (0.905, 0.505, 0.618) poisoned both buckets
// past the consistency gate and the session kept rung-1 gains on x and z,
// 19-29 % under the design bandwidth. These tests pin the fix (T1 in
// ihunter_fixes/docs/TUNER_IMPROVEMENTS_PLAN.md): a lag on the lower
// bound is a failed fit, not "no measurable lag".

namespace
{

struct FieldEpisode
{
  double step{};
  Eigen::VectorXd t, y;
};

FieldEpisode load_field_csv(const std::string & name)
{
  const std::string path = std::string(GEO_TUNER_TEST_DATA_DIR) +
    "/field_2026-09-10/" + name;
  std::ifstream f(path);
  EXPECT_TRUE(f.good()) << "missing fixture " << path;
  FieldEpisode ep;
  std::string line;
  std::vector<double> ts, ys;
  while (std::getline(f, line)) {
    if (line.empty()) {continue;}
    if (line[0] == '#') {
      const auto pos = line.find("step=");
      if (pos != std::string::npos) {ep.step = std::stod(line.substr(pos + 5));}
      continue;
    }
    if (!std::isdigit(static_cast<unsigned char>(line[0])) && line[0] != '-') {
      continue;   // "t,y"
    }
    const auto comma = line.find(',');
    ts.push_back(std::stod(line.substr(0, comma)));
    ys.push_back(std::stod(line.substr(comma + 1)));
  }
  ep.t = Eigen::Map<Eigen::VectorXd>(ts.data(), static_cast<Eigen::Index>(ts.size()));
  ep.y = Eigen::Map<Eigen::VectorXd>(ys.data(), static_cast<Eigen::Index>(ys.size()));
  return ep;
}

// Applied gains per episode, from the session report (kx_applied/kv_applied).
struct FieldCase
{
  const char * file;
  double kx, kv;
};

constexpr double kFlightTauGuess = 0.3;   // the flight's attctrl_tau

}  // namespace

// The three floor-pinned episodes must come back at_bounds and fail ok().
// Before the fix they were "accepted" with alphas 0.905 / 0.505 / 0.618.
TEST(FieldReplay, TauFloorFitsAreRejected)
{
  const FieldCase cases[] = {
    {"ep008_z_rung1_rep0.csv", 1.736, 2.389},
    {"ep009_z_rung1_rep1.csv", 1.736, 2.389},
    {"ep011_x_rung1_rep0.csv", 1.230, 1.948},
  };
  for (const auto & c : cases) {
    const auto ep = load_field_csv(c.file);
    const auto fit = fit_closed_loop(ep.t, ep.y, ep.step, c.kx, c.kv, kFlightTauGuess);
    EXPECT_LT(fit.tau, 0.02) << c.file << ": expected the lag to sit on the floor";
    EXPECT_TRUE(fit.at_bounds) << c.file << ": a floor lag must count as at_bounds";
    EXPECT_FALSE(fit.ok()) << c.file << ": a floor lag must not be accepted";
  }
}

// Every episode whose lag fitted freely must keep fitting cleanly -- the
// rejection must not take good identifications with it.
TEST(FieldReplay, CleanFitsSurvive)
{
  const FieldCase cases[] = {
    {"ep000_z_rung0_rep0.csv", 2.778, 2.504},
    {"ep001_z_rung0_rep1.csv", 2.778, 2.504},
    {"ep002_z_rung0_rep2.csv", 2.778, 2.504},
    {"ep003_x_rung0_rep0.csv", 1.961, 2.329},
    {"ep004_x_rung0_rep1.csv", 1.961, 2.329},
    {"ep005_x_rung0_rep2.csv", 1.961, 2.329},
    {"ep006_y_rung0_rep0.csv", 2.778, 3.167},
    {"ep007_y_rung0_rep1.csv", 2.778, 3.167},
    {"ep010_z_rung1_rep2.csv", 1.736, 2.389},
    {"ep012_x_rung1_rep1.csv", 1.230, 1.948},
    {"ep013_x_rung1_rep2.csv", 1.230, 1.948},
    {"ep014_y_rung1_rep0.csv", 1.736, 2.295},
    {"ep015_y_rung1_rep1.csv", 1.736, 2.295},
  };
  for (const auto & c : cases) {
    const auto ep = load_field_csv(c.file);
    const auto fit = fit_closed_loop(ep.t, ep.y, ep.step, c.kx, c.kv, kFlightTauGuess);
    EXPECT_TRUE(fit.ok()) << c.file;
    EXPECT_GT(fit.tau, 0.02) << c.file << ": clean flight lags were 61-268 ms";
    EXPECT_GE(fit.alpha, 0.85) << c.file;
    EXPECT_LE(fit.alpha, 1.30) << c.file;
  }
}

// Bucket-level consequence of the rejection, with the conductor's own
// aggregation: x rung 1 recovers (two clean estimates agree), z rung 1 is
// safely HELD (one clean estimate is not enough to act on) instead of
// being updated from poisoned data. Both beat the flown outcome, where
// the poisoned spreads (2.09x, 1.79x) rejected the buckets wholesale.
TEST(FieldReplay, BucketOutcomesAfterRejection)
{
  auto alpha_of = [](const char * file, double kx, double kv) {
      const auto ep = load_field_csv(file);
      return fit_closed_loop(ep.t, ep.y, ep.step, kx, kv, kFlightTauGuess);
    };
  std::vector<double> x_bucket, z_bucket;
  for (const char * f :
    {"ep011_x_rung1_rep0.csv", "ep012_x_rung1_rep1.csv", "ep013_x_rung1_rep2.csv"})
  {
    const auto fit = alpha_of(f, 1.230, 1.948);
    if (fit.ok() && fit.alpha >= 0.4 && fit.alpha <= 2.5) {
      x_bucket.push_back(fit.alpha);
    }
  }
  for (const char * f :
    {"ep008_z_rung1_rep0.csv", "ep009_z_rung1_rep1.csv", "ep010_z_rung1_rep2.csv"})
  {
    const auto fit = alpha_of(f, 1.736, 2.389);
    if (fit.ok() && fit.alpha >= 0.4 && fit.alpha <= 2.5) {
      z_bucket.push_back(fit.alpha);
    }
  }
  const auto x_est = robust_ratio_estimate(x_bucket, 2, 1.35);
  EXPECT_TRUE(x_est.ok) << x_est.reason;
  EXPECT_EQ(x_est.n_used, 2);
  EXPECT_LT(x_est.spread, 1.35);

  const auto z_est = robust_ratio_estimate(z_bucket, 2, 1.35);
  EXPECT_FALSE(z_est.ok) << "one clean z estimate must HOLD gains, not update";
  EXPECT_EQ(static_cast<int>(z_bucket.size()), 1);
}

// ------------------------------------------------- fixed-tau fits (T2)

// With the lag frozen at its true value, alpha must come back as cleanly
// as the free fit finds it -- and the frozen dimension must never flag
// at_bounds (the zero-width tau interval is vacuously unpinned).
TEST(FixedTauFit, RecoversAlphaWithTheTrueLagFrozen)
{
  const double kx = 2.0, kv = kv_for(2.0);
  for (double alpha : {0.7, 1.0, 1.3}) {
    const auto s = simulate_loop(kx, kv, alpha, 0.15, 1.0, 6.0, 0.01, 0.003, 7);
    const auto fit = fit_closed_loop(s.t, s.y, 1.0, kx, kv, 0.15, false, 0.15);
    EXPECT_TRUE(fit.ok());
    EXPECT_NEAR(fit.alpha, alpha, 0.06);
    EXPECT_DOUBLE_EQ(fit.tau, 0.15);   // frozen, byte-for-byte
  }
}

// A moderately wrong frozen lag must not silently corrupt alpha: the
// mismatch shows up in the residual, not in a biased estimate. (The
// 2026-09-10 preview measured this on flight data: freezing at a
// session-median lag moved clean-episode alphas by up to ~30 % when the
// true lag differed -- which is why lag_mode per_axis freezes each axis
// at its OWN first clean identification, never a pooled value.)
TEST(FixedTauFit, WrongLagShowsInTheResidual)
{
  const double kx = 2.0, kv = kv_for(2.0);
  const auto s = simulate_loop(kx, kv, 1.0, 0.20, 1.0);
  const auto right = fit_closed_loop(s.t, s.y, 1.0, kx, kv, 0.2, false, 0.20);
  const auto wrong = fit_closed_loop(s.t, s.y, 1.0, kx, kv, 0.2, false, 0.05);
  EXPECT_LT(right.nrmse, 0.01);
  EXPECT_GT(wrong.nrmse, 2.0 * right.nrmse);
}

// On the flight data, freezing the lag at the axis's clean median keeps
// the clean episodes' alphas where the free fit put them (the estimator
// is consistent) while the corrupted episodes' corruption -- which the
// free fit hid by pinning tau -- reappears as a residual 2x and more
// above the clean episodes'. Note what this implies for the conductor:
// with tau frozen there is no tau floor to pin, so T1's at-bounds
// rejection cannot fire; in lag_mode per_axis the bucket spread gate is
// the backstop. That is why per_episode stays the default.
TEST(FixedTauFit, CorruptionShowsInTheResidualOnFlightData)
{
  const double z_tau = 0.141;   // median of the session's clean z lags
  const auto clean = load_field_csv("ep010_z_rung1_rep2.csv");
  const auto fclean = fit_closed_loop(
    clean.t, clean.y, clean.step, 1.736, 2.389, z_tau, false, z_tau);
  EXPECT_TRUE(fclean.ok());
  EXPECT_NEAR(fclean.alpha, 1.053, 0.05);   // the free fit's answer
  for (const char * file : {"ep008_z_rung1_rep0.csv", "ep009_z_rung1_rep1.csv"}) {
    const auto ep = load_field_csv(file);
    const auto fit = fit_closed_loop(
      ep.t, ep.y, ep.step, 1.736, 2.389, z_tau, false, z_tau);
    EXPECT_GT(fit.nrmse, 2.0 * fclean.nrmse) << file;
  }
}

// Same invariance on the x axis: the two clean rung-1 episodes keep their
// free-fit alphas under a frozen lag.
TEST(FixedTauFit, CleanAlphasInvariantUnderFrozenLag)
{
  const struct {const char * file; double free_alpha;} cases[] = {
    {"ep012_x_rung1_rep1.csv", 0.989},
    {"ep013_x_rung1_rep2.csv", 1.106},
  };
  for (const auto & c : cases) {
    const auto ep = load_field_csv(c.file);
    const auto fit = fit_closed_loop(
      ep.t, ep.y, ep.step, 1.230, 1.948, 0.138, false, 0.138);
    EXPECT_TRUE(fit.ok()) << c.file;
    EXPECT_NEAR(fit.alpha, c.free_alpha, 0.05) << c.file;
  }
}
