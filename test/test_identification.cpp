// Tests for the session identification and the gain decisions built on it.
//
// Three layers:
//   1. building blocks and closed-form checks (filters, phase margin);
//   2. a closed-loop simulator with KNOWN truth -- plant gain, delay, lag,
//      and the disturbance measured on the 2026-09-13 flights (0.13 m/s^2
//      rms, ~2 s correlation) -- exercising the estimator, its intervals
//      and the decision rates it produces (Monte Carlo);
//   3. the three recorded field flights (test/data/field_accel/), exported
//      from their bags with scripts/geo-tuner-bag-export.
//
// Thresholds in the Monte Carlo tests are acceptance requirements on the
// method, stated before running it: a tuner that updates correct gains in
// more than 1 session in 10, or misses a 30 % plant-gain error or a lag
// that eats the phase margin in more than 1 in 10, is not fit to fly.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <random>
#include <string>
#include <vector>

#include "geo_tuner/core/accel_loop_id.hpp"
#include "geo_tuner/core/loop_design.hpp"
#include "geo_tuner/core/session_recording.hpp"

using namespace geo_tuner;  // NOLINT(build/namespaces)

namespace
{

struct Truth
{
  double alpha{1.0};
  double delay{0.04};      // transport + estimator delay [s]
  double tau{0.12};        // inner-loop lag [s]
  double wind_sigma{0.0};  // disturbance acceleration rms [m/s^2]
  double wind_tc{2.0};     // disturbance correlation time [s]
  double vel_noise{0.02};  // odometry velocity noise [m/s]
};

struct Flight
{
  double kx{2.56}, kv{3.04};
  double step{1.0};
  int n_steps{4};          // 0 -> +d -> 0 -> -d -> 0
  double step_period{5.0};
  double hover_before{5.0};
  double hover_after{40.0};   // other axes flying: this axis just holds
};

/// One axis of the controller + plant, integrated at 1 kHz, recorded at 50 Hz.
AxisSegment simulate(const Truth & tr, const Flight & fl, std::mt19937_64 & rng)
{
  constexpr double dt = 0.001;
  constexpr int rec_every = 20;
  std::normal_distribution<double> nrm(0.0, 1.0);
  const double T = fl.hover_before + fl.n_steps * fl.step_period + fl.hover_after;
  const int n = static_cast<int>(T / dt);
  std::deque<double> dbuf(std::max<size_t>(1, static_cast<size_t>(tr.delay / dt)), 0.0);
  double p = 0.0, v = 0.0, a_act = 0.0, w = 0.0, vm = 0.0;
  const double ph = std::exp(-dt / tr.wind_tc);
  const double wq = std::sqrt(1.0 - ph * ph) * tr.wind_sigma;
  AxisSegment seg;
  const double legs[] = {1.0, 0.0, -1.0, 0.0};
  for (int k = 0; k < n; ++k) {
    const double t = k * dt;
    double r = 0.0;
    const double ts = t - fl.hover_before;
    if (ts >= 0.0) {
      const int leg = static_cast<int>(ts / fl.step_period);
      if (leg < fl.n_steps) {r = fl.step * legs[leg % 4];}
    }
    if (k % rec_every == 0) {vm = v + tr.vel_noise * nrm(rng);}
    // The controller acts on measured state (sampled at the record rate).
    const double u = fl.kx * (r - p) - fl.kv * vm;
    dbuf.push_back(u);
    const double ud = dbuf.front();
    dbuf.pop_front();
    a_act += (tr.tau > 0.0 ? dt / tr.tau : 1.0) * (tr.alpha * ud - a_act);
    w = ph * w + wq * nrm(rng);
    v += dt * (a_act + w);
    p += dt * v;
    if (k % rec_every == 0) {
      seg.r.push_back(r);
      seg.v.push_back(vm);
      seg.u.push_back(u);
      seg.kx.push_back(fl.kx);
      seg.kv.push_back(fl.kv);
    }
  }
  return seg;
}

/// The simulated controller samples velocity at 50 Hz and holds it: half a
/// sample of lag the loop really has, beyond Truth::delay + Truth::tau.
constexpr double kHoldLag = 0.010;

AccelLoopResult identify(const Truth & tr, const Flight & fl, std::mt19937_64 & rng)
{
  return identify_accel_loop({simulate(tr, fl, rng)});
}

}  // namespace

// ------------------------------------------------------------ building blocks

TEST(AccelLoopBlocks, LowpassPassesDcAndKeepsPhase)
{
  std::vector<double> x(1000);
  for (size_t i = 0; i < x.size(); ++i) {x[i] = 2.0 + std::sin(2 * M_PI * 0.5 * i / 50.0);}
  const auto y = filtfilt_lowpass(x, 3.0, 50.0);
  double err = 0.0;
  for (size_t i = 100; i < 900; ++i) {err = std::max(err, std::abs(y[i] - x[i]));}
  EXPECT_LT(err, 0.02);   // 0.5 Hz: unit gain, zero phase
  for (size_t i = 0; i < x.size(); ++i) {x[i] = std::sin(2 * M_PI * 15.0 * i / 50.0);}
  const auto z = filtfilt_lowpass(x, 3.0, 50.0);
  double amp = 0.0;
  for (size_t i = 100; i < 900; ++i) {amp = std::max(amp, std::abs(z[i]));}
  EXPECT_LT(amp, 0.05);   // 15 Hz: attenuated
}

TEST(AccelLoopBlocks, NominalCommandOfAStep)
{
  std::vector<double> r(500, 1.0), kx(500, 2.0), kv(500, 2.4);
  r[0] = 0.0;
  const auto z = nominal_command(r, kx, kv, 50.0);
  EXPECT_NEAR(z[1], 2.0, 0.15);          // kx * step at the step
  EXPECT_NEAR(z.back(), 0.0, 1e-3);      // settled
}

TEST(AccelLoopBlocks, StudentT)
{
  EXPECT_NEAR(t_quantile(0.90, 7), 1.895, 1e-3);
  EXPECT_NEAR(t_quantile(0.95, 2), 4.303, 1e-3);
  EXPECT_NEAR(t_quantile(0.90, 200), 1.653, 0.01);
}

// ------------------------------------------------------- phase margin, design

TEST(LoopDesign, PhaseMarginMatchesClosedFormWithoutLag)
{
  // L = (kv s + kx)/s^2: crossover w^4 = kx^2 + kv^2 w^2, PM = atan(w kv/kx).
  const double kx = 2.56, kv = 3.04;
  const double w2 = 0.5 * (kv * kv + std::sqrt(std::pow(kv, 4) + 4 * kx * kx));
  const double w = std::sqrt(w2);
  double wc = 0.0;
  EXPECT_NEAR(phase_margin_deg(kx, kv, LoopPlant{1.0, 0.0, 0.0}, &wc),
    std::atan(w * kv / kx) * 180.0 / M_PI, 1e-6);
  EXPECT_NEAR(wc, w, 1e-6);
}

TEST(LoopDesign, LagCostsPhaseAndPureLagNeverDestabilises)
{
  const double kx = 2.56, kv = 3.04;
  double last = 1e9;
  for (double tau : {0.0, 0.1, 0.2, 0.3}) {
    const double pm = phase_margin_deg(kx, kv, LoopPlant{1.0, 0.0, tau});
    EXPECT_LT(pm, last);
    last = pm;
  }
  // Routh on tau s^3 + s^2 + a kv s + a kx: stable iff kv > tau kx, for ANY a.
  EXPECT_TRUE(std::isinf(gain_margin(kx, kv, LoopPlant{1.0, 0.0, 0.2})));
  // A delay makes the gain margin finite.
  EXPECT_TRUE(std::isfinite(gain_margin(kx, kv, LoopPlant{1.0, 0.1, 0.1})));
}

namespace
{

AccelLoopResult fake_id(double alpha, double a_lo, double a_hi, double delay, double tau,
  double lag_hi)
{
  AccelLoopResult id;
  id.ok = true;
  id.alpha = alpha;
  id.alpha_lo = a_lo;
  id.alpha_hi = a_hi;
  id.delay = delay;
  id.tau = tau;
  id.lag = delay + tau;
  id.lag_lo = std::max(0.0, id.lag - (lag_hi - id.lag));
  id.lag_hi = lag_hi;
  id.r2 = 0.95;
  return id;
}

}  // namespace

TEST(LoopDesign, KeepsTheRequestWhenTheMarginAllows)
{
  const auto id = fake_id(1.0, 0.95, 1.05, 0.0, 0.08, 0.10);
  const auto d = design_lag_aware(1.6, 0.95, id, MarginSpec{});
  EXPECT_FALSE(d.limited);
  EXPECT_NEAR(d.wn, 1.6, 1e-12);
  EXPECT_NEAR(d.kx, 2.56, 1e-12);
  EXPECT_NEAR(d.kv, 3.04, 1e-12);
}

TEST(LoopDesign, LowersBandwidthForALongLag)
{
  const auto id = fake_id(1.0, 0.9, 1.1, 0.10, 0.25, 0.40);
  const MarginSpec spec;
  const auto d = design_lag_aware(1.6, 0.95, id, spec);
  EXPECT_TRUE(d.limited);
  EXPECT_LT(d.wn, 1.6);
  EXPECT_GE(d.pm_nominal_deg, spec.nominal_deg - 1e-6);
  EXPECT_GE(d.pm_worst_deg, spec.worst_deg - 1e-6);
}

TEST(LoopDesign, ConfirmsGainsTheEvidenceSupports)
{
  const auto id = fake_id(1.02, 0.94, 1.10, 0.0, 0.15, 0.20);
  const DecisionConfig cfg;
  const auto dec = decide_axis(2.50, 2.95, id, cfg);
  EXPECT_EQ(dec.verdict, AxisVerdict::CONFIRMED) << dec.why;
  EXPECT_DOUBLE_EQ(dec.kx_new, 2.50);
}

TEST(LoopDesign, ImpreciseEvidenceCannotConfirm)
{
  // Gains designed for alpha 1 against a gusty estimate of 0.87 in a 1.7x
  // wide interval: they sit inside the supported range with margin to spare,
  // yet are 13 % soft if the estimate is right. Not a confirmation.
  const auto id = fake_id(0.87, 0.66, 1.14, 0.0, 0.08, 0.10);
  const auto dec = decide_axis(2.56, 3.04, id, DecisionConfig{});
  EXPECT_EQ(dec.verdict, AxisVerdict::INCONCLUSIVE) << dec.why;
  EXPECT_DOUBLE_EQ(dec.kx_new, 2.56);
  // The same gains on a precise estimate are a verdict either way.
  const auto precise = fake_id(0.87, 0.82, 0.92, 0.0, 0.08, 0.10);
  EXPECT_NE(decide_axis(2.56, 3.04, precise, DecisionConfig{}).verdict,
    AxisVerdict::INCONCLUSIVE);
}

TEST(LoopDesign, UpdatesGainsOutsideTheSupportedRange)
{
  const auto id = fake_id(1.0, 0.95, 1.05, 0.0, 0.15, 0.20);
  const DecisionConfig cfg;
  const auto dec = decide_axis(1.50, 2.30, id, cfg);   // the 2026-09-13 10:38 de-tune
  EXPECT_EQ(dec.verdict, AxisVerdict::UPDATE) << dec.why;
  EXPECT_GT(dec.kx_new, 1.50);
  EXPECT_LE(dec.kx_new, 1.50 * cfg.max_change + 1e-12);
}

TEST(LoopDesign, UpdatesGainsWithoutPhaseMargin)
{
  // Gains inside the alpha range but far too stiff for a 300 ms lag.
  const auto id = fake_id(1.0, 0.95, 1.05, 0.1, 0.2, 0.35);
  const DecisionConfig cfg;
  const auto dec = decide_axis(2.56, 3.04, id, cfg);
  EXPECT_EQ(dec.verdict, AxisVerdict::UPDATE) << dec.why;
  EXPECT_TRUE(dec.design.limited);
  EXPECT_LT(dec.kx_new, 2.56);
}

TEST(LoopDesign, NoEstimateTouchesNothing)
{
  AccelLoopResult id;
  id.reason = "setpoint never moved";
  const auto dec = decide_axis(2.0, 2.5, id, DecisionConfig{});
  EXPECT_EQ(dec.verdict, AxisVerdict::NO_ESTIMATE);
  EXPECT_DOUBLE_EQ(dec.kx_new, 2.0);
  EXPECT_DOUBLE_EQ(dec.kv_new, 2.5);
}

TEST(LoopDesign, ValidationChecksMarginPlantAndOvershoot)
{
  const auto prior = fake_id(1.0, 0.93, 1.07, 0.0, 0.15, 0.20);
  const MarginSpec spec;
  auto v = validate_gains(2.56, 3.04, prior, prior, 0.05, spec, 0.25);
  EXPECT_TRUE(v.pass) << v.why;
  // plant changed between design and validation
  v = validate_gains(2.56, 3.04, fake_id(1.4, 1.3, 1.5, 0.0, 0.15, 0.2), prior, 0.05, spec, 0.25);
  EXPECT_FALSE(v.pass);
  // ringing seen in the records
  v = validate_gains(2.56, 3.04, prior, prior, 0.40, spec, 0.25);
  EXPECT_FALSE(v.pass);
  // validation data that does not identify is not a pass
  v = validate_gains(2.56, 3.04, AccelLoopResult{}, prior, 0.05, spec, 0.25);
  EXPECT_FALSE(v.pass);
  // the margin floor broken on the validation estimate (300 ms lag)
  const auto slow = fake_id(1.0, 0.93, 1.07, 0.10, 0.20, 0.32);
  v = validate_gains(2.56, 3.04, slow, slow, 0.05, spec, 0.25);
  EXPECT_FALSE(v.pass) << v.why;
}

TEST(LoopDesign, ValidationIsASafetyTestNotTheDesignTarget)
{
  // Gains designed to 45 deg that the validation round estimates at 39 deg:
  // inside the noise of a one-round estimate, above the 35 deg floor. The
  // pooled decision may still lower them; validation must not throw away a
  // safe update (quad_sim 2026-09-13 s2 x).
  const auto prior = fake_id(1.02, 0.85, 1.22, 0.0, 0.167, 0.188);
  const auto val = fake_id(1.15, 1.05, 1.26, 0.10, 0.09, 0.21);
  const auto v = validate_gains(2.52, 2.99, val, prior, 0.08, MarginSpec{}, 0.25);
  EXPECT_GE(v.pm_nominal_deg, 35.0);
  EXPECT_LT(v.pm_nominal_deg, 45.0);
  EXPECT_TRUE(v.pass) << v.why;
}

TEST(LoopDesign, AWorstCornerOfAWideIntervalIsNotEvidence)
{
  // quad_sim 2026-09-13 s3 y, round 1: margin fine at the estimate, missing
  // only at the corner of a 2.6x interval. More data, not a gain cut.
  const auto wide = fake_id(0.95, 0.59, 1.51, 0.10, 0.10, 0.26);
  const auto dec = decide_axis(2.0, 2.7, wide, DecisionConfig{});
  EXPECT_GE(dec.pm_now_nominal_deg, 40.0);
  EXPECT_LT(dec.pm_now_worst_deg, 30.0);
  EXPECT_EQ(dec.verdict, AxisVerdict::INCONCLUSIVE) << dec.why;
  // The same corner on a precise interval is acted on.
  const auto precise = fake_id(1.40, 1.33, 1.51, 0.10, 0.10, 0.26);
  const auto p = decide_axis(2.0, 2.7, precise, DecisionConfig{});
  EXPECT_EQ(p.verdict, AxisVerdict::UPDATE) << p.why;
}

TEST(LoopDesign, AnUpdateOnAWideIntervalMovesOnlyToItsEdge)
{
  // Gains significantly soft (below the range even at the upper alpha), on a
  // 1.44x interval: update, but to the design at alpha_hi -- stiff enough for
  // every plant in the interval, too stiff for none -- not to the point
  // estimate (quad_sim 2026-09-13 s2 x: the point design overshot the true
  // plant's by 13 % and failed validation).
  const auto wide = fake_id(1.02, 0.85, 1.22, 0.0, 0.10, 0.12);
  const auto dec = decide_axis(1.572, 2.223, wide, DecisionConfig{});
  ASSERT_EQ(dec.verdict, AxisVerdict::UPDATE) << dec.why;
  EXPECT_NEAR(dec.kx_new, 1.6 * 1.6 / 1.22, 1e-9);
  EXPECT_NEAR(dec.kv_new, 2 * 0.95 * 1.6 / 1.22, 1e-9);
  EXPECT_LT(dec.kx_new, dec.design.kx);
  // On a narrow interval the edge is the design.
  const auto narrow = fake_id(1.02, 1.00, 1.04, 0.0, 0.10, 0.12);
  EXPECT_NEAR(decide_axis(1.572, 2.223, narrow, DecisionConfig{}).kx_new, 2.56 / 1.04, 1e-9);
  // Missing margin is acted on however wide the interval, towards softer gains.
  const auto slow_wide = fake_id(1.02, 0.85, 1.22, 0.10, 0.25, 0.40);
  const auto slow = decide_axis(2.56, 3.04, slow_wide, DecisionConfig{});
  EXPECT_EQ(slow.verdict, AxisVerdict::UPDATE);
  EXPECT_LT(slow.kx_new, 2.56);
}

// ------------------------------------------------------- estimator vs truth

TEST(AccelLoopId, RecoversTruthInCalmAir)
{
  std::mt19937_64 rng(1);
  for (const Truth tr : {Truth{1.0, 0.04, 0.12}, Truth{0.75, 0.02, 0.06},
      Truth{1.25, 0.0, 0.18}})
  {
    const auto id = identify(tr, Flight{}, rng);
    ASSERT_TRUE(id.ok) << id.reason;
    EXPECT_NEAR(id.alpha, tr.alpha, 0.04 * tr.alpha);
    // A lag and a delay trade nearly 1:1 below the loop bandwidth; their
    // sum is what the margin depends on, and what is required here.
    EXPECT_NEAR(id.lag, tr.delay + tr.tau, 0.04);
    EXPECT_GT(id.r2, 0.9);
  }
}

TEST(AccelLoopId, IntervalsCoverTruthInFieldWind)
{
  // Coverage of the 90 % intervals over independent sessions. The jackknife
  // over 8 blocks is known to under-cover a little; require >= 75 %.
  std::mt19937_64 rng(2);
  Truth tr{1.0, 0.04, 0.12, 0.13};
  int n = 0, cover_a = 0, cover_l = 0;
  double sum_log_err = 0.0;
  for (int k = 0; k < 40; ++k) {
    const auto id = identify(tr, Flight{}, rng);
    if (!id.ok) {continue;}
    ++n;
    cover_a += (id.alpha_lo <= tr.alpha && tr.alpha <= id.alpha_hi);
    const double lag_true = tr.delay + tr.tau + kHoldLag;
    cover_l += (id.lag_lo <= lag_true && lag_true <= id.lag_hi);
    sum_log_err += std::abs(std::log(id.alpha));
  }
  ASSERT_GE(n, 36);
  EXPECT_GE(static_cast<double>(cover_a) / n, 0.75);
  EXPECT_GE(static_cast<double>(cover_l) / n, 0.75);
  // Mean |alpha error| well under the +/-12 % the per-step fits scattered.
  EXPECT_LT(sum_log_err / n, 0.06);
}

TEST(AccelLoopId, LagIsNotUnderestimatedInWind)
{
  // The lag sets the phase margin, so reading it short is the unsafe error.
  // Over independent sessions the mean total lag must sit within 10 % of the
  // truth, at the field wind and at twice it, for a z-like and an x-like loop.
  struct Case
  {
    Truth tr;
    uint64_t seed;
  };
  for (const Case c : {Case{Truth{1.0, 0.02, 0.05, 0.0}, 20}, Case{Truth{1.0, 0.02, 0.05, 0.13}, 21},
      Case{Truth{1.0, 0.04, 0.175, 0.0}, 25}, Case{Truth{1.0, 0.04, 0.175, 0.13}, 22},
      Case{Truth{1.0, 0.04, 0.175, 0.26}, 23}, Case{Truth{0.8, 0.02, 0.14, 0.26}, 24}})
  {
    std::mt19937_64 rng(c.seed);
    const double truth = c.tr.delay + c.tr.tau;
    double sum = 0.0;
    int n = 0;
    for (int k = 0; k < 30; ++k) {
      const auto id = identify(c.tr, Flight{}, rng);
      if (!id.ok) {continue;}
      sum += id.lag;
      ++n;
    }
    ASSERT_GE(n, 25);
    const double mean = sum / n;
    std::printf("lag truth %.0f ms wind %.2f: mean estimate %.0f ms (%+.0f %%)\n", 1e3 * truth,
      c.tr.wind_sigma, 1e3 * mean, 100.0 * (mean / truth - 1.0));
    // The simulated controller holds its velocity term for one 20 ms sample,
    // which is half a sample (10 ms) of real loop lag on top of delay + tau.
    // Short by more than 5 % fails; long by more than 10 % beyond the hold
    // fails (conservative, but it costs bandwidth).
    EXPECT_GE(mean, 0.95 * (truth + kHoldLag)) << "wind " << c.tr.wind_sigma;
    EXPECT_LE(mean, 1.10 * (truth + kHoldLag)) << "wind " << c.tr.wind_sigma;
  }
}

TEST(AccelLoopId, RefusesWithoutExcitation)
{
  std::mt19937_64 rng(3);
  Flight fl;
  fl.n_steps = 0;
  const auto id = identify(Truth{1.0, 0.04, 0.12, 0.13}, fl, rng);
  EXPECT_FALSE(id.ok);
}

TEST(AccelLoopId, SegmentsAreNotFilteredAcrossAGap)
{
  std::mt19937_64 rng(4);
  const Truth tr{1.0, 0.04, 0.12};
  Flight fl;
  fl.hover_after = 5.0;
  const auto a = simulate(tr, fl, rng);
  const auto b = simulate(tr, fl, rng);
  const auto id = identify_accel_loop({a, b});
  ASSERT_TRUE(id.ok) << id.reason;
  EXPECT_NEAR(id.alpha, 1.0, 0.05);
}

// ----------------------------------------------- decision rates (Monte Carlo)

namespace
{

/// A session as the conductor flies it, on fixed gains: rounds of steps,
/// every round identified on ALL data so far, until a verdict other than
/// "more data" or the rounds run out. Returns the verdict it ends on.
AxisVerdict session_verdict(
  const Truth & tr, const Flight & fl, int max_rounds, std::mt19937_64 & rng)
{
  std::vector<AxisSegment> segs;
  AxisVerdict v = AxisVerdict::NO_ESTIMATE;
  for (int round = 0; round < max_rounds; ++round) {
    segs.push_back(simulate(tr, fl, rng));
    const auto id = identify_accel_loop(segs);
    const auto dec = decide_axis(fl.kx, fl.kv, id, DecisionConfig{});
    v = dec.verdict;
    if (std::getenv("GEO_TUNER_DIAG")) {
      std::printf("  round %d: alpha %.3f [%.3f %.3f] lag %.0f [%.0f %.0f] -> %s: %s\n", round + 1,
        id.alpha, id.alpha_lo, id.alpha_hi, 1e3 * id.lag, 1e3 * id.lag_lo, 1e3 * id.lag_hi,
        to_string(v), dec.why.c_str());
    }
    if (v == AxisVerdict::CONFIRMED || v == AxisVerdict::UPDATE) {break;}
  }
  return v;
}

struct Rates
{
  double update{0.0}, confirmed{0.0};
};

Rates session_rates(const Truth & tr, double kx, double kv, int sessions, uint64_t seed)
{
  std::mt19937_64 rng(seed);
  Flight fl;
  fl.kx = kx;
  fl.kv = kv;
  int updates = 0, confirmed = 0;
  for (int k = 0; k < sessions; ++k) {
    const auto v = session_verdict(tr, fl, 3, rng);
    updates += v == AxisVerdict::UPDATE;
    confirmed += v == AxisVerdict::CONFIRMED;
  }
  std::printf("alpha %.2f lag %.0f ms, gains %.2f/%.2f: %d/%d updated, %d confirmed\n", tr.alpha,
    1e3 * (tr.delay + tr.tau), kx, kv, updates, sessions, confirmed);
  return {static_cast<double>(updates) / sessions, static_cast<double>(confirmed) / sessions};
}

}  // namespace

// Sessions of up to 3 rounds in the field wind, as the conductor flies them.

TEST(DecisionRates, CorrectGainsAreLeftAloneInFieldWind)
{
  // Gains designed for the true plant (wn 1.6, zeta 0.95, alpha 1): the old
  // tuner's gates failed such a session 61 % of the time by chance. The
  // 160 ms lag puts their true phase margin at ~47 deg, just above the
  // 45 deg spec -- deliberately the hard case for the margin rule.
  const Truth tr{1.0, 0.0, 0.16, 0.13};
  EXPECT_LE(session_rates(tr, 2.56, 3.04, 30, 11).update, 0.10);
}

TEST(DecisionRates, AMaterialErrorIsNeverConfirmed)
{
  // Gains 25 % soft for the true plant (designed for alpha 1, plant 0.8): a
  // session may update or run out of rounds, but must essentially never
  // call them confirmed.
  const Truth tr{0.80, 0.0, 0.16, 0.13};
  EXPECT_LE(session_rates(tr, 2.56, 3.04, 30, 14).confirmed, 0.10);
}

TEST(DecisionRates, APlantGainErrorIsCorrected)
{
  // The plant delivers 70 % of the command: gains for alpha 1 are 30 % soft.
  const Truth tr{0.70, 0.0, 0.16, 0.13};
  EXPECT_GE(session_rates(tr, 2.56, 3.04, 30, 12).update, 0.90);
}

TEST(DecisionRates, ALagThatEatsTheMarginIsCaught)
{
  const Truth tr{1.0, 0.10, 0.25, 0.13};
  EXPECT_GE(session_rates(tr, 2.56, 3.04, 30, 13).update, 0.90);
}

// ------------------------------------------------------ recorded field flights

namespace
{

struct FlightResult
{
  AccelLoopResult id[3];
  SessionSample last;
};

FlightResult analyse(const std::string & name)
{
  const auto rec = SessionRecording::load_csv(
    std::string(GEO_TUNER_TEST_DATA_DIR) + "/field_accel/" + name);
  FlightResult r;
  for (int a = 0; a < 3; ++a) {r.id[a] = identify_accel_loop(rec.segments(a));}
  r.last = rec.samples.back();
  return r;
}

const char * kFlights[] = {
  "field_2026-09-10_12-19.csv", "field_2026-09-13_09-20.csv", "field_2026-09-13_10-38.csv"};

}  // namespace

TEST(FieldFlights, EveryAxisIdentifiesWithAPhysicalLag)
{
  for (const auto * f : kFlights) {
    const auto r = analyse(f);
    for (int a = 0; a < 3; ++a) {
      ASSERT_TRUE(r.id[a].ok) << f << " axis " << a << ": " << r.id[a].reason;
      EXPECT_GT(r.id[a].r2, 0.85) << f << " axis " << a;
      EXPECT_GT(r.id[a].alpha, 0.85) << f << " axis " << a;
      EXPECT_LT(r.id[a].alpha, 1.25) << f << " axis " << a;
    }
    // z is direct thrust; x/y go through the attitude loop.
    EXPECT_GE(r.id[2].lag, 0.04) << f;
    EXPECT_LE(r.id[2].lag, 0.12) << f;
    for (int a = 0; a < 2; ++a) {
      EXPECT_GE(r.id[a].lag, 0.10) << f << " axis " << a;
      EXPECT_LE(r.id[a].lag, 0.22) << f << " axis " << a;
    }
  }
}

TEST(FieldFlights, TheThreeFlightsAgree)
{
  // The property the old tuner never had: independent flights, different
  // gains and air, one plant. Every pair of alpha intervals overlaps.
  std::vector<FlightResult> rs;
  for (const auto * f : kFlights) {rs.push_back(analyse(f));}
  for (int a = 0; a < 3; ++a) {
    for (size_t i = 0; i < rs.size(); ++i) {
      for (size_t j = i + 1; j < rs.size(); ++j) {
        EXPECT_LE(rs[i].id[a].alpha_lo, rs[j].id[a].alpha_hi) << "axis " << a;
        EXPECT_LE(rs[j].id[a].alpha_lo, rs[i].id[a].alpha_hi) << "axis " << a;
      }
    }
  }
}

TEST(FieldFlights, VerdictsOnTheGainsEachFlightEndedWith)
{
  DecisionConfig cfg;
  // 09:20 ended on the gains flown since: x and y are supported. x has the
  // data to confirm it (alpha interval 1.15x); y had 5 s of excitation and a
  // 1.24x interval, short of the power a confirmation needs (1.19x) -- a
  // session would fly another round. Neither is an update.
  auto r = analyse("field_2026-09-13_09-20.csv");
  EXPECT_EQ(decide_axis(r.last.kx[0], r.last.kv[0], r.id[0], cfg).verdict,
    AxisVerdict::CONFIRMED);
  const auto dy = decide_axis(r.last.kx[1], r.last.kv[1], r.id[1], cfg);
  EXPECT_TRUE(dy.verdict == AxisVerdict::CONFIRMED || dy.verdict == AxisVerdict::INCONCLUSIVE)
    << dy.why;
  // z's kv 2.50 is zeta ~0.75, not the 0.95 design: every flight says raise it.
  for (const auto * f : kFlights) {
    r = analyse(f);
    const auto dz = decide_axis(r.last.kx[2], r.last.kv[2], r.id[2], cfg);
    EXPECT_EQ(dz.verdict, AxisVerdict::UPDATE) << f;
    EXPECT_GT(dz.kv_new, r.last.kv[2]) << f;
  }
  // 10:38 ended de-tuned on x (kx 1.50): corrected back up.
  r = analyse("field_2026-09-13_10-38.csv");
  const auto dx = decide_axis(r.last.kx[0], r.last.kv[0], r.id[0], cfg);
  EXPECT_EQ(dx.verdict, AxisVerdict::UPDATE);
  EXPECT_GT(dx.kx_new, 2.2);
}

// ----------------------------------------------------------- session record

TEST(SessionRecording, CsvRoundTripAndSegments)
{
  SessionRecording rec;
  for (int i = 0; i < 10; ++i) {
    SessionSample s;
    s.t = 0.02 * i;
    s.seg = i < 6 ? 0 : 1;
    s.r = {0.1 * i, 0.0, 5.0};
    s.v = {0.01 * i, 0.0, 0.0};
    s.u = {0.5, -0.5, 0.25};
    s.kx = {2.0, 2.1, 2.2};
    s.kv = {2.5, 2.6, 2.7};
    rec.samples.push_back(s);
  }
  const std::string path = "/tmp/geo_tuner_test_session.csv";
  ASSERT_TRUE(rec.save_csv(path));
  const auto back = SessionRecording::load_csv(path);
  ASSERT_EQ(back.size(), rec.size());
  EXPECT_NEAR(back.samples[7].r[0], 0.7, 1e-4);
  EXPECT_NEAR(back.samples[7].kv[2], 2.7, 1e-4);
  const auto segs = back.segments(0);
  ASSERT_EQ(segs.size(), 2u);
  EXPECT_EQ(segs[0].r.size(), 6u);
  EXPECT_EQ(segs[1].r.size(), 4u);
}
