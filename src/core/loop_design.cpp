#include "geo_tuner/core/loop_design.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace geo_tuner
{
namespace
{

std::string fmt(double v, int precision)
{
  std::ostringstream os;
  os.setf(std::ios::fixed);
  os.precision(precision);
  os << v;
  return os.str();
}

double mag(double w, double kx, double kv, const LoopPlant & p)
{
  return p.alpha * std::hypot(kx, w * kv) /
         (w * w * std::hypot(1.0, w * p.tau));
}

/// Phase of L(jw) + 180 deg, in radians.
double phase_plus_pi(double w, double kx, double kv, const LoopPlant & p)
{
  return std::atan2(w * kv, kx) - w * p.delay - std::atan(w * p.tau);
}

LoopPlant scaled_lag(const AccelLoopResult & id, double alpha, double lag)
{
  LoopPlant p;
  p.alpha = alpha;
  if (id.lag > 1e-9) {
    const double k = lag / id.lag;
    p.delay = id.delay * k;
    p.tau = id.tau * k;
  } else {
    p.tau = lag;   // nothing to apportion: a lag is the milder assumption
  }
  return p;
}

}  // namespace

double phase_margin_deg(double kx, double kv, const LoopPlant & p, double * w_c)
{
  // |L| is strictly decreasing in w for this structure: bisect on log w.
  double lo = 1e-3, hi = 1e3;
  if (mag(lo, kx, kv, p) < 1.0) {lo = 1e-6;}
  for (int i = 0; i < 200; ++i) {
    const double mid = std::sqrt(lo * hi);
    (mag(mid, kx, kv, p) > 1.0 ? lo : hi) = mid;
  }
  const double w = std::sqrt(lo * hi);
  if (w_c) {*w_c = w;}
  return phase_plus_pi(w, kx, kv, p) * 180.0 / M_PI;
}

double gain_margin(double kx, double kv, const LoopPlant & p)
{
  if (p.delay <= 0.0 && p.tau <= 0.0) {return std::numeric_limits<double>::infinity();}
  // Phase crossover: phase_plus_pi(w) = 0, positive at low w, falling.
  double lo = 1e-3, hi = 1e3;
  if (phase_plus_pi(hi, kx, kv, p) > 0.0) {return std::numeric_limits<double>::infinity();}
  for (int i = 0; i < 200; ++i) {
    const double mid = std::sqrt(lo * hi);
    (phase_plus_pi(mid, kx, kv, p) > 0.0 ? lo : hi) = mid;
  }
  return 1.0 / mag(std::sqrt(lo * hi), kx, kv, p);
}

double nominal_phase_margin_deg(double kx, double kv, const AccelLoopResult & id)
{
  return phase_margin_deg(kx, kv, LoopPlant{id.alpha, id.delay, id.tau});
}

double worst_phase_margin_deg(double kx, double kv, const AccelLoopResult & id)
{
  double worst = std::numeric_limits<double>::infinity();
  for (double a : {id.alpha_lo, id.alpha, id.alpha_hi}) {
    if (!(a > 0.0)) {continue;}
    worst = std::min(worst, phase_margin_deg(kx, kv, scaled_lag(id, a, id.lag_hi)));
  }
  return worst;
}

LoopDesign design_lag_aware(
  double wn_target, double zeta, const AccelLoopResult & id, const MarginSpec & spec)
{
  const auto make = [&](double wn) {
      LoopDesign d;
      d.wn = wn;
      d.kx = wn * wn / id.alpha;
      d.kv = 2.0 * zeta * wn / id.alpha;
      d.pm_nominal_deg = nominal_phase_margin_deg(d.kx, d.kv, id);
      d.pm_worst_deg = worst_phase_margin_deg(d.kx, d.kv, id);
      return d;
    };
  const auto meets = [&](const LoopDesign & d) {
      return d.pm_nominal_deg >= spec.nominal_deg && d.pm_worst_deg >= spec.worst_deg;
    };
  LoopDesign d = make(wn_target);
  if (meets(d)) {return d;}
  // Phase margin falls as wn rises for this loop; bisect for the largest wn
  // that meets both margins.
  double lo = 0.05, hi = wn_target;
  if (!meets(make(lo))) {
    LoopDesign low = make(lo);
    low.limited = true;
    return low;
  }
  for (int i = 0; i < 60; ++i) {
    const double mid = 0.5 * (lo + hi);
    (meets(make(mid)) ? lo : hi) = mid;
  }
  d = make(lo);
  d.limited = true;
  return d;
}

const char * to_string(AxisVerdict v)
{
  switch (v) {
    case AxisVerdict::NO_ESTIMATE: return "no_estimate";
    case AxisVerdict::INCONCLUSIVE: return "inconclusive";
    case AxisVerdict::CONFIRMED: return "confirmed";
    case AxisVerdict::UPDATE: return "update";
  }
  return "?";
}

AxisDecision decide_axis(
  double kx_now, double kv_now, const AccelLoopResult & id, const DecisionConfig & cfg)
{
  AxisDecision dec;
  if (!id.ok) {
    dec.verdict = AxisVerdict::NO_ESTIMATE;
    dec.kx_new = kx_now;
    dec.kv_new = kv_now;
    dec.why = "no estimate: " + id.reason;
    return dec;
  }
  dec.design = design_lag_aware(cfg.wn_target, cfg.zeta, id, cfg.margins);
  dec.pm_now_nominal_deg = nominal_phase_margin_deg(kx_now, kv_now, id);
  dec.pm_now_worst_deg = worst_phase_margin_deg(kx_now, kv_now, id);

  // The gains the evidence supports: the design bandwidth at every alpha in
  // the interval, widened by the materiality tolerance.
  const double wn = dec.design.wn;
  const double widen = 1.0 + cfg.gain_tolerance;
  const double kx_lo = wn * wn / id.alpha_hi / widen, kx_hi = wn * wn / id.alpha_lo * widen;
  const double kv_lo = 2.0 * cfg.zeta * wn / id.alpha_hi / widen;
  const double kv_hi = 2.0 * cfg.zeta * wn / id.alpha_lo * widen;
  const bool in_range = kx_now >= kx_lo && kx_now <= kx_hi && kv_now >= kv_lo &&
    kv_now <= kv_hi;
  const double ci_ratio = id.alpha_hi / std::max(id.alpha_lo, 1e-9);
  const bool nominal_ok =
    dec.pm_now_nominal_deg >= cfg.margins.nominal_deg - cfg.pm_hysteresis_deg;
  const bool worst_ok = dec.pm_now_worst_deg >= cfg.margins.worst_deg - cfg.pm_hysteresis_deg;
  // Missing margin at the estimate is acted on whatever the interval. Missing
  // it only at the interval's worst corner needs an interval with power: the
  // worst corner of a 2.6x interval is not evidence about the plant, and
  // acting on it cut y's gains 38 % into gusts (quad_sim 2026-09-13 s3:
  // alpha [0.59, 1.51], nominal PM 44, worst 29; the soft gains then
  // overshot 92-159 % and failed validation).
  if (nominal_ok && !worst_ok && ci_ratio > cfg.max_alpha_ci_ratio) {
    dec.verdict = AxisVerdict::INCONCLUSIVE;
    dec.kx_new = kx_now;
    dec.kv_new = kv_now;
    dec.why = "PM " + fmt(dec.pm_now_nominal_deg, 0) + " deg at the estimate, worst " +
      fmt(dec.pm_now_worst_deg, 0) + " only at the corner of a " + fmt(ci_ratio, 2) +
      "x alpha interval (acting on the corner needs <= " + fmt(cfg.max_alpha_ci_ratio, 2) +
      "x); more data needed";
    return dec;
  }
  const bool margin_ok = nominal_ok && worst_ok;

  const std::string range_text = "kx " + fmt(kx_now, 2) + " in [" + fmt(kx_lo, 2) + ", " +
    fmt(kx_hi, 2) + "], kv " + fmt(kv_now, 2) + " in [" + fmt(kv_lo, 2) + ", " +
    fmt(kv_hi, 2) + "]";
  if (in_range && margin_ok) {
    // Consistent with the evidence. Calling that CONFIRMED needs power: a
    // wide interval contains wrong gains as easily as right ones.
    dec.kx_new = kx_now;
    dec.kv_new = kv_now;
    if (ci_ratio > cfg.max_alpha_ci_ratio) {
      dec.verdict = AxisVerdict::INCONCLUSIVE;
      dec.why = range_text + ", but alpha interval [" + fmt(id.alpha_lo, 2) + ", " +
        fmt(id.alpha_hi, 2) + "] is " + fmt(ci_ratio, 2) + "x wide (confirming needs <= " +
        fmt(cfg.max_alpha_ci_ratio, 2) + "x); more data needed";
    } else {
      dec.verdict = AxisVerdict::CONFIRMED;
      dec.why = range_text + ", PM " + fmt(dec.pm_now_nominal_deg, 0) + " deg (worst " +
        fmt(dec.pm_now_worst_deg, 0) + ")";
    }
    return dec;
  }

  // UPDATE. Outside the widened range is significant at the interval's level
  // however wide it is, so no power requirement here -- but the move is the
  // SMALLEST the evidence supports, not a jump to the point estimate: gains
  // go to the nearest edge of the supported range (the design at the upper
  // alpha for soft gains, at the lower alpha for stiff ones). On a narrow
  // interval both edges are the nominal design; on a wide one the step is
  // conservative and the next round, with more data, refines it. Jumping to
  // the point estimate on a 1.4x interval overshot the truth by 13-16 % and
  // failed validation (quad_sim 2026-09-13, s2 x, s3 y). Missing margin goes
  // to the soft edge.
  const double kx_soft = wn * wn / id.alpha_hi, kx_stiff = wn * wn / id.alpha_lo;
  const double kv_soft = 2.0 * cfg.zeta * wn / id.alpha_hi;
  const double kv_stiff = 2.0 * cfg.zeta * wn / id.alpha_lo;
  double kx_t = margin_ok ? std::clamp(kx_now, kx_soft, kx_stiff) : kx_soft;
  double kv_t = margin_ok ? std::clamp(kv_now, kv_soft, kv_stiff) : kv_soft;
  std::string target = margin_ok ? "nearest edge of the supported range" :
    "soft edge of the supported range";
  // The target must meet the margins itself; the nominal design does by
  // construction.
  if (nominal_phase_margin_deg(kx_t, kv_t, id) < cfg.margins.nominal_deg ||
    worst_phase_margin_deg(kx_t, kv_t, id) < cfg.margins.worst_deg)
  {
    kx_t = dec.design.kx;
    kv_t = dec.design.kv;
    target = "nominal design";
  }
  dec.verdict = AxisVerdict::UPDATE;
  dec.kx_new = std::clamp(kx_t, kx_now / cfg.max_change, kx_now * cfg.max_change);
  dec.kv_new = std::clamp(kv_t, kv_now / cfg.max_change, kv_now * cfg.max_change);
  dec.clipped = dec.kx_new != kx_t || dec.kv_new != kv_t;
  dec.why = std::string(margin_ok ? "" : "PM " + fmt(dec.pm_now_nominal_deg, 0) +
    " deg (worst " + fmt(dec.pm_now_worst_deg, 0) + ") below " +
    fmt(cfg.margins.nominal_deg - cfg.pm_hysteresis_deg, 0) + "/" +
    fmt(cfg.margins.worst_deg - cfg.pm_hysteresis_deg, 0) + "; ") +
    (in_range ? "" : "gains outside the supported range (" + range_text + "); ") +
    "design wn " + fmt(wn, 2) + (dec.design.limited ? " (lowered for phase margin)" : "") +
    " -> kx " + fmt(dec.kx_new, 2) + " kv " + fmt(dec.kv_new, 2) + " (" + target +
    (dec.clipped ? ", rate-limited" : "") + ")";
  return dec;
}

ValidationResult validate_gains(
  double kx, double kv, const AccelLoopResult & id_new, const AccelLoopResult & id_prior,
  double overshoot_median, const MarginSpec & spec, double max_overshoot)
{
  ValidationResult v;
  if (!id_new.ok) {
    v.why = "validation data did not identify: " + id_new.reason;
    return v;
  }
  v.pm_nominal_deg = nominal_phase_margin_deg(kx, kv, id_new);
  v.pm_worst_deg = worst_phase_margin_deg(kx, kv, id_new);
  if (v.pm_nominal_deg < spec.worst_deg) {
    v.why = "phase margin on validation data " + fmt(v.pm_nominal_deg, 0) +
      " deg is below the " + fmt(spec.worst_deg, 0) + " deg floor";
    return v;
  }
  if (id_prior.ok && (id_new.alpha_hi < id_prior.alpha_lo || id_new.alpha_lo > id_prior.alpha_hi)) {
    v.why = "plant changed between design and validation: alpha [" +
      fmt(id_prior.alpha_lo, 2) + ", " + fmt(id_prior.alpha_hi, 2) + "] -> [" +
      fmt(id_new.alpha_lo, 2) + ", " + fmt(id_new.alpha_hi, 2) + "]";
    return v;
  }
  if (overshoot_median > max_overshoot) {
    v.why = "measured overshoot " + fmt(100.0 * overshoot_median, 0) + " % > " +
      fmt(100.0 * max_overshoot, 0) + " %";
    return v;
  }
  v.pass = true;
  v.why = "validated: PM " + fmt(v.pm_nominal_deg, 0) + " deg (worst " +
    fmt(v.pm_worst_deg, 0) + "), overshoot " +
    fmt(100.0 * overshoot_median, 0) + " %";
  return v;
}

}  // namespace geo_tuner
