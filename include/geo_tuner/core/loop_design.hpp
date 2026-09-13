// Gain design and gain decisions from an identified acceleration loop.
//
// The position loop the controller closes on one axis is
//
//     L(s) = alpha * (kv s + kx) * e^(-d s) / (s^2 (tau s + 1))
//
// with (alpha, d, tau) from identify_accel_loop(). The old design used the
// delay-free second order (kx = wn^2/alpha, kv = 2 zeta wn / alpha) and
// then spent all of its logic estimating alpha -- the parameter that barely
// matters: for wn 1.6, zeta 0.95 and a 180 ms lag, alpha anywhere in
// 0.7-1.3 moves the phase margin by +/-2 deg, while the lag moves it from
// 75 deg (no lag) to 39 deg (250 ms). So here:
//
//   * the lag is part of the design: the requested bandwidth is kept only
//     if the loop keeps the classical robustness pair -- a nominal phase
//     margin of 45 deg at the identified plant AND a floor of 35 deg on the
//     worst plant inside the identification's confidence interval --
//     otherwise it is lowered until it does. These are textbook values
//     (45-60 deg nominal, never below ~30-35 deg under uncertainty), chosen
//     before looking at any flight, not fitted to one;
//   * gains in force are UPDATED when they lack the phase margin or lie
//     outside what the alpha interval supports (widened by a materiality
//     tolerance) -- both significant at the interval's level however wide it
//     is -- and then only by the smallest move the evidence supports: to the
//     nearest edge of the supported range, not to the point estimate;
//   * gains inside the range are CONFIRMED only when the interval is tight
//     enough to have power (max_alpha_ci_ratio); otherwise INCONCLUSIVE and
//     more data is flown;
//   * applied gains are accepted only after a validation on data they flew
//     (validate_gains), never on the numbers that designed them. Validation
//     is the safety test -- is there evidence the margin floor is broken? --
//     not a repeat of the design target: a threshold at or above what the
//     design guarantees rejects correct updates about half the time on a
//     noisy one-round estimate. Whether the validated gains are the best
//     ones is decided again on the pooled data by decide_axis.
#ifndef GEO_TUNER__CORE__LOOP_DESIGN_HPP_
#define GEO_TUNER__CORE__LOOP_DESIGN_HPP_

#include <string>

#include "geo_tuner/core/accel_loop_id.hpp"

namespace geo_tuner
{

struct LoopPlant
{
  double alpha{1.0};
  double delay{0.0};   // [s]
  double tau{0.0};     // [s]
};

/// Phase margin [deg] of L(s); w_c (optional) receives the crossover [rad/s].
double phase_margin_deg(double kx, double kv, const LoopPlant & p, double * w_c = nullptr);

/// Factor by which alpha may grow before instability (inf without lag/delay).
double gain_margin(double kx, double kv, const LoopPlant & p);

/// Phase margin at the identified (point-estimate) plant.
double nominal_phase_margin_deg(double kx, double kv, const AccelLoopResult & id);

/// Worst (smallest) phase margin over the corners of the identification's
/// confidence interval: alpha at both ends, lag at its upper end.
double worst_phase_margin_deg(double kx, double kv, const AccelLoopResult & id);

struct MarginSpec
{
  double nominal_deg{45.0};
  double worst_deg{35.0};
};

struct LoopDesign
{
  double wn{0.0};        // bandwidth actually designed for [rad/s]
  double kx{0.0}, kv{0.0};
  double pm_nominal_deg{0.0};
  double pm_worst_deg{0.0};
  bool limited{false};   // wn lowered below the request to keep the margins
};

/// kx = wn^2/alpha, kv = 2 zeta wn/alpha at the identified alpha, with wn the
/// largest value <= wn_target meeting both margins of `spec`.
LoopDesign design_lag_aware(
  double wn_target, double zeta, const AccelLoopResult & id, const MarginSpec & spec);

enum class AxisVerdict
{
  NO_ESTIMATE,   // identification refused; gains untouched
  INCONCLUSIVE,  // gains not contradicted, but the evidence is too imprecise to confirm
  CONFIRMED,     // gains in force are supported by precise enough evidence
  UPDATE,        // gains in force are not; apply kx_new/kv_new
};

const char * to_string(AxisVerdict v);

struct AxisDecision
{
  AxisVerdict verdict{AxisVerdict::NO_ESTIMATE};
  double kx_new{0.0}, kv_new{0.0};
  LoopDesign design;
  double pm_now_nominal_deg{0.0};
  double pm_now_worst_deg{0.0};
  bool clipped{false};   // update limited by max_change
  std::string why;
};

struct DecisionConfig
{
  double wn_target{1.6};
  double zeta{0.95};
  MarginSpec margins;
  double gain_tolerance{0.10};   // materiality: gains within this of the CI range stand
  // Materiality on the margins, the same band validation allows: gains in
  // force stand while they keep margins - this; a design always targets the
  // full margins. Without it, gains a few degrees above the spec are
  // "updated" whenever estimation noise dips the estimate below it.
  double pm_hysteresis_deg{5.0};
  // Power requirement for a performance verdict (CONFIRMED, or UPDATE of
  // gains that keep their margins). Gains are confirmed within gain_tolerance
  // of the range the interval supports, so with the truth at the interval's
  // centre a gain error up to (1 + gain_tolerance) * sqrt(ratio) can pass.
  // Derived from the materiality it must guarantee, not tuned: confirmed
  // gains within 20 % of the design needs ratio <= (1.20 / 1.10)^2 = 1.19.
  // (1.30 allowed ~25 % and confirmed 25 %-soft gains in 5 of 30 three-round
  // sessions.) A wider interval that still contains the gains proves nothing.
  double max_alpha_ci_ratio{1.19};
  double max_change{1.6};        // factor limit on one update
};

AxisDecision decide_axis(
  double kx_now, double kv_now, const AccelLoopResult & id, const DecisionConfig & cfg);

struct ValidationResult
{
  bool pass{false};
  double pm_nominal_deg{0.0};
  double pm_worst_deg{0.0};
  std::string why;
};

/// Out-of-sample safety acceptance of gains that were applied and then flown.
/// Fails when the validation round shows the margin floor broken (nominal
/// phase margin at its own estimate below spec.worst_deg), the plant changed
/// (alpha intervals of design and validation disjoint), or the steps rang
/// (median overshoot above max_overshoot).
///
/// id_new   identification from the validation round's data ONLY
/// id_prior identification that designed the gains
/// overshoot_median  measured straight from the validation step records
ValidationResult validate_gains(
  double kx, double kv, const AccelLoopResult & id_new, const AccelLoopResult & id_prior,
  double overshoot_median, const MarginSpec & spec, double max_overshoot);

}  // namespace geo_tuner

#endif  // GEO_TUNER__CORE__LOOP_DESIGN_HPP_
