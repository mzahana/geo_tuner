// Robust aggregation of repeated per-episode identifications.
//
// A single 6-second step fit is a noisy estimator of the plant-gain
// factor alpha (position axes) or the closed-loop time constant T (yaw):
// process noise, the structural 2nd-order approximation of a truly
// higher-order lateral response, and delay/wn ambiguity all inject
// episode-to-episode variance that maps 1:1 (inverted) into the applied
// gains.
//
// The fix is statistical, not structural: repeat the step N times per
// (axis, rung), aggregate with the *median* (robust to one corrupted
// episode), and refuse to update at all when the surviving estimates
// disagree by more than a consistency factor -- inconsistent estimates
// mean the identification, not the plant, is the problem, and applying
// any of them would just bake noise into the gains.
#ifndef GEO_TUNER__CORE__AGGREGATE_HPP_
#define GEO_TUNER__CORE__AGGREGATE_HPP_

#include <string>
#include <vector>

namespace geo_tuner
{

/// Outcome of aggregating N repeated positive-ratio estimates.
struct RobustEstimate
{
  double value{};      // median of the accepted estimates (NaN if none)
  int n_used{};        // how many estimates went into the median
  double spread{};     // max/min ratio of the used estimates (>= 1)
  bool ok{false};      // enough estimates and spread within the gate
  std::string reason;  // human-readable rejection reason if not ok
};

/// Aggregate repeated estimates of a positive ratio-type quantity
/// (alpha, T) into one robust value.
///
/// values      accepted per-episode estimates (already gated for fit
///             quality and physical plausibility); must be > 0
/// min_count   minimum number of estimates required to act at all
/// max_spread  max/min ratio allowed among the estimates. 1.35 means the
///             worst pair disagrees by <= 35% -- beyond that the
///             session's identification is inconsistent and no gain
///             update should be made from it.
RobustEstimate robust_ratio_estimate(
  const std::vector<double> & values, int min_count = 2, double max_spread = 1.35);

/// Median of a non-empty list (0.0 for an empty one).
double median(std::vector<double> vals);

/// Accumulates per-episode estimates for one (axis, rung) pair.
///
/// `alphas` holds the quantity the gain update is computed from (the
/// plant-gain factor on a position axis, the closed-loop time constant on
/// yaw). `lags` holds the identified in-loop lag, which the bandwidth
/// ladder uses for its stability margin.
struct EpisodeBucket
{
  std::vector<double> alphas;
  std::vector<double> lags;

  void add(double alpha, double lag)
  {
    alphas.push_back(alpha);
    lags.push_back(lag);
  }
  int count() const {return static_cast<int>(alphas.size());}
  double median_lag() const {return lags.empty() ? 0.0 : median(lags);}
};

}  // namespace geo_tuner

#endif  // GEO_TUNER__CORE__AGGREGATE_HPP_
