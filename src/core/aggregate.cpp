#include "geo_tuner/core/aggregate.hpp"

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

}  // namespace

double median(std::vector<double> vals)
{
  if (vals.empty()) {return 0.0;}
  std::sort(vals.begin(), vals.end());
  const size_t n = vals.size();
  const size_t m = n / 2;
  return (n % 2) ? vals[m] : 0.5 * (vals[m - 1] + vals[m]);
}

RobustEstimate robust_ratio_estimate(
  const std::vector<double> & values, int min_count, double max_spread)
{
  std::vector<double> vals;
  for (double v : values) {
    if (v > 0.0 && std::isfinite(v)) {vals.push_back(v);}
  }
  const int n = static_cast<int>(vals.size());
  if (n == 0) {
    return {std::numeric_limits<double>::quiet_NaN(), 0,
      std::numeric_limits<double>::infinity(), false, "no accepted estimates"};
  }
  const double med = median(vals);
  const double spread =
    *std::max_element(vals.begin(), vals.end()) /
    *std::min_element(vals.begin(), vals.end());
  if (n < min_count) {
    return {med, n, spread, false,
      "only " + std::to_string(n) + " accepted estimate(s), need >= " +
      std::to_string(min_count)};
  }
  if (spread > max_spread) {
    return {med, n, spread, false,
      "estimates inconsistent: spread " + fmt(spread, 2) + "x > " +
      fmt(max_spread, 2) + "x"};
  }
  return {med, n, spread, true, ""};
}

}  // namespace geo_tuner
