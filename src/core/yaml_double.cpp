#include "geo_tuner/core/yaml_double.hpp"

#include <cmath>
#include <sstream>

namespace geo_tuner
{

std::string format_double(double v)
{
  if (std::isnan(v)) {return ".nan";}
  if (std::isinf(v)) {return v > 0 ? ".inf" : "-.inf";}

  // Exponent notation only where Python's repr would use it; otherwise a
  // short precision makes the default float format pick it anyway
  // (50.0 at 1 significant digit is "5e+01"), which is legal YAML but not
  // what the reports and configs used to contain.
  const double mag = std::abs(v);
  const bool allow_exp = v != 0.0 && (mag < 1e-4 || mag >= 1e16);

  std::string s;
  for (int p = 1; p <= 17; ++p) {
    std::ostringstream os;
    os.precision(p);
    os << v;
    s = os.str();
    if (!allow_exp &&
      (s.find('e') != std::string::npos || s.find('E') != std::string::npos))
    {
      continue;
    }
    if (std::stod(s) == v) {break;}
  }
  // A bare "2" is an integer in YAML; the consumers of these files want a
  // double. An exponent form ("1e-05") is already typed float.
  if (s.find('.') == std::string::npos && s.find('e') == std::string::npos &&
    s.find('E') == std::string::npos)
  {
    s += ".0";
  }
  return s;
}

}  // namespace geo_tuner
