// Emit a double as YAML the way Python's yaml.safe_dump did.
//
// yaml-cpp writes a native double with 17 significant digits and drops the
// decimal point when the value is integral, so 2.7 becomes
// "2.7000000000000002" and 2.0 becomes "2". The second one is not
// cosmetic: a generated geometric_controller.yaml with `x: 2` declares an
// INTEGER ROS parameter, and the controller declares gains.pos.x as a
// double -- the node then refuses to start on its own tuned config.
//
// Writing the number as a pre-formatted plain scalar fixes both: the
// shortest representation that round-trips, always carrying a '.' or an
// exponent so YAML types it as a float.
#ifndef GEO_TUNER__CORE__YAML_DOUBLE_HPP_
#define GEO_TUNER__CORE__YAML_DOUBLE_HPP_

#include <yaml-cpp/yaml.h>

#include <string>

namespace geo_tuner
{

/// Shortest round-tripping decimal form of `v`, always float-typed YAML.
std::string format_double(double v);

/// `v` as a YAML scalar node that emits like Python's yaml.safe_dump.
inline YAML::Node yaml_double(double v) {return YAML::Node(format_double(v));}

}  // namespace geo_tuner

#endif  // GEO_TUNER__CORE__YAML_DOUBLE_HPP_
