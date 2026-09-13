// geo-tuner-identify: the conductor's identification and gain decision,
// offline, on a session recording.
//
//   geo-tuner-identify SESSION.csv [--wn 1.6] [--wn-z W] [--zeta 0.95]
//                      [--pm 45] [--pm-worst 35] [--tol 0.10] [--from T]
//
// Prints, per position axis, the identified acceleration loop with its
// confidence interval and the verdict for the gains in force at the end of
// the recording (or at --from's start, when given, using data after T).
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

#include "geo_tuner/core/loop_design.hpp"
#include "geo_tuner/core/session_recording.hpp"

using namespace geo_tuner;  // NOLINT(build/namespaces)

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "usage: geo-tuner-identify SESSION.csv [--wn W] [--wn-z W] [--zeta Z] "
      "[--pm DEG] [--pm-worst DEG] [--tol F] [--from T]\n";
    return 2;
  }
  DecisionConfig dc;
  double wn_z = -1.0, from_t = -1.0;
  for (int i = 2; i + 1 < argc; i += 2) {
    const std::string k = argv[i];
    const double v = std::stod(argv[i + 1]);
    if (k == "--wn") {dc.wn_target = v;} else if (k == "--wn-z") {wn_z = v;} else if (
      k == "--zeta")
    {
      dc.zeta = v;
    } else if (k == "--pm") {dc.margins.nominal_deg = v;} else if (k == "--pm-worst") {
      dc.margins.worst_deg = v;
    } else if (k == "--tol") {
      dc.gain_tolerance = v;
    } else if (k == "--from") {from_t = v;} else {
      std::cerr << "unknown option " << k << "\n";
      return 2;
    }
  }
  SessionRecording rec;
  try {
    rec = SessionRecording::load_csv(argv[1]);
  } catch (const std::exception & e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
  if (rec.samples.empty()) {
    std::cerr << "empty recording\n";
    return 1;
  }
  size_t first = 0;
  if (from_t >= 0.0) {
    while (first < rec.samples.size() && rec.samples[first].t < from_t) {++first;}
  }
  const double dur = rec.samples.back().t - rec.samples[first].t;
  AccelLoopConfig cfg;
  if (rec.samples.size() > first + 1) {
    cfg.fs = static_cast<double>(rec.samples.size() - first - 1) / dur;
    cfg.fs = std::round(cfg.fs);
  }
  std::printf("%s: %zu samples, %.0f s, fs %.0f Hz\n", argv[1], rec.samples.size() - first,
    dur, cfg.fs);
  const auto & last = rec.samples.back();
  for (int a = 0; a < 3; ++a) {
    const char ax = "xyz"[a];
    const auto id = identify_accel_loop(rec.segments(a, first), cfg);
    DecisionConfig d = dc;
    if (a == 2 && wn_z > 0.0) {d.wn_target = wn_z;}
    std::printf("%c: alpha %.3f [%.3f, %.3f]  lag %3.0f ms [%3.0f, %3.0f] (delay %.0f + tau %.0f)"
      "  R2 %.2f (raw %.2f)  excited %.0f s%s%s\n", ax, id.alpha, id.alpha_lo, id.alpha_hi,
      1e3 * id.lag, 1e3 * id.lag_lo, 1e3 * id.lag_hi, 1e3 * id.delay, 1e3 * id.tau, id.r2,
      id.r2_raw, id.excited_s,
      id.ok ? "" : "  REFUSED: ", id.ok ? "" : id.reason.c_str());
    const auto dec = decide_axis(last.kx[a], last.kv[a], id, d);
    std::printf("   gains in force kx %.3f kv %.3f -> %s: %s\n", last.kx[a], last.kv[a],
      to_string(dec.verdict), dec.why.c_str());
    if (dec.verdict != AxisVerdict::NO_ESTIMATE) {
      std::printf("   design wn %.2f: kx %.3f kv %.3f, PM %.0f deg (worst %.0f)%s\n",
        dec.design.wn, dec.design.kx, dec.design.kv, dec.design.pm_nominal_deg,
        dec.design.pm_worst_deg,
        dec.design.limited ? " (wn limited by phase margin)" : "");
    }
  }
  return 0;
}
