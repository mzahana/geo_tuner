// geo-tuner-identify: the conductor's identification and gain decision,
// offline, on a session recording.
//
//   geo-tuner-identify SESSION.csv [--wn 1.6] [--wn-z W] [--zeta 0.95]
//                      [--pm 45] [--pm-worst 35] [--tol 0.10] [--from T]
//                      [--segments A-B] [--json]
//
// Prints, per position axis, the identified acceleration loop with its
// confidence interval and the verdict for the gains in force at the end of
// the recording (or at --from's start, when given, using data after T).
//
// --segments A-B restricts to the recording's seg ids A..B (one seg per
// round in practice), so the doctor can identify a round independently.
// --json prints the same result as one JSON document for the doctor.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <set>
#include <string>
#include <vector>

#include "geo_tuner/core/loop_design.hpp"
#include "geo_tuner/core/session_recording.hpp"

using namespace geo_tuner;  // NOLINT(build/namespaces)

namespace
{

std::string json_escape(const std::string & s)
{
  std::string out;
  for (const char c : s) {
    if (c == '"' || c == '\\') {out += '\\';}
    out += c;
  }
  return out;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "usage: geo-tuner-identify SESSION.csv [--wn W] [--wn-z W] [--zeta Z] "
      "[--pm DEG] [--pm-worst DEG] [--tol F] [--from T] [--segments A-B] [--json]\n";
    return 2;
  }
  DecisionConfig dc;
  double wn_z = -1.0, from_t = -1.0;
  int seg_lo = -1, seg_hi = -1;
  bool json = false;
  for (int i = 2; i < argc; ++i) {
    const std::string k = argv[i];
    if (k == "--json") {json = true; continue;}
    if (i + 1 >= argc) {
      std::cerr << "missing value for " << k << "\n";
      return 2;
    }
    const std::string val = argv[++i];
    if (k == "--segments") {
      const auto dash = val.find('-');
      if (dash == std::string::npos) {
        seg_lo = seg_hi = std::stoi(val);
      } else {
        seg_lo = std::stoi(val.substr(0, dash));
        seg_hi = std::stoi(val.substr(dash + 1));
      }
      continue;
    }
    const double v = std::stod(val);
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
  if (seg_lo >= 0) {
    SessionRecording sel;
    for (const auto & s : rec.samples) {
      if (s.seg >= seg_lo && s.seg <= seg_hi) {sel.samples.push_back(s);}
    }
    rec = sel;
  }
  if (rec.samples.empty()) {
    std::cerr << "empty recording\n";
    return 1;
  }
  size_t first = 0;
  if (from_t >= 0.0) {
    while (first < rec.samples.size() && rec.samples[first].t < from_t) {++first;}
  }
  std::set<int> segs;
  for (size_t i = first; i < rec.samples.size(); ++i) {segs.insert(rec.samples[i].seg);}
  const double dur = rec.samples.back().t - rec.samples[first].t;
  AccelLoopConfig cfg;
  if (rec.samples.size() > first + 1) {
    cfg.fs = static_cast<double>(rec.samples.size() - first - 1) / dur;
    cfg.fs = std::round(cfg.fs);
  }
  if (!json) {
    std::printf("%s: %zu samples, %.0f s, fs %.0f Hz\n", argv[1], rec.samples.size() - first,
      dur, cfg.fs);
  } else {
    std::printf("{\"file\": \"%s\", \"n_samples\": %zu, \"duration_s\": %.1f, \"fs\": %.0f, "
      "\"segments\": [", json_escape(argv[1]).c_str(), rec.samples.size() - first, dur, cfg.fs);
    bool sep = false;
    for (const int s : segs) {
      std::printf("%s%d", sep ? ", " : "", s);
      sep = true;
    }
    std::printf("], \"axes\": {");
  }
  const auto & last = rec.samples.back();
  for (int a = 0; a < 3; ++a) {
    const char ax = "xyz"[a];
    const auto id = identify_accel_loop(rec.segments(a, first), cfg);
    DecisionConfig d = dc;
    if (a == 2 && wn_z > 0.0) {d.wn_target = wn_z;}
    const auto dec = decide_axis(last.kx[a], last.kv[a], id, d);
    if (json) {
      std::printf("%s\"%c\": {\"ok\": %s, ", a ? ", " : "", ax, id.ok ? "true" : "false");
      if (!id.ok) {std::printf("\"reason\": \"%s\", ", json_escape(id.reason).c_str());}
      std::printf(
        "\"alpha\": %.4f, \"alpha_ci\": [%.4f, %.4f], "
        "\"lag_ms\": %.1f, \"lag_ci_ms\": [%.1f, %.1f], \"delay_ms\": %.1f, \"tau_ms\": %.1f, "
        "\"r2\": %.3f, \"r2_raw\": %.3f, \"excited_s\": %.1f, "
        "\"kx\": %.4f, \"kv\": %.4f, \"verdict\": \"%s\", \"why\": \"%s\", "
        "\"pm_now_nominal_deg\": %.1f, \"pm_now_worst_deg\": %.1f, "
        "\"design\": {\"wn\": %.3f, \"kx\": %.4f, \"kv\": %.4f, "
        "\"pm_nominal_deg\": %.1f, \"pm_worst_deg\": %.1f, \"limited\": %s}}",
        id.alpha, id.alpha_lo, id.alpha_hi,
        1e3 * id.lag, 1e3 * id.lag_lo, 1e3 * id.lag_hi, 1e3 * id.delay, 1e3 * id.tau,
        id.r2, id.r2_raw, id.excited_s,
        last.kx[a], last.kv[a], to_string(dec.verdict), json_escape(dec.why).c_str(),
        dec.pm_now_nominal_deg, dec.pm_now_worst_deg,
        dec.design.wn, dec.design.kx, dec.design.kv,
        dec.design.pm_nominal_deg, dec.design.pm_worst_deg,
        dec.design.limited ? "true" : "false");
      continue;
    }
    std::printf("%c: alpha %.3f [%.3f, %.3f]  lag %3.0f ms [%3.0f, %3.0f] (delay %.0f + tau %.0f)"
      "  R2 %.2f (raw %.2f)  excited %.0f s%s%s\n", ax, id.alpha, id.alpha_lo, id.alpha_hi,
      1e3 * id.lag, 1e3 * id.lag_lo, 1e3 * id.lag_hi, 1e3 * id.delay, 1e3 * id.tau, id.r2,
      id.r2_raw, id.excited_s,
      id.ok ? "" : "  REFUSED: ", id.ok ? "" : id.reason.c_str());
    std::printf("   gains in force kx %.3f kv %.3f -> %s: %s\n", last.kx[a], last.kv[a],
      to_string(dec.verdict), dec.why.c_str());
    if (dec.verdict != AxisVerdict::NO_ESTIMATE) {
      std::printf("   design wn %.2f: kx %.3f kv %.3f, PM %.0f deg (worst %.0f)%s\n",
        dec.design.wn, dec.design.kx, dec.design.kv, dec.design.pm_nominal_deg,
        dec.design.pm_worst_deg,
        dec.design.limited ? " (wn limited by phase margin)" : "");
    }
  }
  if (json) {std::printf("}}\n");}
  return 0;
}
