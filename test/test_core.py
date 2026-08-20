import math

import numpy as np
import pytest

from geo_tuner.core.gain_design import (
    GainSet, LoopShape, VehicleParams, correct_gains_from_identification,
    design_gains, geometric_controller_yaml, geometric_mavros_yaml,
    pd_from_wn_zeta, wn_zeta_from_pd)
from geo_tuner.core.safety import (
    OdomSample, SafetyLimits, SafetyMonitor, Violation, tilt_from_quat)
from geo_tuner.core.step_fit import fit_step_response, second_order_step


class TestGainDesign:
    def test_pd_roundtrip(self):
        kx, kv = pd_from_wn_zeta(2.0, 0.9)
        wn, zeta = wn_zeta_from_pd(kx, kv)
        assert wn == pytest.approx(2.0)
        assert zeta == pytest.approx(0.9)

    def test_repo_defaults_map_to_sane_wn(self):
        # kx=7.4, kv=4.8 from the node defaults
        wn, zeta = wn_zeta_from_pd(7.4, 4.8)
        assert 2.5 < wn < 3.0
        assert 0.8 < zeta < 1.0

    def test_max_thrust_from_hover(self):
        v = VehicleParams(mass=2.5, hover_throttle=0.5)
        assert v.max_thrust == pytest.approx(2.5 * 9.81 / 0.5)

    def test_design_respects_separation_cap(self):
        v = VehicleParams(mass=2.5, hover_throttle=0.45)
        s = LoopShape(attctrl_tau=0.3, timescale_separation=4.0,
                      latency=0.01)  # latency not binding
        g = design_gains(v, s)
        assert g.wn_xy == pytest.approx((2.0 / 0.3) / 4.0)
        assert g.kx[0] == pytest.approx(g.wn_xy ** 2, rel=1e-2)
        assert g.kv[0] == pytest.approx(2 * s.zeta * g.wn_xy, rel=1e-2)

    def test_design_respects_latency_cap(self):
        v = VehicleParams(mass=2.5, hover_throttle=0.45)
        s = LoopShape(latency=0.2, latency_margin=0.3)  # cap at 1.5 rad/s
        g = design_gains(v, s)
        assert g.wn_xy == pytest.approx(1.5)
        assert any("latency" in n for n in g.notes)

    def test_request_clipping(self):
        v = VehicleParams(mass=2.5, hover_throttle=0.45)
        s = LoopShape()
        g = design_gains(v, s, wn_request=100.0)
        assert g.wn_xy < 100.0
        assert any("clipped" in n for n in g.notes)

    def test_z_stiffer_than_xy(self):
        v = VehicleParams(mass=2.5, hover_throttle=0.45)
        g = design_gains(v, LoopShape())
        assert g.kx[2] > g.kx[0]

    def test_identification_correction(self):
        # Plant has 20% low effective gain (alpha=0.8): measured wn lower
        kx_applied = 4.0
        alpha_true = 0.8
        wn_meas = math.sqrt(alpha_true * kx_applied)
        kx_new, kv_new, alpha = correct_gains_from_identification(
            kx_applied, wn_meas, wn_target=2.0, zeta_target=0.9)
        assert alpha == pytest.approx(alpha_true)
        # With corrected gains the effective wn hits the target
        assert math.sqrt(alpha_true * kx_new) == pytest.approx(2.0)
        assert alpha_true * kv_new == pytest.approx(2 * 0.9 * 2.0)

    def test_yaml_generation_complete(self):
        v = VehicleParams(mass=2.5, hover_throttle=0.45)
        g = design_gains(v, LoopShape())
        y = geometric_controller_yaml(g, mass=2.5)
        p = y["geometric_controller_node"]["ros__parameters"]
        assert p["mass"] == 2.5
        assert p["gains"]["pos"]["x"] == g.kx[0]
        assert p["gains"]["ki"]["x"] == 0.0
        m = geometric_mavros_yaml(g)
        assert m["geometric_mavros_node"]["ros__parameters"]["max_thrust"] == g.max_thrust


class TestStepFit:
    def _simulate(self, wn, zeta, delay, step, t_end=6.0, dt=0.01, noise=0.0,
                  seed=0):
        t = np.arange(0.0, t_end, dt)
        y = step * second_order_step(t - delay, wn, zeta)
        if noise > 0:
            y = y + np.random.default_rng(seed).normal(0, noise, t.size)
        return t, y

    @pytest.mark.parametrize("wn,zeta", [(1.6, 0.95), (2.5, 0.7), (1.0, 1.2)])
    def test_recovers_known_system(self, wn, zeta):
        t, y = self._simulate(wn, zeta, delay=0.08, step=0.5)
        r = fit_step_response(t, y, step=0.5)
        assert r.ok
        assert r.wn == pytest.approx(wn, rel=0.05)
        assert r.zeta == pytest.approx(zeta, rel=0.10)
        assert r.delay == pytest.approx(0.08, abs=0.03)

    def test_robust_to_noise(self):
        t, y = self._simulate(1.8, 0.9, delay=0.06, step=0.5, noise=0.02)
        r = fit_step_response(t, y, step=0.5)
        assert r.ok
        assert r.wn == pytest.approx(1.8, rel=0.10)

    def test_bad_fit_flagged(self):
        rng = np.random.default_rng(1)
        t = np.arange(0, 5, 0.01)
        y = rng.normal(0, 0.5, t.size)  # pure noise, no response
        r = fit_step_response(t, y, step=0.5)
        assert not r.ok

    def test_underdamped_overshoot_measured(self):
        t, y = self._simulate(2.0, 0.4, delay=0.0, step=1.0)
        r = fit_step_response(t, y, step=1.0)
        # zeta=0.4 -> ~25% overshoot
        assert 0.15 < r.overshoot < 0.35

    def test_rejects_short_data(self):
        with pytest.raises(ValueError):
            fit_step_response(np.arange(5), np.zeros(5), step=0.5)

    def test_bound_pinned_fit_rejected(self):
        # A ramp (pure integrator response) cannot be explained by the
        # 2nd-order model inside the bounds; params pin and ok must be False
        t = np.arange(0, 6, 0.01)
        y = 0.02 * t  # slow ramp, never settles
        r = fit_step_response(t, y, step=0.5)
        assert not r.ok

    def test_ambiguous_higher_order_response_prefers_prior(self):
        # True plant: 2nd order (wn=1.41, zeta=0.95) cascaded with an
        # inner-loop lag (tau=0.15 s) — the case where a naive fit locks
        # onto "high wn, overdamped, huge delay" (alpha ~ 3.7). The
        # multi-start fitter must return wn near the truth instead.
        from scipy.signal import lsim, TransferFunction
        wn, zeta, tau = 1.41, 0.95, 0.15
        num = [wn * wn]
        den = np.polymul([1, 2 * zeta * wn, wn * wn], [tau, 1])
        t = np.arange(0, 6, 0.01)
        _, y, _ = lsim(TransferFunction(num, den), np.ones_like(t), t)
        r = fit_step_response(t, 0.5 * y, step=0.5, wn_guess=wn,
                              zeta_guess=zeta)
        alpha = r.wn ** 2 / wn ** 2
        assert 0.4 <= alpha <= 2.5, f"alpha={alpha:.2f} (wn={r.wn:.2f})"

    def test_zero_delay_is_legitimate(self):
        # delay pinned at its LOWER bound (0) must not reject the fit
        t, y = self._simulate(1.8, 0.9, delay=0.0, step=0.5)
        r = fit_step_response(t, y, step=0.5)
        assert not r.at_bounds
        assert r.ok


class TestSafety:
    def _sample(self, t=0.0, pos=(0, 0, 10), vel=(0, 0, 0),
                quat=(1, 0, 0, 0), rates=(0, 0, 0)):
        return OdomSample(t=t, pos=pos, vel=vel, quat=quat, body_rates=rates)

    def test_nominal_hover_safe(self):
        m = SafetyMonitor()
        assert m.check(self._sample(), setpoint=(0, 0, 10)) == []

    def test_tilt_violation(self):
        m = SafetyMonitor()
        # 45 deg roll: q = (cos22.5, sin22.5, 0, 0)
        q = (math.cos(math.pi / 8), math.sin(math.pi / 8), 0, 0)
        assert tilt_from_quat(q) == pytest.approx(math.pi / 4)
        v = m.check(self._sample(quat=q), setpoint=None)
        assert Violation.TILT in v

    def test_position_error_violation(self):
        m = SafetyMonitor(SafetyLimits(max_pos_error=1.0))
        v = m.check(self._sample(pos=(3, 0, 10)), setpoint=(0, 0, 10))
        assert Violation.POS_ERROR in v

    def test_altitude_bounds(self):
        m = SafetyMonitor(SafetyLimits(min_altitude=2.0, max_altitude=20.0))
        assert Violation.ALTITUDE_LOW in m.check(self._sample(pos=(0, 0, 1)), None)
        assert Violation.ALTITUDE_HIGH in m.check(self._sample(pos=(0, 0, 30)), None)

    def test_velocity_violation(self):
        m = SafetyMonitor(SafetyLimits(max_velocity=2.0))
        assert Violation.VELOCITY in m.check(self._sample(vel=(3, 0, 0)), None)

    def test_oscillation_detected(self):
        m = SafetyMonitor(SafetyLimits(osc_rate_rms=0.5, osc_window=2.0))
        v = []
        for i in range(200):
            t = i * 0.02
            w = 3.0 * math.sin(2 * math.pi * 6.0 * t)  # 6 Hz wobble, 3 rad/s
            v = m.check(self._sample(t=t, rates=(w, 0, 0)), None)
        assert Violation.OSCILLATION in v

    def test_no_oscillation_on_smooth_flight(self):
        m = SafetyMonitor(SafetyLimits(osc_rate_rms=0.5))
        v = []
        for i in range(200):
            v = m.check(self._sample(t=i * 0.02, rates=(0.05, 0.02, 0)), None)
        assert Violation.OSCILLATION not in v

    def test_stale_odom(self):
        m = SafetyMonitor(SafetyLimits(odom_timeout=0.3))
        m.check(self._sample(t=0.0), None)
        assert m.check_stale(1.0) == [Violation.ODOM_STALE]
        assert m.check_stale(0.1) == []
