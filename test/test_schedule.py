"""Unit tests for the episode scheduling logic that shortens a session.

These exercise the pure decision functions of TuningConductor (leg
schedule, settle predicate, adaptive episode end, sequential bucket
stop) on a bare instance — no ROS context, no vehicle.
"""

import math

import pytest

from geo_tuner.core.aggregate import EpisodeBucket
from geo_tuner.tuning_conductor import State, TuningConductor


class FakeOdom:
    def __init__(self, pos, vel, yaw=0.0, wz=0.0):
        self.pos = pos
        self.vel = vel
        self.quat = (math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw))
        self.body_rates = (0.0, 0.0, wz)


def make_conductor(**over):
    """A TuningConductor with only the fields the schedule logic uses."""
    c = TuningConductor.__new__(TuningConductor)
    c.bidirectional = True
    c.step_sign = 1.0
    c.leg_offset = 0.0
    c.step_applied = 0.0
    c.axes = ["x"]
    c.axis_idx = 0
    c.setpoint = [0.0, 0.0, 3.0]
    c.setpoint_yaw = 0.0
    c.settle_tol_pos = 0.06
    c.settle_tol_vel = 0.10
    c.settle_tol_yaw = 0.05
    c.settle_quiet_time = 0.4
    c._quiet_t0 = None
    c.episode_quiet_time = 0.5
    c.episode_settle_band = 0.04
    c.episode_settle_floor = 0.0
    c.settle_tol_frac = 0.15
    c.step_size = 0.5
    c.step_size_z = 0.4
    c.yaw_step = 0.5
    c.recording = []
    c.step_t0 = 0.0
    c.rep = 0
    c.min_episodes = 2
    c.episodes_per_rung = 3
    c.early_stop_spread = 1.15
    c.bucket = EpisodeBucket()
    c.odom = FakeOdom((0.0, 0.0, 3.0), (0.0, 0.0, 0.0))
    for k, v in over.items():
        setattr(c, k, v)
    return c


class TestLegSchedule:
    def test_bidirectional_walks_out_and_back(self):
        """0 -> +d -> 0 -> -d -> 0: every leg is a step of size d and the
        excursion never exceeds d from the hover point."""
        c = make_conductor()
        legs = []
        for _ in range(6):
            new = c._next_leg(0.5)
            legs.append(new - c.leg_offset)      # the applied step
            c.leg_offset = new
            if c.leg_offset == 0.0:              # flip after returning
                c.step_sign *= -1.0
        assert legs == [0.5, -0.5, -0.5, 0.5, 0.5, -0.5]
        assert all(abs(s) == pytest.approx(0.5) for s in legs)

    def test_never_walks_away_from_the_hover_point(self):
        """Safety property: the commanded offset is always one step from
        the hover point and returns to it — steps never accumulate, over
        any number of episodes or axis changes."""
        c = make_conductor()
        d = 0.5
        seen_centre = 0
        for n in range(40):
            new = c._next_leg(d)
            assert abs(new) <= d + 1e-12, f"episode {n}: offset {new} > step"
            c.leg_offset = new
            if c.leg_offset == 0.0:
                seen_centre += 1
                c.step_sign *= -1.0
        # back at centre every other episode, and the walk is balanced
        assert seen_centre == 20
        assert c.leg_offset in (0.0, d, -d)

    def test_axis_change_recentres(self):
        """_advance_axis clears the leg schedule, so a new axis always
        starts its steps from the hover point."""
        c = make_conductor(leg_offset=0.5, setpoint_yaw=0.5, rung=0,
                           axes=["x", "y"], wn_ladder=[1.2],
                           bucket=EpisodeBucket(), hover=[0.0, 0.0, 3.0])
        c.state = State.SETTLE
        c.state_t0 = 0.0
        c._goto = lambda st: setattr(c, "state", st)
        c._advance_axis()
        assert c.leg_offset == 0.0
        assert c.setpoint_yaw == 0.0
        assert c.setpoint == [0.0, 0.0, 3.0]

    def test_classic_schedule_always_steps_out(self):
        c = make_conductor(bidirectional=False)
        assert c._next_leg(0.5) == pytest.approx(0.5)
        c.step_sign = -1.0
        assert c._next_leg(0.5) == pytest.approx(-0.5)


class TestSettlePredicate:
    def test_requires_quiet_for_the_full_window(self):
        c = make_conductor()
        assert not c._is_quiet(0.0)      # first quiet sample only starts it
        assert not c._is_quiet(0.3)
        assert c._is_quiet(0.45)

    def test_motion_resets_the_window(self):
        c = make_conductor()
        c._is_quiet(0.0)
        c.odom = FakeOdom((0.0, 0.0, 3.0), (0.5, 0.0, 0.0))   # still moving
        assert not c._is_quiet(0.3)
        c.odom = FakeOdom((0.0, 0.0, 3.0), (0.0, 0.0, 0.0))
        assert not c._is_quiet(0.5)      # window restarted at 0.5
        assert c._is_quiet(0.95)

    def test_position_error_blocks_quiet(self):
        c = make_conductor(odom=FakeOdom((0.4, 0.0, 3.0), (0.0, 0.0, 0.0)))
        assert not c._is_quiet(0.0)
        assert not c._is_quiet(2.0)

    def test_yaw_axis_also_checks_heading(self):
        c = make_conductor(axes=["yaw"], setpoint_yaw=0.5,
                           odom=FakeOdom((0.0, 0.0, 3.0), (0.0, 0.0, 0.0),
                                         yaw=0.0))
        assert not c._is_quiet(1.0)      # heading not there yet
        c.odom = FakeOdom((0.0, 0.0, 3.0), (0.0, 0.0, 0.0), yaw=0.5)
        c._is_quiet(1.0)
        assert c._is_quiet(1.5)


def _record(c, ys, dt=0.02):
    c.recording = [(i * dt, y) for i, y in enumerate(ys)]
    return c.recording[-1][0]


class TestAdaptiveEpisodeEnd:
    def test_settled_response_ends_the_episode(self):
        c = make_conductor(step_applied=0.5)
        t_end = _record(c, [0.5] * 200)
        assert c._response_settled(t_end)

    def test_still_moving_does_not(self):
        c = make_conductor(step_applied=0.5)
        ys = [0.5 + 0.05 * math.sin(4.0 * i * 0.02) for i in range(200)]
        t_end = _record(c, ys)
        assert not c._response_settled(t_end)

    def test_flat_transport_delay_is_not_mistaken_for_settling(self):
        """Right after the step the response is flat at ~0 for the delay;
        that window must not end the episode."""
        c = make_conductor(step_applied=0.5)
        t_end = _record(c, [0.0] * 30)
        assert not c._response_settled(t_end)

    def test_wrong_direction_rejected(self):
        c = make_conductor(step_applied=-0.5)
        t_end = _record(c, [0.5] * 200)
        assert not c._response_settled(t_end)


class TestSequentialStop:
    def test_stops_early_on_tight_agreement(self):
        c = make_conductor(rep=2)
        c.bucket.add(1.20, 0.1)
        c.bucket.add(1.25, 0.1)
        assert c._bucket_settled()

    def test_not_before_the_minimum(self):
        c = make_conductor(rep=1)
        c.bucket.add(1.20, 0.1)
        assert not c._bucket_settled()

    def test_disagreement_flies_the_full_bucket(self):
        c = make_conductor(rep=2)
        c.bucket.add(1.00, 0.1)
        c.bucket.add(1.30, 0.1)     # 1.30x > early_stop_spread 1.15x
        assert not c._bucket_settled()

    def test_a_discarded_episode_blocks_the_early_stop(self):
        """Two flown, one accepted: the identification is already noisy,
        so the bucket must run to full length."""
        c = make_conductor(rep=2)
        c.bucket.add(1.20, 0.1)
        assert not c._bucket_settled()

    def test_early_stop_gate_is_stricter_than_the_update_gate(self):
        """An early stop must never let through evidence that the full
        bucket's consistency gate would reject."""
        c = make_conductor(rep=2, consistency=1.35)
        assert c.early_stop_spread <= 1.35


class TestStepAmplitude:
    """The commanded envelope is hover +/- the configured amplitude, and
    the amplitude is validated against what the safety monitor allows."""

    def test_axis_step_magnitude_follows_the_axis(self):
        c = make_conductor(axes=["x", "z", "yaw"])
        c.axis_idx = 0
        assert c._axis_step_mag() == pytest.approx(0.5)
        c.axis_idx = 1
        assert c._axis_step_mag() == pytest.approx(0.4)
        c.axis_idx = 2
        assert c._axis_step_mag() == pytest.approx(0.5)

    def test_envelope_is_the_amplitude(self):
        """A 0.2 m amplitude confines the manoeuvre to +/-0.2 m."""
        c = make_conductor(step_size=0.2)
        offs = []
        for _ in range(8):
            c.leg_offset = c._next_leg(c._axis_step_mag())
            offs.append(c.leg_offset)
            if c.leg_offset == 0.0:
                c.step_sign *= -1.0
        assert max(abs(o) for o in offs) == pytest.approx(0.2)

    def test_quiet_tolerance_tightens_with_a_small_step(self):
        """A 0.06 m absolute tolerance is 30% of a 0.2 m step; the
        fractional gate must take over."""
        c = make_conductor(step_size=0.2,
                           odom=FakeOdom((0.05, 0.0, 3.0), (0.0, 0.0, 0.0)))
        assert not c._is_quiet(0.0)     # 0.05 m > 0.15 * 0.2 = 0.03 m
        assert not c._is_quiet(1.0)
        c.odom = FakeOdom((0.02, 0.0, 3.0), (0.0, 0.0, 0.0))
        c._is_quiet(1.0)
        assert c._is_quiet(1.5)

    def test_settle_band_has_a_noise_floor(self):
        """A tiny step must not demand a flatness finer than the odometry
        noise, which would silently disable the adaptive stop."""
        c = make_conductor(step_applied=0.1, episode_settle_floor=0.01)
        ys = [0.1 + (0.004 if i % 2 else -0.004) for i in range(200)]
        t_end = _record(c, ys)                 # 8 mm ripple
        assert c._response_settled(t_end)      # under the 10 mm floor
        c.episode_settle_floor = 0.0           # band = 0.04 * 0.1 = 4 mm
        assert not c._response_settled(t_end)


class FakeSafety:
    def __init__(self, max_pos_error=2.0):
        self.limits = type("L", (), {"max_pos_error": max_pos_error})()


def amplitude_validator(max_pos_error=2.0, max_step=2.0):
    c = make_conductor()
    c.safety = FakeSafety(max_pos_error)
    c.max_step_size = max_step
    c.min_step_size = 0.05
    c.max_yaw_step = 1.0
    c.small_step_warn = 0.25
    return c


class TestAmplitudeValidation:
    def test_step_must_stay_under_the_position_error_abort(self):
        """A step commands its full amplitude as instantaneous position
        error, so a step at max_pos_error aborts the session on the spot."""
        c = amplitude_validator(max_pos_error=2.0)
        assert c._step_ceiling() == pytest.approx(1.6)   # 0.8 * 2.0
        assert c._validate_step("step_size", 1.5, False)[0]
        ok, why = c._validate_step("step_size", 1.9, False)
        assert not ok and "max_pos_error" in why

    def test_max_step_size_can_be_the_binding_limit(self):
        c = amplitude_validator(max_pos_error=10.0, max_step=1.0)
        assert c._step_ceiling() == pytest.approx(1.0)
        assert not c._validate_step("step_size", 1.2, False)[0]

    def test_confined_space_amplitudes_are_accepted_with_a_warning(self):
        c = amplitude_validator()
        ok, note = c._validate_step("step_size", 0.15, False)
        assert ok and "nrmse" in note        # allowed, but flagged as small
        ok, note = c._validate_step("step_size", 0.5, False)
        assert ok and note == ""

    def test_degenerate_values_rejected(self):
        c = amplitude_validator()
        assert not c._validate_step("step_size", 0.0, False)[0]
        assert not c._validate_step("step_size", -0.5, False)[0]
        assert not c._validate_step("step_size", float("nan"), False)[0]

    def test_yaw_step_bounds(self):
        c = amplitude_validator()
        assert c._validate_step("yaw_step", 0.5, True)[0]
        assert not c._validate_step("yaw_step", 1.5, True)[0]
        assert not c._validate_step("yaw_step", 0.01, True)[0]
