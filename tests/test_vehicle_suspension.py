import pytest
from gameplay import vehicle_suspension as mod

CASES = [
    ("radius", 'm.wheel_radius(0.6) == approx(0.3)'),
    ("force", 'm.suspension_force(0.2, 1000, 50, 0.1) == approx(205.0)'),
    ("accum", 'm.fixed_step_accumulator(0.05, 0.03, 0.02) == (2, approx(0.01))'),
    ("friction", 'm.friction_circle(6, 8, 5) == approx((3.0, 4.0))'),
    ("transfer", 'm.weight_transfer(1000, 2, 0.5, 1.6) == approx(625.0)'),
    ("gravity", 'm.gravity_at(0) == approx(9.8)'),
    ("reverse", 'm.reverse_wheel_speed(2, True) == approx(-2.0)'),
    ("handbrake", 'm.handbrake_lateral(2, True) == approx(0.6)'),
    ("sleep", 'm.sleep_threshold(0.01, 0.01, 0.1, 0.1) is True'),
    ("contacts", 'm.contact_points([1, 2, 3]) == 3'),
    ("integrate", 'm.deterministic_integrate(2, 4, 0.5) == approx(4.0)'),
    ("grip", 'm.tire_grip(100, 0.5) == approx(50.0)'),
    ("travel", 'm.suspension_travel(0.3, 0.1, 0.5) == approx(0.2)'),
    ("abs", 'm.abs_intervention(6, 4) is True'),
    ("slope", 'm.slope_gravity(9.8, 60) == approx(4.9)')
]

@pytest.mark.parametrize("name,expr", CASES, ids=[c[0] for c in CASES])
def test_contract(name, expr):
    assert eval(expr, {"m": mod, "approx": pytest.approx})
