"""Off-road vehicle suspension / tire helpers.

The functions in this module are the pure-math core of the vehicle model.
They are intentionally free of engine state so the same code can run on the
client and the server and produce bit-identical results (see
:func:`deterministic_integrate`).
"""

import math

# Gravity at the reference altitude (sea level), in m/s^2.
GRAVITY_SEA_LEVEL = 9.8
# Mean Earth radius used for the inverse-square gravity falloff, in metres.
EARTH_RADIUS = 6_371_000.0

# Fixed-point scale used by :func:`deterministic_integrate`.  Keeping the
# integration in integers makes the result independent of the host CPU's
# floating-point rounding mode / register width.
FIXED_POINT_SCALE = 10 ** 9

# How much of the lateral friction survives when the handbrake is engaged.
HANDBRAKE_LATERAL_FACTOR = 0.3


def wheel_radius(diameter):
    """Tyre radius from its diameter.

    The suspension ray must be cast from ``hardpoint - radius * normal`` (and
    extended by the radius) so the contact patch, not the hardpoint, is what
    touches the step.  This is the radius used for that offset.
    """
    if diameter < 0:
        raise ValueError("wheel diameter must be non-negative")
    return diameter / 2.0


def suspension_force(compression, stiffness, damping, velocity):
    """Spring + damper force along the suspension axis.

    ``compression`` is the positive compression of the spring (metres),
    ``velocity`` is the positive compression rate (m/s).  The spring cannot
    pull the wheel down, so it only contributes while compressed; the damper
    is symmetric and always contributes.

    Forces are always computed from the *fixed* physics step, never from a
    variable frame delta, which is what keeps the damping stable at low FPS.
    """
    spring = compression * stiffness if compression > 0.0 else 0.0
    return spring + damping * velocity


def fixed_step_accumulator(accum, dt, step):
    """Consume accumulated time in fixed-size physics steps.

    Returns ``(steps, remainder)``.

    ``accum`` is the total unspent time (already including this frame's
    delta), ``dt`` is the real frame delta and ``step`` is the fixed physics
    timestep.  The number of sub-steps is capped at the number of steps that
    actually fit in ``dt`` (rounded up, at least one).  This clamps catch-up
    work on a slow frame instead of running an unbounded number of steps,
    which is what caused the "chase the clock" spikes.

    Any leftover backlog beyond that cap is discarded rather than carried
    forever: the per-frame capacity is one frame of real time, so a backlog
    could never drain and would instead add permanent simulation lag.
    """
    if step <= 0:
        raise ValueError("fixed step must be positive")
    if accum < 0:
        accum = 0.0
    max_steps = max(1, int(math.ceil(dt / step)))
    steps = min(int(accum / step), max_steps)
    remainder = accum - steps * step
    # Drop excess backlog and guard tiny negative remainders from rounding.
    remainder = math.fmod(remainder, step)
    if remainder < 0.0:
        remainder += step
    return steps, remainder


def friction_circle(long_force, lat_force, limit):
    """Clamp a tire force pair to the friction circle of radius ``limit``.

    The traction budget is shared between longitudinal and lateral force:
    a demand outside the circle is scaled back along its own direction
    instead of being allowed to keep accelerating past the grip limit.
    """
    if limit <= 0.0:
        return 0.0, 0.0
    magnitude = math.hypot(long_force, lat_force)
    if magnitude <= limit:
        return float(long_force), float(lat_force)
    scale = limit / magnitude
    return long_force * scale, lat_force * scale


def weight_transfer(mass, accel, height, track):
    """Load transferred across the track under lateral acceleration.

    ``mass`` is the *total* vehicle mass (sprung + unsprung), because the
    unsprung wheels also generate load transfer.  ``height`` is the centre of
    gravity height and ``track`` the wheel track.  Returns the load moved from
    the inside wheels to the outside wheels, in newtons.
    """
    if track <= 0:
        raise ValueError("track must be positive")
    return mass * accel * height / track


def gravity_at(altitude):
    """Gravity sampled from the vehicle's altitude.

    Treating gravity as a constant made the client/server prediction drift
    apart on long slopes; sampling it by position keeps both sides on the
    same value and follows the inverse-square falloff.
    """
    r = EARTH_RADIUS + altitude
    if r <= 0:
        raise ValueError("altitude below the centre of the Earth")
    return GRAVITY_SEA_LEVEL * (EARTH_RADIUS / r) ** 2


def reverse_wheel_speed(speed, reverse):
    """Wheel speed with the direction negated while reversing.

    ABS and traction control read wheel speed as a signed value; without the
    sign flip they intervene in the wrong direction in reverse.
    """
    return -speed if reverse else speed


def handbrake_lateral(lateral, engaged):
    """Lateral friction available while the handbrake is pulled.

    The handbrake does more than lock the rear wheel speed: it also drops the
    lateral grip so the rear can slide.  ``lateral`` is the available lateral
    friction before the handbrake, the result is the reduced value.
    """
    if engaged:
        return lateral * HANDBRAKE_LATERAL_FACTOR
    return lateral


def sleep_threshold(linear, angular, lin_limit, ang_limit,
                    accel=0.0, accel_limit=None):
    """True when the body may go to sleep.

    A body that is still creeping on a slope has near-zero linear velocity but
    keeps micro-jittering; sleep must also require the angular velocity (and,
    when supplied, the linear acceleration) to be under its limit.
    """
    if abs(linear) > lin_limit or abs(angular) > ang_limit:
        return False
    if accel_limit is not None and abs(accel) > accel_limit:
        return False
    return True


def contact_points(chassis):
    """Number of chassis contact points used by collision resolution.

    Treating the chassis as one convex hull made a light scrape on a rock
    launch the whole car.  Collision is solved per contact point instead, so
    this returns how many points the chassis provides.
    """
    if chassis is None:
        return 0
    return len(chassis)


def deterministic_integrate(value, rate, dt):
    """Integrate ``value`` by ``rate * dt`` using fixed-point arithmetic.

    Everything is scaled to integers before the multiply so the same inputs
    give the same output on every platform, regardless of floating-point
    precision differences in the host.
    """
    scale = FIXED_POINT_SCALE
    q_value = int(round(value * scale))
    q_rate = int(round(rate * scale))
    q_dt = int(round(dt * scale))
    q_delta = _trunc_div(q_rate * q_dt, scale)
    return (q_value + q_delta) / scale


def _trunc_div(numerator, denominator):
    """Integer division truncating toward zero (Python's ``//`` floors)."""
    quotient = abs(numerator) // denominator
    return -quotient if numerator < 0 else quotient


def tire_grip(load, mu):
    """Maximum friction force for a tire carrying ``load`` with coefficient ``mu``."""
    if load < 0:
        raise ValueError("tire load must be non-negative")
    return load * mu


def suspension_travel(current, rest, max_travel):
    """Suspension travel away from rest, clamped to the strut's maximum.

    ``current`` and ``rest`` are suspension lengths; travel is measured as
    ``current - rest`` (positive when the strut is extended) and limited to
    ``max_travel`` in either direction.
    """
    travel = current - rest
    if travel > max_travel:
        return max_travel
    if travel < -max_travel:
        return -max_travel
    return travel


def abs_intervention(wheel_speed, target):
    """True when ABS should intervene: the wheel is spinning faster than target."""
    return wheel_speed > target


def slope_gravity(gravity, slope_deg):
    """Component of gravity normal to a slope.

    On a slope the normal load is ``m * g * cos(theta)``; using the full
    constant gravity here made predictions on ramps drift away from the
    server's simulation.
    """
    return gravity * math.cos(math.radians(slope_deg))
