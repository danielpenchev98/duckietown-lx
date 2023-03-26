from typing import Tuple

import numpy as np


def PIDController(
    v_0: float, y_ref: float, y_hat: float, prev_e_y: float, prev_int_y: float, delta_t: float
) -> Tuple[float, float, float, float]:
    """
    PID performing lateral control.

    Args:
        v_0:        linear Duckiebot speed (constant).
        y_ref:      target y coordinate.
        y_hat:      the current estimated y.
        prev_e_y:   tracking error at previous iteration.
        prev_int_y: previous integral error term.
        delta_t:    time interval since last call.

    Returns:
        v_0:        linear velocity of the Duckiebot
        omega:      angular velocity of the Duckiebot
        e:          current tracking error (automatically becomes prev_e_y at next iteration).
        e_int:      current integral error (automatically becomes prev_int_y at next iteration).
    """
    

    k_prop = 1.5
    k_int = 0.0
    k_d = 7.0

    e = y_ref - y_hat

    e_d = (e - prev_e_y)/delta_t
    e_prop = e
    e_int = prev_int_y + e*delta_t

    e_int = max(min(e_int,2),-2)

    omega = k_prop*e_prop + k_int*e_int + k_d*e_d

    print(f"y_ref {y_ref}, y_hat {y_hat}, omega {omega}")


    # clamped_omega = max(min(omega, 5),-5)
    # is_saturated = clamped_omega != omega
    # is_making_saturation_worse = omega * e > 0
    # if is_saturated and is_making_saturation_worse:
    #     e_int = prev_int_y
    
    return v_0, omega, e, e_int
