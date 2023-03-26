from typing import Tuple

import numpy as np



def PIDController(
        v_0: float,
        theta_ref: float,
        theta_hat: float,
        prev_e: float,
        prev_int: float,
        delta_t: float
) -> Tuple[float, float, float, float]:
    """
    PID performing heading control.
    Args:
        v_0:        linear Duckiebot speed (given).
        theta_ref:  reference heading pose.
        theta_hat:  the current estiamted theta.
        prev_e:     tracking error at previous iteration.
        prev_int:   previous integral error term.
        delta_t:    time interval since last call.
    Returns:
        v_0:     linear velocity of the Duckiebot
        omega:   angular velocity of the Duckiebot
        e:       current tracking error (automatically becomes prev_e_y at next iteration).
        e_int:   current integral error (automatically becomes prev_int_y at next iteration).
    """
    
    k_prop = 5.0
    k_int = 0.2
    k_der = 0.1
    
    e = theta_ref - theta_hat
    e_int = prev_int + e * delta_t
    e_prop =  e
    e_der =  (e - prev_e)/delta_t

    omega = k_prop * e_prop + k_der * e_der + k_int * e_int

    #e_int = max(min(e_int,2),-2)

    clamped_omega = max(min(omega, 5),-5)

    is_saturated = omega != clamped_omega
    is_making_saturation_worse = omega * e > 0 # if omega is saturated, then error will make omega even mopre saturated (even more negative/positive)

    if is_saturated and is_making_saturation_worse: # anti-windup
       e_int = prev_int

    #print(f"\n\nDelta time : {delta_t} \nE : {np.rad2deg(e)} \nE int : {e_int} \nPrev e : {prev_e} \nOmega : {omega} \nTheta hat: {np.rad2deg(theta_hat)} \n")
    return v_0,clamped_omega, e, e_int
