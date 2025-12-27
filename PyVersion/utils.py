from __future__ import annotations
from typing import Tuple
import math

#data types:
PointR3 = Tuple[float, float, float] #(x,y,z) coord 

def _R3_distance(a: PointR3, b: PointR3) -> float:
    """
    Calculate the Euclidean distance between two 3D points.
    
    Args:
        a: First 3D point as (x, y, z) coordinates
        b: Second 3D point as (x, y, z) coordinates
        
    Returns:
        float: Euclidean distance between points a and b
    """
    v_0 = (a[0] - b[0])**2
    v_1 = (a[1] - b[1])**2
    v_2 = (a[2] - b[2])**2
    return math.sqrt(v_0 + v_1 + v_2)

#student-t distribution helper:
def _logpdf_student_t(z: float, v: int = 5) -> float:
    return (
        math.lgamma((v + 1) / 2)                # Γ((v+1)/2)
        - math.lgamma(v / 2)                    # Γ(v/2)
        - 0.5 * math.log(v * math.pi)           # (v π)½
        - (v + 1) / 2 * math.log1p(z * z / v)   # (1 + z²/v)^{(v+1)/2}
    )


#Derive 95% confidence radius from P confidence value
LOOKUP_CEP95 = [(0.05, 7.4), (0.17, 6.1), (0.43, 4.3), (0.80, 2.5)]

def _cep95_from_conf(p_conf: float, table=LOOKUP_CEP95) -> float:
    xs, ys = zip(*table)
    if p_conf <= xs[0]:
        return ys[0]
    if p_conf >= xs[-1]:
        return ys[-1]
    i = next(j for j, x in enumerate(xs) if x > p_conf) - 1
    t = (p_conf - xs[i]) / (xs[i+1] - xs[i])
    return ys[i] + t * (ys[i+1] - ys[i])