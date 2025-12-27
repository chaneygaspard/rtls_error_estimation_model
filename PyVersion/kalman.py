import numpy as np
import math
from typing import Optional, Tuple
from dataclasses import dataclass, field

@dataclass(slots=True)
class KalmanFilter:

    Q = np.array([[0.0025**2, 0.0], [0.0, 0.0001**2]])
    P: np.ndarray = field(default_factory=lambda: np.array([[1.0, 0.0], [0.0, 0.1]]))
    d_0: float = 1.0
    sigma: float = 4.0

    #given r_i, d_i, (RSSI_0, n)_{i|i}, returns (RSSI_0, n)_{i+1|i+1}
    def sequence_step(self, RSSI0_i: float, n_i: float, r_val: float, d_val: float) -> Tuple[float, float]:

        #note: x_ji designates x{i+1|i},  x{i+1|i+1} is designated by x_jj
        x_ji = np.array([RSSI0_i, n_i])

        #P{i+1|i} = P{i|i} + Q
        self.P += self.Q

        #vect H = [1 X] in R^{1*2}
        safe_d_val = max(d_val, 1e-6) # Prevent log(0)
        X = (-10)*math.log10(safe_d_val / self.d_0)
        H = np.array([1.0, X]).reshape(1, 2)

        #predicted r_val & residual
        r_predict = np.dot(H, x_ji)[0]
        resid = r_val - r_predict

        #S & K matrices
        S = (H @ self.P @ H.T)[0, 0] + self.sigma**2
        K = self.P @ H.T / S 

        #x{i+1|i+1} & P{i+1|i+1}
        x_jj = x_ji + (K.flatten() * resid)
        self.P = (np.eye(2) - (K @ H)) @ self.P

        #return (RSSI_0, n)_{i+1|i+1}
        RSSI0_j, n_j = x_jj
        return (RSSI0_j, n_j)