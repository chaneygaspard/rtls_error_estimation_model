from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, List
from utils import PointR3
from kalman import KalmanFilter
import math
import time

#Object classes
@dataclass(slots = True)
class Anchor:
    macadress: str
    coord: PointR3
    ewma: float = field(default=1.0,  repr=False)   # start "healthy"
    last_seen: float = field(default=0.0, repr=False)  # epoch time
    RSSI_0: float = field(default=-59.0)
    n: float = field(default=2.0)
    adaptive_mode: bool = field(default=True, repr=False)
    kalman: KalmanFilter = field(default_factory=KalmanFilter, repr=False)
    message_count: int = field(default=0, repr=False)  # tracks how many messages this anchor has processed

    def update_health(self, z: float, now: float, LAMBDA: float = 0.05) -> None:
        self.ewma = LAMBDA * z * z + (1 - LAMBDA) * self.ewma
        self.last_seen = now
    
    def update_parameters(self, measured_rssi: float, estimated_distance: float) -> None:
        """Update RSSI_0 and n using Kalman filter if in adaptive mode."""
        if self.adaptive_mode:
            self.RSSI_0, self.n = self.kalman.sequence_step(
                self.RSSI_0, self.n, measured_rssi, estimated_distance
            )
        self.message_count += 1  # Increment message counter each time anchor is updated

    def is_warning(self) -> bool:
        return (4 <= self.ewma < 8)

    def is_faulty(self) -> bool:
        return (self.ewma >= 8)

@dataclass(slots = True)
class Tag:
    macadress: str
    est_coord: PointR3
    rssi_dict: Dict[str, float]

    #returns rssi val given anchor_id
    def rssi_for_anchor(self, anchor_id: str) -> float:
        return self.rssi_dict[anchor_id]
    
    #returns list of  anchors in communication with the tag
    def anchors_included(self) -> List[str]:
        return list(self.rssi_dict.keys())

#Model classes
@dataclass(slots = True)
class PathLossModel:
    d_0: float = 1.0
    sigma: float = 4.0 #de base: 4.0

    #generate mu as a function of estimated distance from a coordinate
    def mu(self, RSSI_0: float, n: float, est_dist: float):
        safe_dist = max(est_dist, 1e-6)  # Prevent log(0)
        return (RSSI_0 - (10 * n * math.log10(safe_dist / self.d_0)))
    
    #generate z as a function of rssi freq from coord and estimated distance
    def z(self, rssi_freq: float, RSSI_0: float, n: float, est_dist:float): 
        mu = self.mu(RSSI_0, n, est_dist)
        return ((rssi_freq - mu) / self.sigma)
