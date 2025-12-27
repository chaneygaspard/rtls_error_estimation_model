from __future__ import annotations
from dataclasses import dataclass
from functools import cached_property
from typing import Dict, List
import math, time, random   

from models import Anchor, Tag, PathLossModel
from utils  import _R3_distance, _logpdf_student_t, _cep95_from_conf

EWMA_THRESHOLD: float = 8.0

def update_anchors_from_tag_data(
    anchors: List[Anchor], 
    tag: Tag, 
    model: PathLossModel, 
    now: float, 
    deltaR: float = 7.0, #de base: 10.0
    T_vis: int = 6000
) -> None:
    """
    Updates the shared state of all anchors based on data from a single tag.
    This includes both path-loss parameters (RSSI0, n) and health (ewma).
    """
    temp_system = TagSystem(tag, model)

    # --- Parameter updates (from old update_anchor_parameters) ---
    for anchor in temp_system._get_significant_anchors(anchors):
        if anchor.macadress in tag.rssi_dict:
            rssi = tag.rssi_for_anchor(anchor.macadress)
            dist = _R3_distance(anchor.coord, tag.est_coord)
            anchor.update_parameters(rssi, dist)

    # --- Health updates (from old update_anchor_healths) ---
    if not tag.rssi_dict:
        return

    # To calculate z-values for health, we need a temporary system
    temp_system = TagSystem(tag, model)
    z_dict = temp_system.z_vals(anchors) # Pass anchors in

    max_rssi = max(tag.rssi_dict.values())
    anchor_map = {a.macadress: a for a in anchors}

    for anchor_id, z_val in z_dict.items():
        anchor = anchor_map[anchor_id] 
        anch_rssi = tag.rssi_for_anchor(anchor_id)
        delta_rssi = max_rssi - anch_rssi
        time_since = now - anchor.last_seen if anchor.last_seen else 0.0

        # Gate checks
        if delta_rssi > deltaR: 
            continue
        if time_since > T_vis:
            continue 

        anchor.update_health(z_val, now)

@dataclass(slots=False)
class TagSystem:
    tag: Tag
    model: PathLossModel

    def _get_significant_anchors(self, anchors: List[Anchor], max_n: int = 5) -> List[Anchor]:
        rssi_dict = self.tag.rssi_dict
        if not rssi_dict:
            return []
        max_rssi = max(rssi_dict.values())
        keep = [
            a for a in anchors
            if a.macadress in rssi_dict and rssi_dict[a.macadress] >= max_rssi - 10 and a.ewma < EWMA_THRESHOLD
        ]
        keep.sort(key=lambda a: rssi_dict[a.macadress], reverse=True)
        return keep[:max_n]

    def distances(self, anchors: List[Anchor]) -> Dict[str, float]:
        return {
            anchor.macadress: _R3_distance(anchor.coord, self.tag.est_coord)
            for anchor in self._get_significant_anchors(anchors)
        }   
    
    def z_vals(self, anchors: List[Anchor]) -> Dict[str, float]:
        # Only use significant anchors
        significant_anchors = self._get_significant_anchors(anchors)
        dist_dict = {
            anchor.macadress: _R3_distance(anchor.coord, self.tag.est_coord)
            for anchor in significant_anchors
        }
        z_values = {}
        anchor_map = {a.macadress: a for a in anchors}
        for anch_id, est_dist in dist_dict.items():
            anchor = anchor_map[anch_id]
            rssi_value = self.tag.rssi_for_anchor(anch_id)
            z_values[anch_id] = self.model.z(rssi_value, anchor.RSSI_0, anchor.n, est_dist)
        return z_values
    
    def confidence_score(self, anchors: List[Anchor], v: int = 5, scale: float = 2) -> float:
        z_dict = self.z_vals(anchors)
        if not z_dict:
            return 0.0        
        sig = sum(_logpdf_student_t(z, v) for _, z in z_dict.items())
        l = sig / len(z_dict)
        return math.exp(l / scale)

    def error_radius(self, anchors: List[Anchor]) -> float:
        P_val = self.confidence_score(anchors)
        return _cep95_from_conf(P_val)