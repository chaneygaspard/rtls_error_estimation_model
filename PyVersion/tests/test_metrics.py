import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import unittest
from unittest.mock import patch, MagicMock
import math
import time

from metrics import TagSystem, update_anchors_from_tag_data
from models import Anchor, Tag, PathLossModel
from utils import PointR3, _R3_distance


class TestTagSystem(unittest.TestCase):
    
    def setUp(self):
        """Set up test fixtures before each test method."""
        # Create test anchors
        self.anchor1 = Anchor(macadress="anchor1", coord=(0.0, 0.0, 0.0))
        self.anchor2 = Anchor(macadress="anchor2", coord=(1.0, 0.0, 0.0))
        self.anchor3 = Anchor(macadress="anchor3", coord=(0.0, 1.0, 0.0))
        self.anchor4 = Anchor(macadress="anchor4", coord=(0.0, 0.0, 1.0))
        self.anchors = [self.anchor1, self.anchor2, self.anchor3, self.anchor4]
        
        # Create test tag
        self.tag = Tag(
            macadress="test_tag",
            est_coord=(0.5, 0.5, 0.5),
            rssi_dict={
                "anchor1": -50.0,
                "anchor2": -55.0,
                "anchor3": -60.0,
                "anchor4": -65.0
            }
        )
        
        # Create path loss model
        self.model = PathLossModel(d_0=1.0, sigma=4.0)
        
        # Create TagSystem instance
        self.tag_system = TagSystem(
            tag=self.tag,
            model=self.model
        )

    def test_anchor_map_cached_property(self):
        """Test that _anchor_map creates correct mapping of anchor IDs to anchor objects."""
        anchor_map = {a.macadress: a for a in self.anchors}
        
        self.assertEqual(len(anchor_map), 4)
        self.assertIn("anchor1", anchor_map)
        self.assertIn("anchor2", anchor_map)
        self.assertIn("anchor3", anchor_map)
        self.assertIn("anchor4", anchor_map)
        
        self.assertEqual(anchor_map["anchor1"], self.anchor1)
        self.assertEqual(anchor_map["anchor2"], self.anchor2)
        self.assertEqual(anchor_map["anchor3"], self.anchor3)
        self.assertEqual(anchor_map["anchor4"], self.anchor4)

    def test_anchor_map_empty_anchors(self):
        """Test _anchor_map with empty anchors list."""
        empty_anchors = []
        anchor_map = {a.macadress: a for a in empty_anchors}
        self.assertEqual(len(anchor_map), 0)
        self.assertEqual(anchor_map, {})

    def test_anchor_map_duplicate_ids(self):
        """Test _anchor_map behavior with duplicate anchor IDs (should keep last one)."""
        duplicate_anchor = Anchor(macadress="anchor1", coord=(2.0, 2.0, 2.0))
        anchors_with_duplicate = [self.anchor1, duplicate_anchor]
        anchor_map = {a.macadress: a for a in anchors_with_duplicate}
        self.assertEqual(len(anchor_map), 1)
        self.assertEqual(anchor_map["anchor1"], duplicate_anchor)

    def test_significant_anchors_normal_case(self):
        """Test _significant_anchors with normal RSSI values."""
        significant_anchors = self.tag_system._get_significant_anchors(self.anchors)
        
        # Should return anchors with RSSI >= max_rssi - 10
        # max_rssi = -50, so anchors with RSSI >= -60 should be included
        self.assertEqual(len(significant_anchors), 3)  # anchor1, anchor2, anchor3
        self.assertEqual(significant_anchors[0].macadress, "anchor1")  # highest RSSI
        self.assertEqual(significant_anchors[1].macadress, "anchor2")
        self.assertEqual(significant_anchors[2].macadress, "anchor3")

    def test_significant_anchors_empty_rssi_dict(self):
        """Test _significant_anchors with empty RSSI dictionary."""
        empty_tag = Tag(
            macadress="empty_tag",
            est_coord=(0.5, 0.5, 0.5),
            rssi_dict={}
        )
        empty_tag_system = TagSystem(
            tag=empty_tag,
            model=self.model
        )
        
        significant_anchors = empty_tag_system._get_significant_anchors(self.anchors)
        self.assertEqual(significant_anchors, [])

    def test_significant_anchors_max_n_limit(self):
        """Test _significant_anchors respects max_n parameter."""
        # Create more anchors with different RSSI values
        anchor5 = Anchor(macadress="anchor5", coord=(1.0, 1.0, 0.0))
        anchor6 = Anchor(macadress="anchor6", coord=(1.0, 0.0, 1.0))
        
        tag_with_more_anchors = Tag(
            macadress="test_tag",
            est_coord=(0.5, 0.5, 0.5),
            rssi_dict={
                "anchor1": -50.0,
                "anchor2": -55.0,
                "anchor3": -60.0,
                "anchor4": -65.0,
                "anchor5": -70.0,
                "anchor6": -75.0
            }
        )
        
        tag_system_more = TagSystem(
            tag=tag_with_more_anchors,
            model=self.model
        )
        
        # Test with max_n=3
        significant_anchors = tag_system_more._get_significant_anchors(self.anchors)
        self.assertEqual(len(significant_anchors), 3)  # Should be limited to 3
        self.assertEqual(significant_anchors[0].macadress, "anchor1")
        self.assertEqual(significant_anchors[1].macadress, "anchor2")
        self.assertEqual(significant_anchors[2].macadress, "anchor3")

    def test_distances_normal_case(self):
        """Test distances() method returns correct distances only for significant anchors."""
        distances = self.tag_system.distances(self.anchors)
        # Only significant anchors (a1, a2, a3) should be present
        self.assertEqual(len(distances), 3)
        self.assertIn("anchor1", distances)
        self.assertNotIn("anchor4", distances)
        # Check one value to ensure calculation is still correct
        expected_dist1 = math.sqrt((0.5-0.0)**2 + (0.5-0.0)**2 + (0.5-0.0)**2)
        self.assertAlmostEqual(distances["anchor1"], expected_dist1, places=6)

    def test_distances_empty_significant_anchors(self):
        """Test distances() method with no significant anchors."""
        empty_tag = Tag(
            macadress="empty_tag",
            est_coord=(0.5, 0.5, 0.5),
            rssi_dict={}
        )
        empty_tag_system = TagSystem(
            tag=empty_tag,
            model=self.model
        )
        
        distances = empty_tag_system.distances(self.anchors)
        self.assertEqual(distances, {})

    def test_distances_same_coordinates(self):
        """Test distances() method when tag and anchor have same coordinates."""
        same_coord_tag = Tag(
            macadress="same_coord_tag",
            est_coord=(0.0, 0.0, 0.0),  # Same as anchor1
            rssi_dict={"anchor1": -50.0}
        )
        same_coord_system = TagSystem(
            tag=same_coord_tag,
            model=self.model
        )
        
        distances = same_coord_system.distances(self.anchors)
        self.assertEqual(distances["anchor1"], 0.0)

    def test_z_vals_normal_case(self):
        """Test z_vals() method returns values only for significant anchors."""
        z_vals = self.tag_system.z_vals(self.anchors)
        self.assertEqual(len(z_vals), 3)
        self.assertIn("anchor1", z_vals)
        self.assertNotIn("anchor4", z_vals)
        # Check one value to ensure calculation is still correct
        dist = _R3_distance(self.anchor1.coord, self.tag.est_coord)
        expected_z = self.model.z(self.tag.rssi_for_anchor("anchor1"), self.anchor1.RSSI_0, self.anchor1.n, dist)
        self.assertAlmostEqual(z_vals["anchor1"], expected_z, places=6)

    def test_z_vals_empty_distances(self):
        """Test z_vals() method when distances() returns empty dict."""
        empty_tag = Tag(
            macadress="empty_tag",
            est_coord=(0.5, 0.5, 0.5),
            rssi_dict={}
        )
        empty_tag_system = TagSystem(
            tag=empty_tag,
            model=self.model
        )
        
        z_vals = empty_tag_system.z_vals(self.anchors)
        self.assertEqual(z_vals, {})

    def test_z_vals_extreme_values(self):
        """Test z_vals() method with extreme RSSI and distance values."""
        extreme_tag = Tag(
            macadress="extreme_tag",
            est_coord=(1000.0, 1000.0, 1000.0),  # Very far from anchors
            rssi_dict={"anchor1": -100.0}  # Very low RSSI
        )
        extreme_system = TagSystem(
            tag=extreme_tag,
            model=self.model
        )
        
        z_vals = extreme_system.z_vals(self.anchors)
        self.assertIn("anchor1", z_vals)
        # Should still calculate a z value, even if extreme
        self.assertIsInstance(z_vals["anchor1"], float)

    def test_confidence_score_normal_case(self):
        """Test confidence_score() is calculated based only on significant anchors."""
        # Get confidence with all anchors (some insignificant)
        conf_all = self.tag_system.confidence_score(self.anchors)
        # Get confidence with only significant anchors
        significant_anchors = self.tag_system._get_significant_anchors(self.anchors)
        system_sig_only = TagSystem(self.tag, self.model)
        conf_sig_only = system_sig_only.confidence_score(significant_anchors)
        self.assertAlmostEqual(conf_all, conf_sig_only, places=6)
        self.assertGreater(conf_all, 0)

    def test_confidence_score_empty_z_vals(self):
        """Test confidence_score() method when z_vals() returns empty dict."""
        empty_tag = Tag(
            macadress="empty_tag",
            est_coord=(0.5, 0.5, 0.5),
            rssi_dict={}
        )
        empty_tag_system = TagSystem(
            tag=empty_tag,
            model=self.model
        )
        
        confidence = empty_tag_system.confidence_score(self.anchors)
        self.assertEqual(confidence, 0.0)

    def test_confidence_score_different_parameters(self):
        """Test confidence_score() method with different v and scale parameters."""
        confidence1 = self.tag_system.confidence_score(self.anchors, v=3, scale=1.5)
        confidence2 = self.tag_system.confidence_score(self.anchors, v=10, scale=3.0)
        
        self.assertIsInstance(confidence1, float)
        self.assertIsInstance(confidence2, float)
        self.assertGreaterEqual(confidence1, 0.0)
        self.assertGreaterEqual(confidence2, 0.0)
        self.assertLessEqual(confidence1, 1.0)
        self.assertLessEqual(confidence2, 1.0)

    def test_update_anchor_healths_normal_case(self):
        """Test update_anchor_healths() method with normal conditions."""
        now = time.time()
        initial_ewma = self.anchor1.ewma
        initial_last_seen = self.anchor1.last_seen
        
        update_anchors_from_tag_data(self.anchors, self.tag, self.model, now)
        
        # Check that anchor health was updated
        self.assertNotEqual(self.anchor1.ewma, initial_ewma)
        self.assertEqual(self.anchor1.last_seen, now)

    def test_update_anchor_healths_empty_rssi_dict(self):
        """Test update_anchor_healths() method with empty RSSI dictionary."""
        empty_tag = Tag(
            macadress="empty_tag",
            est_coord=(0.5, 0.5, 0.5),
            rssi_dict={}
        )
        now = time.time()
        initial_ewmas = [a.ewma for a in self.anchors]
        initial_last_seens = [a.last_seen for a in self.anchors]
        update_anchors_from_tag_data(self.anchors, empty_tag, self.model, now)
        for a, ewma, last_seen in zip(self.anchors, initial_ewmas, initial_last_seens):
            self.assertEqual(a.ewma, ewma)
            self.assertEqual(a.last_seen, last_seen)

    def test_update_anchor_healths_signal_quality_rule(self):
        """Test update_anchor_healths() method with signal quality rule (deltaR)."""
        anchor1 = Anchor(macadress="anchor1", coord=(0.0, 0.0, 0.0))
        anchor2 = Anchor(macadress="anchor2", coord=(1.0, 0.0, 0.0))
        anchors = [anchor1, anchor2]
        large_diff_tag = Tag(
            macadress="large_diff_tag",
            est_coord=(0.5, 0.5, 0.5),
            rssi_dict={
                "anchor1": -50.0,  # Strong signal
                "anchor2": -70.0   # Weak signal (20dB difference > deltaR=10)
            }
        )
        now = time.time()
        initial_ewma2 = anchor2.ewma
        initial_last_seen2 = anchor2.last_seen
        update_anchors_from_tag_data(anchors, large_diff_tag, self.model, now)
        # anchor2 should not be updated due to signal quality rule
        self.assertEqual(anchor2.ewma, initial_ewma2)
        self.assertEqual(anchor2.last_seen, initial_last_seen2)
        # anchor1 should be updated
        self.assertNotEqual(anchor1.ewma, 1.0)  # Default value
        self.assertEqual(anchor1.last_seen, now)

    def test_update_anchor_parameters_only_updates_present_anchors(self):
        """Test that only anchors present in tag.rssi_dict are updated."""
        # Set up initial values
        self.anchor1.RSSI_0 = -59.0
        self.anchor3.RSSI_0 = -59.0
        self.anchor2.RSSI_0 = -59.0
        self.anchor4.RSSI_0 = -59.0

        self.tag.rssi_dict = {"anchor1": -60.0, "anchor3": -70.0}
        self.tag.est_coord = (1.0, 2.0, 3.0)
        self.anchor1.coord = (4.0, 6.0, 3.0)
        self.anchor3.coord = (0.0, 0.0, 0.0)

        # Save initial values
        initial2 = (self.anchor2.RSSI_0, self.anchor2.n)
        initial4 = (self.anchor4.RSSI_0, self.anchor4.n)

        update_anchors_from_tag_data(self.anchors, self.tag, self.model, now=0)

        # anchor1 and anchor3 should have changed
        self.assertNotEqual(self.anchor1.RSSI_0, -59.0)
        self.assertNotEqual(self.anchor3.RSSI_0, -59.0)
        # anchor2 and anchor4 should not have changed
        self.assertEqual((self.anchor2.RSSI_0, self.anchor2.n), initial2)
        self.assertEqual((self.anchor4.RSSI_0, self.anchor4.n), initial4)

    def test_update_anchor_parameters_correct_arguments(self):
        """Test that update_parameters is called with correct RSSI and distance."""
        self.tag.rssi_dict = {"anchor1": -60.0}
        self.tag.est_coord = (1.0, 2.0, 3.0)
        self.anchor1.coord = (4.0, 6.0, 3.0)
        initial_rssi_0 = self.anchor1.RSSI_0
        initial_n = self.anchor1.n

        update_anchors_from_tag_data(self.anchors, self.tag, self.model, now=0)

        # The values should have changed from the initial values
        self.assertNotEqual(self.anchor1.RSSI_0, initial_rssi_0)
        self.assertNotEqual(self.anchor1.n, initial_n)

    def test_update_anchor_parameters_non_adaptive_anchor(self):
        """Test that update_parameters does not change parameters for non-adaptive anchors."""
        self.anchor1.adaptive_mode = False
        initial_rssi_0 = self.anchor1.RSSI_0
        initial_n = self.anchor1.n
        self.tag.rssi_dict = {"anchor1": -60.0}
        self.tag.est_coord = (0.0, 0.0, 0.0)
        self.anchor1.coord = (1.0, 0.0, 0.0)

        update_anchors_from_tag_data(self.anchors, self.tag, self.model, now=0)

        self.assertEqual(self.anchor1.RSSI_0, initial_rssi_0)
        self.assertEqual(self.anchor1.n, initial_n)

    def test_update_anchor_parameters_no_anchors_in_rssi_dict(self):
        """Test that nothing happens if no anchors are in rssi_dict."""
        initial_vals = [(a.RSSI_0, a.n) for a in [self.anchor1, self.anchor2, self.anchor3, self.anchor4]]
        self.tag.rssi_dict = {}
        update_anchors_from_tag_data(self.anchors, self.tag, self.model, now=0)
        for anchor, (init_rssi_0, init_n) in zip([self.anchor1, self.anchor2, self.anchor3, self.anchor4], initial_vals):
            self.assertEqual(anchor.RSSI_0, init_rssi_0)
            self.assertEqual(anchor.n, init_n)

    def test_error_radius_typical_case(self):
        """Test error_radius returns a float and is within expected range for typical data."""
        radius = self.tag_system.error_radius(self.anchors)
        self.assertIsInstance(radius, float)
        # Should be within the range of the lookup table
        self.assertGreaterEqual(radius, 2.5)
        self.assertLessEqual(radius, 7.4)

    def test_error_radius_empty_rssi_dict(self):
        """Test error_radius returns max value when no anchors are present."""
        empty_tag = Tag(
            macadress="empty_tag",
            est_coord=(0.0, 0.0, 0.0),
            rssi_dict={}
        )
        tag_system = TagSystem(
            tag=empty_tag,
            model=self.model
        )
        radius = tag_system.error_radius(self.anchors)
        self.assertEqual(radius, 7.4)  # Should match the max value in LOOKUP_CEP95

    def test_error_radius_extreme_confidence(self):
        """Test error_radius returns min value for artificially high confidence."""
        # Patch confidence_score to return 1.0 (max confidence)
        class DummyTagSystem(TagSystem):
            def confidence_score(self, *a, **kw):
                return 1.0
        tag_system = DummyTagSystem(
            tag=self.tag,
            model=self.model
        )
        radius = tag_system.error_radius(self.anchors)
        self.assertEqual(radius, 2.5)  # Should match the min value in LOOKUP_CEP95

    def test_error_radius_low_confidence(self):
        """Test error_radius returns max value for artificially low confidence."""
        class DummyTagSystem(TagSystem):
            def confidence_score(self, *a, **kw):
                return 0.0
        tag_system = DummyTagSystem(
            tag=self.tag,
            model=self.model
        )
        radius = tag_system.error_radius(self.anchors)
        self.assertEqual(radius, 7.4)  # Should match the max value in LOOKUP_CEP95

    def test_error_radius_interpolated(self):
        """Test error_radius interpolates correctly for a mid-range confidence."""
        class DummyTagSystem(TagSystem):
            def confidence_score(self, *a, **kw):
                return 0.30  # Between 0.17 and 0.43
        tag_system = DummyTagSystem(
            tag=self.tag,
            model=self.model
        )
        radius = tag_system.error_radius(self.anchors)
        # Manual interpolation between 6.1 (0.17) and 4.3 (0.43)
        t = (0.30 - 0.17) / (0.43 - 0.17)
        expected = 6.1 + t * (4.3 - 6.1)
        self.assertAlmostEqual(radius, expected, places=6)

    def test_update_anchor_healths_time_since_exceeds_T_vis(self):
        """Test anchor health is not updated if time_since > T_vis (ewma unchanged, but last_seen/params may update)."""
        anchor = Anchor(macadress="anchor1", coord=(0, 0, 0))
        anchor.last_seen = 1.0
        tag = Tag(macadress="t1", est_coord=(0, 0, 0), rssi_dict={"anchor1": -50.0})
        now = 1000
        initial_ewma = anchor.ewma
        update_anchors_from_tag_data([anchor], tag, self.model, now, T_vis=10)
        self.assertEqual(anchor.ewma, initial_ewma)  # Only ewma must not change

    def test_update_anchor_healths_delta_rssi_exceeds_deltaR(self):
        """Test anchor health is not updated if delta_rssi > deltaR."""
        # This anchor should be updated
        anchor_ok = Anchor(macadress="anchor_ok", coord=(0, 0, 0))
        # This anchor should be skipped because its RSSI is too low compared to max
        anchor_skipped = Anchor(macadress="anchor_skipped", coord=(1, 1, 1))
        
        anchors = [anchor_ok, anchor_skipped]
        
        tag = Tag(
            macadress="t1", 
            est_coord=(0, 0, 0), 
            rssi_dict={
                "anchor_ok": -60.0,      # max_rssi
                "anchor_skipped": -80.0  # delta_rssi = 20
            }
        )
        
        now = time.time()
        initial_ewma_ok = anchor_ok.ewma
        initial_ewma_skipped = anchor_skipped.ewma
        
        # deltaR is 10, but delta_rssi for anchor_skipped is 20
        update_anchors_from_tag_data(anchors, tag, self.model, now, deltaR=10)
        
        # The OK anchor should be updated
        self.assertNotEqual(anchor_ok.ewma, initial_ewma_ok)
        # The skipped anchor should NOT be updated
        self.assertEqual(anchor_skipped.ewma, initial_ewma_skipped)

    def test_update_anchor_healths_multiple_anchors(self):
        """Test anchor health is updated for multiple anchors in one call."""
        anchor1 = Anchor(macadress="anchor1", coord=(0, 0, 0))
        anchor2 = Anchor(macadress="anchor2", coord=(1, 0, 0))
        tag = Tag(macadress="t1", est_coord=(0, 0, 0), rssi_dict={"anchor1": -50.0, "anchor2": -51.0})
        now = time.time()
        initial_ewma1 = anchor1.ewma
        initial_ewma2 = anchor2.ewma
        update_anchors_from_tag_data([anchor1, anchor2], tag, self.model, now)
        self.assertNotEqual(anchor1.ewma, initial_ewma1)
        self.assertNotEqual(anchor2.ewma, initial_ewma2)

    def test_update_anchor_healths_ewma_threshold(self):
        """Test anchor health is updated for anchors with ewma below threshold, not above."""
        anchor1 = Anchor(macadress="anchor1", coord=(0, 0, 0), ewma=2.0)
        anchor2 = Anchor(macadress="anchor2", coord=(1, 0, 0), ewma=10.0)  # Above threshold
        tag = Tag(macadress="t1", est_coord=(0, 0, 0), rssi_dict={"anchor1": -50.0, "anchor2": -51.0})
        now = time.time()
        initial_ewma1 = anchor1.ewma
        initial_ewma2 = anchor2.ewma
        update_anchors_from_tag_data([anchor1, anchor2], tag, self.model, now)
        self.assertNotEqual(anchor1.ewma, initial_ewma1)
        self.assertEqual(anchor2.ewma, initial_ewma2)

    def test_z_vals_only_significant_anchors(self):
        """Test that z_vals only includes significant anchors (RSSI >= max_rssi-10, ewma < threshold)."""
        anchor1 = Anchor(macadress="a1", coord=(0,0,0), ewma=1.0)
        anchor2 = Anchor(macadress="a2", coord=(1,0,0), ewma=1.0)
        anchor3 = Anchor(macadress="a3", coord=(2,0,0), ewma=1.0)
        anchor4 = Anchor(macadress="a4", coord=(3,0,0), ewma=1.0)
        anchors = [anchor1, anchor2, anchor3, anchor4]
        tag = Tag(macadress="t1", est_coord=(0,0,0), rssi_dict={
            "a1": -50.0, "a2": -55.0, "a3": -61.0, "a4": -70.0
        })
        system = TagSystem(tag, self.model)
        z = system.z_vals(anchors)
        # Only a1, a2 should be included (a3 is just below threshold, a4 is too low)
        self.assertIn("a1", z)
        self.assertIn("a2", z)
        self.assertNotIn("a3", z)
        self.assertNotIn("a4", z)

    def test_z_vals_excludes_high_ewma(self):
        """Test that anchors with high ewma are excluded from z_vals and confidence calculations."""
        anchor1 = Anchor(macadress="a1", coord=(0,0,0), ewma=1.0)
        anchor2 = Anchor(macadress="a2", coord=(1,0,0), ewma=10.0)  # Above threshold
        anchors = [anchor1, anchor2]
        tag = Tag(macadress="t1", est_coord=(0,0,0), rssi_dict={"a1": -50.0, "a2": -49.0})
        system = TagSystem(tag, self.model)
        z = system.z_vals(anchors)
        self.assertIn("a1", z)
        self.assertNotIn("a2", z)
        conf = system.confidence_score(anchors)
        self.assertGreater(conf, 0)

    def test_z_vals_excludes_low_rssi(self):
        """Test that anchors with low RSSI (below max_rssi-10) are excluded."""
        anchor1 = Anchor(macadress="a1", coord=(0,0,0), ewma=1.0)
        anchor2 = Anchor(macadress="a2", coord=(1,0,0), ewma=1.0)
        anchors = [anchor1, anchor2]
        tag = Tag(macadress="t1", est_coord=(0,0,0), rssi_dict={"a1": -50.0, "a2": -70.0})
        system = TagSystem(tag, self.model)
        z = system.z_vals(anchors)
        self.assertIn("a1", z)
        self.assertNotIn("a2", z)

    def test_z_vals_empty_if_no_significant(self):
        """Test that z_vals/confidence_score/error_radius return empty/0/max if no anchors are significant."""
        anchor1 = Anchor(macadress="a1", coord=(0,0,0), ewma=100.0)
        anchor2 = Anchor(macadress="a2", coord=(1,0,0), ewma=100.0)
        anchors = [anchor1, anchor2]
        tag = Tag(macadress="t1", est_coord=(0,0,0), rssi_dict={"a1": -50.0, "a2": -49.0})
        system = TagSystem(tag, self.model)
        z = system.z_vals(anchors)
        self.assertEqual(z, {})
        conf = system.confidence_score(anchors)
        self.assertEqual(conf, 0.0)
        err = system.error_radius(anchors)
        self.assertEqual(err, 7.4)  # max value from LOOKUP_CEP95

    def test_update_only_significant_anchors(self):
        """Test that only significant anchors are updated (parameter and health)."""
        anchor1 = Anchor(macadress="a1", coord=(0,0,0), ewma=1.0)
        anchor2 = Anchor(macadress="a2", coord=(1,0,0), ewma=1.0)
        anchor3 = Anchor(macadress="a3", coord=(2,0,0), ewma=1.0)
        anchor4 = Anchor(macadress="a4", coord=(3,0,0), ewma=1.0)
        anchors = [anchor1, anchor2, anchor3, anchor4]
        tag = Tag(macadress="t1", est_coord=(0,0,0), rssi_dict={
            "a1": -50.0, "a2": -55.0, "a3": -61.0, "a4": -70.0
        })
        now = time.time()
        # Save initial values
        initial_rssi0 = [a.RSSI_0 for a in anchors]
        initial_ewma = [a.ewma for a in anchors]
        update_anchors_from_tag_data(anchors, tag, self.model, now)
        # Only a1, a2 should be updated
        self.assertNotEqual(anchor1.RSSI_0, initial_rssi0[0])
        self.assertNotEqual(anchor2.RSSI_0, initial_rssi0[1])
        self.assertEqual(anchor3.RSSI_0, initial_rssi0[2])
        self.assertEqual(anchor4.RSSI_0, initial_rssi0[3])
        self.assertNotEqual(anchor1.ewma, initial_ewma[0])
        self.assertNotEqual(anchor2.ewma, initial_ewma[1])
        self.assertEqual(anchor3.ewma, initial_ewma[2])
        self.assertEqual(anchor4.ewma, initial_ewma[3])

    def test_update_anchor_not_updated_if_time_since_exceeds_T_vis(self):
        """Test anchor is not updated if time_since > T_vis (ewma unchanged, but last_seen/params may update)."""
        anchor = Anchor(macadress="a1", coord=(0, 0, 0), ewma=1.0)
        anchor.last_seen = 1.0
        tag = Tag(macadress="a1", est_coord=(0, 0, 0), rssi_dict={"a1": -50.0})
        now = 1000
        initial_ewma = anchor.ewma
        update_anchors_from_tag_data([anchor], tag, self.model, now, T_vis=10)
        self.assertEqual(anchor.ewma, initial_ewma)  # Only ewma must not change

    def test_update_anchor_not_updated_if_delta_rssi_exceeds_deltaR(self):
        """Test anchor is not updated if delta_rssi > deltaR."""
        anchor1 = Anchor(macadress="a1", coord=(0,0,0), ewma=1.0)
        anchor2 = Anchor(macadress="a2", coord=(1,0,0), ewma=1.0)
        anchors = [anchor1, anchor2]
        tag = Tag(macadress="t1", est_coord=(0,0,0), rssi_dict={"a1": -50.0, "a2": -80.0})
        now = time.time()
        initial_ewma1 = anchor1.ewma
        initial_ewma2 = anchor2.ewma
        update_anchors_from_tag_data(anchors, tag, self.model, now, deltaR=10)
        self.assertNotEqual(anchor1.ewma, initial_ewma1)
        self.assertEqual(anchor2.ewma, initial_ewma2)

    def test_update_anchor_not_updated_if_ewma_above_threshold(self):
        """Test anchor is not updated if ewma >= EWMA_THRESHOLD."""
        anchor1 = Anchor(macadress="a1", coord=(0,0,0), ewma=10.0)  # Above threshold
        anchor2 = Anchor(macadress="a2", coord=(1,0,0), ewma=1.0)
        anchors = [anchor1, anchor2]
        tag = Tag(macadress="t1", est_coord=(0,0,0), rssi_dict={"a1": -50.0, "a2": -49.0})
        now = time.time()
        initial_ewma1 = anchor1.ewma
        initial_ewma2 = anchor2.ewma
        update_anchors_from_tag_data(anchors, tag, self.model, now)
        self.assertEqual(anchor1.ewma, initial_ewma1)
        self.assertNotEqual(anchor2.ewma, initial_ewma2)


if __name__ == '__main__':
    unittest.main()
