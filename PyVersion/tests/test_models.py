import unittest
import math
import sys
import os
import time

# Add the parent directory to the path to import models
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from models import Anchor, Tag, PathLossModel


class TestAnchor(unittest.TestCase):
    """Test cases for the Anchor class."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.anchor = Anchor(macadress="anchor_001", coord=(1.0, 2.0, 3.0))
    
    def test_anchor_initialization(self):
        """Test Anchor initialization with default values."""
        self.assertEqual(self.anchor.macadress, "anchor_001")
        self.assertEqual(self.anchor.coord, (1.0, 2.0, 3.0))
        self.assertEqual(self.anchor.ewma, 1.0)
        self.assertEqual(self.anchor.last_seen, 0.0)
    
    def test_anchor_initialization_custom_values(self):
        """Test Anchor initialization with custom values."""
        anchor = Anchor(macadress="anchor_002", coord=(5.0, 6.0, 7.0), ewma=2.5, last_seen=1234567890.0)
        self.assertEqual(anchor.macadress, "anchor_002")
        self.assertEqual(anchor.coord, (5.0, 6.0, 7.0))
        self.assertEqual(anchor.ewma, 2.5)
        self.assertEqual(anchor.last_seen, 1234567890.0)
    
    def test_update_health_basic(self):
        """Test basic health update functionality."""
        now = time.time()
        z = 2.0
        LAMBDA = 0.05
        
        old_ewma = self.anchor.ewma
        self.anchor.update_health(z, now, LAMBDA)
        
        expected_ewma = LAMBDA * z * z + (1 - LAMBDA) * old_ewma
        self.assertAlmostEqual(self.anchor.ewma, expected_ewma)
        self.assertEqual(self.anchor.last_seen, now)
    
    def test_update_health_default_lambda(self):
        """Test health update with default LAMBDA value."""
        now = time.time()
        z = 1.5
        
        old_ewma = self.anchor.ewma
        self.anchor.update_health(z, now)
        
        expected_ewma = 0.05 * z * z + 0.95 * old_ewma
        self.assertAlmostEqual(self.anchor.ewma, expected_ewma)
        self.assertEqual(self.anchor.last_seen, now)
    
    def test_update_health_zero_z(self):
        """Test health update with z=0."""
        now = time.time()
        z = 0.0
        LAMBDA = 0.1
        
        old_ewma = self.anchor.ewma
        self.anchor.update_health(z, now, LAMBDA)
        
        expected_ewma = LAMBDA * z * z + (1 - LAMBDA) * old_ewma
        self.assertAlmostEqual(self.anchor.ewma, expected_ewma)
        self.assertEqual(self.anchor.last_seen, now)
    
    def test_update_health_multiple_updates(self):
        """Test multiple sequential health updates."""
        now = 1000.0
        z_values = [0.5, 1.2, 0.8, 2.0, 0.3]
        
        for z in z_values:
            self.anchor.update_health(z, now, LAMBDA=0.1)
            now += 1.0
        
        # EWMA should reflect the sequence of updates
        self.assertNotEqual(self.anchor.ewma, 1.0)  # Should have changed from default
        self.assertEqual(self.anchor.last_seen, now - 1.0)  # Should be last update time
    
    def test_anchor_adaptive_mode_default(self):
        """Test that anchor adaptive_mode defaults to True."""
        anchor = Anchor(macadress="test_adaptive", coord=(0.0, 0.0, 0.0))
        self.assertTrue(anchor.adaptive_mode)
        self.assertIsNotNone(anchor.kalman)
    
    def test_anchor_adaptive_mode_disabled(self):
        """Test anchor with adaptive_mode explicitly disabled."""
        anchor = Anchor(macadress="test_fixed", coord=(0.0, 0.0, 0.0), adaptive_mode=False)
        self.assertFalse(anchor.adaptive_mode)
        self.assertIsNotNone(anchor.kalman)  # Kalman instance still exists
    
    def test_update_parameters_adaptive_mode_enabled(self):
        """Test that update_parameters works when adaptive_mode is True."""
        anchor = Anchor(macadress="test_adaptive", coord=(0.0, 0.0, 0.0), adaptive_mode=True)
        
        initial_rssi_0 = anchor.RSSI_0
        initial_n = anchor.n
        
        # Apply parameter update
        anchor.update_parameters(measured_rssi=-65.0, estimated_distance=2.5)
        
        # Parameters should have changed
        self.assertNotEqual(anchor.RSSI_0, initial_rssi_0)
        self.assertNotEqual(anchor.n, initial_n)
        
        # Values should still be reasonable
        self.assertTrue(-80.0 < anchor.RSSI_0 < -40.0)
        self.assertTrue(1.0 < anchor.n < 4.0)
    
    def test_update_parameters_adaptive_mode_disabled(self):
        """Test that update_parameters does nothing when adaptive_mode is False."""
        anchor = Anchor(macadress="test_fixed", coord=(0.0, 0.0, 0.0), adaptive_mode=False)
        
        initial_rssi_0 = anchor.RSSI_0
        initial_n = anchor.n
        
        # Apply parameter update - should do nothing
        anchor.update_parameters(measured_rssi=-65.0, estimated_distance=2.5)
        
        # Parameters should remain unchanged
        self.assertEqual(anchor.RSSI_0, initial_rssi_0)
        self.assertEqual(anchor.n, initial_n)
    
    def test_update_parameters_multiple_calls(self):
        """Test multiple parameter updates on adaptive anchor."""
        anchor = Anchor(macadress="test_multi", coord=(0.0, 0.0, 0.0), adaptive_mode=True)
        
        initial_rssi_0 = anchor.RSSI_0
        initial_n = anchor.n
        
        # Apply multiple updates with different measurements
        measurements = [
            (-64.0, 2.0),
            (-66.5, 2.5),
            (-63.2, 1.8),
            (-67.1, 3.0)
        ]
        
        for rssi, distance in measurements:
            anchor.update_parameters(measured_rssi=rssi, estimated_distance=distance)
        
        # Parameters should have evolved from initial values
        self.assertNotEqual(anchor.RSSI_0, initial_rssi_0)
        self.assertNotEqual(anchor.n, initial_n)
        
        # Values should still be reasonable after multiple updates
        self.assertTrue(-80.0 < anchor.RSSI_0 < -40.0)
        self.assertTrue(1.0 < anchor.n < 4.0)
    
    def test_independent_kalman_instances(self):
        """Test that different anchors have independent Kalman filter instances."""
        anchor1 = Anchor(macadress="anchor1", coord=(0.0, 0.0, 0.0), adaptive_mode=True)
        anchor2 = Anchor(macadress="anchor2", coord=(1.0, 1.0, 1.0), adaptive_mode=True)
        
        # Should have different Kalman instances
        self.assertIsNot(anchor1.kalman, anchor2.kalman)
        
        # Apply update to first anchor only
        anchor1.update_parameters(measured_rssi=-65.0, estimated_distance=2.0)
        
        # Second anchor's Kalman state should be unaffected
        import numpy as np
        initial_P = np.array([[1.0, 0.0], [0.0, 0.1]])
        np.testing.assert_array_equal(anchor2.kalman.P, initial_P)


class TestTag(unittest.TestCase):
    """Test cases for the Tag class."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.tag = Tag(macadress="tag_001", est_coord=(10.0, 20.0, 30.0), rssi_dict={
            "anchor_001": -45.0,
            "anchor_002": -52.0,
            "anchor_003": -48.0
        })
    
    def test_tag_initialization(self):
        """Test Tag initialization."""
        self.assertEqual(self.tag.macadress, "tag_001")
        self.assertEqual(self.tag.est_coord, (10.0, 20.0, 30.0))
        self.assertEqual(self.tag.rssi_dict, {
            "anchor_001": -45.0,
            "anchor_002": -52.0,
            "anchor_003": -48.0
        })
    
    def test_rssi_for_anchor_existing(self):
        """Test getting RSSI for existing anchor."""
        rssi = self.tag.rssi_for_anchor("anchor_001")
        self.assertEqual(rssi, -45.0)
    
    def test_rssi_for_anchor_all_anchors(self):
        """Test getting RSSI for all anchors."""
        expected_rssi = {
            "anchor_001": -45.0,
            "anchor_002": -52.0,
            "anchor_003": -48.0
        }
        
        for anchor_id, expected in expected_rssi.items():
            rssi = self.tag.rssi_for_anchor(anchor_id)
            self.assertEqual(rssi, expected)
    
    def test_rssi_for_anchor_key_error(self):
        """Test that KeyError is raised for non-existent anchor."""
        with self.assertRaises(KeyError):
            self.tag.rssi_for_anchor("non_existent_anchor")
    
    def test_anchors_included(self):
        """Test getting list of included anchors."""
        anchors = self.tag.anchors_included()
        expected = ["anchor_001", "anchor_002", "anchor_003"]
        
        # Order doesn't matter, so we sort both lists
        self.assertEqual(sorted(anchors), sorted(expected))
    
    def test_anchors_included_empty_dict(self):
        """Test anchors_included with empty RSSI dictionary."""
        tag = Tag(macadress="tag_002", est_coord=(0.0, 0.0, 0.0), rssi_dict={})
        anchors = tag.anchors_included()
        self.assertEqual(anchors, [])
    
    def test_anchors_included_single_anchor(self):
        """Test anchors_included with single anchor."""
        tag = Tag(macadress="tag_003", est_coord=(5.0, 5.0, 5.0), rssi_dict={"anchor_single": -50.0})
        anchors = tag.anchors_included()
        self.assertEqual(anchors, ["anchor_single"])


class TestPathLossModel(unittest.TestCase):
    """Test cases for the PathLossModel class."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.model = PathLossModel(
            d_0=1.0,
            sigma=4.0
        )
        # Test parameters
        self.RSSI_0 = -59.0
        self.n = 2.0
    
    def test_pathloss_initialization(self):
        """Test PathLossModel initialization."""
        self.assertEqual(self.model.d_0, 1.0)
        self.assertEqual(self.model.sigma, 4.0)
    
    def test_pathloss_initialization_defaults(self):
        """Test PathLossModel initialization with default values."""
        model = PathLossModel()
        self.assertEqual(model.d_0, 1.0)  # default
        self.assertEqual(model.sigma, 4.0)  # default
    
    def test_mu_at_reference_distance(self):
        """Test mu calculation at reference distance d_0."""
        est_dist = 1.0  # equal to d_0
        mu = self.model.mu(self.RSSI_0, self.n, est_dist)
        expected = self.RSSI_0 - (10 * self.n * math.log10(est_dist / self.model.d_0))
        self.assertAlmostEqual(mu, expected)
        self.assertAlmostEqual(mu, self.RSSI_0)  # Should equal RSSI_0 at d_0
    
    def test_mu_at_different_distances(self):
        """Test mu calculation at different distances."""
        distances = [0.5, 1.0, 2.0, 5.0, 10.0]
        
        for est_dist in distances:
            mu = self.model.mu(self.RSSI_0, self.n, est_dist)
            expected = self.RSSI_0 - (10 * self.n * math.log10(est_dist / self.model.d_0))
            self.assertAlmostEqual(mu, expected)
    
    def test_mu_decreasing_with_distance(self):
        """Test that mu decreases as distance increases."""
        distances = [1.0, 2.0, 5.0, 10.0, 20.0]
        mu_values = [self.model.mu(self.RSSI_0, self.n, d) for d in distances]
        
        # mu should decrease as distance increases
        for i in range(1, len(mu_values)):
            self.assertGreater(mu_values[i-1], mu_values[i])
    
    def test_z_calculation_basic(self):
        """Test basic z calculation."""
        rssi_freq = -50.0
        est_dist = 2.0
        
        z = self.model.z(rssi_freq, self.RSSI_0, self.n, est_dist)
        
        mu = self.model.mu(self.RSSI_0, self.n, est_dist)
        expected_z = (rssi_freq - mu) / self.model.sigma
        self.assertAlmostEqual(z, expected_z)
    
    def test_z_calculation_zero_distance(self):
        """Test z calculation with zero distance (edge case)."""
        rssi_freq = -45.0
        est_dist = 0.0
        
        # This should handle the log(0) case gracefully or raise an error
        try:
            z = self.model.z(rssi_freq, self.RSSI_0, self.n, est_dist)
            # If it doesn't raise an error, check the result
            self.assertIsInstance(z, float)
        except (ValueError, ZeroDivisionError):
            # Expected behavior for log(0)
            pass
    
    def test_z_calculation_negative_distance(self):
        """Test z calculation with negative distance (invalid case)."""
        rssi_freq = -45.0
        est_dist = -1.0
        
        # This should handle negative distance gracefully or raise an error
        try:
            z = self.model.z(rssi_freq, self.RSSI_0, self.n, est_dist)
            # If it doesn't raise an error, check the result
            self.assertIsInstance(z, float)
        except (ValueError, ZeroDivisionError):
            # Expected behavior for log of negative number
            pass
    
    def test_z_calculation_different_sigma(self):
        """Test z calculation with different sigma values."""
        model = PathLossModel(d_0=1.0, sigma=2.0)
        rssi_freq = -50.0
        est_dist = 3.0
        
        z = model.z(rssi_freq, self.RSSI_0, self.n, est_dist)
        mu = model.mu(self.RSSI_0, self.n, est_dist)
        expected_z = (rssi_freq - mu) / model.sigma
        self.assertAlmostEqual(z, expected_z)
    
    def test_z_calculation_different_path_loss_exponent(self):
        """Test z calculation with different path loss exponent n."""
        model = PathLossModel(d_0=1.0, sigma=4.0)
        rssi_freq = -50.0
        est_dist = 2.0
        n_test = 3.0
        
        z = model.z(rssi_freq, self.RSSI_0, n_test, est_dist)
        mu = model.mu(self.RSSI_0, n_test, est_dist)
        expected_z = (rssi_freq - mu) / model.sigma
        self.assertAlmostEqual(z, expected_z)
    
    def test_mu_and_z_consistency(self):
        """Test that mu and z calculations are consistent."""
        rssi_freq = -55.0
        est_dist = 4.0
        
        mu = self.model.mu(self.RSSI_0, self.n, est_dist)
        z = self.model.z(rssi_freq, self.RSSI_0, self.n, est_dist)
        
        # Verify the relationship: z = (rssi_freq - mu) / sigma
        expected_z = (rssi_freq - mu) / self.model.sigma
        self.assertAlmostEqual(z, expected_z)
    
    def test_pathloss_model_with_different_reference_distance(self):
        """Test PathLossModel with different reference distance d_0."""
        model = PathLossModel(d_0=2.0, sigma=4.0)
        est_dist = 4.0
        
        mu = model.mu(self.RSSI_0, self.n, est_dist)
        expected = self.RSSI_0 - (10 * self.n * math.log10(est_dist / model.d_0))
        self.assertAlmostEqual(mu, expected)


if __name__ == '__main__':
    unittest.main()
