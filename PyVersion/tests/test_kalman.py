import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import unittest
import numpy as np
import math
from kalman import KalmanFilter


class TestKalmanFilter(unittest.TestCase):
    """Test cases for the KalmanFilter class."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.kalman = KalmanFilter()
        self.initial_P = np.array([[1.0, 0.0], [0.0, 0.1]])
        self.expected_Q = np.array([[0.0025**2, 0.0], [0.0, 0.0001**2]])
    
    def test_kalman_initialization(self):
        """Test KalmanFilter initialization with default values."""
        self.assertEqual(self.kalman.d_0, 1.0)
        self.assertEqual(self.kalman.sigma, 4.0)
        np.testing.assert_array_equal(self.kalman.P, self.initial_P)
        np.testing.assert_array_equal(self.kalman.Q, self.expected_Q)
    
    def test_kalman_initialization_custom_values(self):
        """Test KalmanFilter initialization with custom values."""
        custom_kalman = KalmanFilter(d_0=2.0, sigma=5.0)
        self.assertEqual(custom_kalman.d_0, 2.0)
        self.assertEqual(custom_kalman.sigma, 5.0)
        # P should still be default since it uses default_factory
        np.testing.assert_array_equal(custom_kalman.P, self.initial_P)
    
    def test_independent_instances(self):
        """Test that different KalmanFilter instances have independent P matrices."""
        kalman1 = KalmanFilter()
        kalman2 = KalmanFilter()
        
        # Modify P in first instance
        kalman1.P[0, 0] = 999.0
        
        # Second instance should be unaffected
        self.assertEqual(kalman2.P[0, 0], 1.0)
        self.assertNotEqual(kalman1.P[0, 0], kalman2.P[0, 0])
    
    def test_sequence_step_basic(self):
        """Test basic sequence_step functionality."""
        RSSI0_i = -60.0
        n_i = 2.0
        r_val = -65.0
        d_val = 2.0
        
        RSSI0_j, n_j = self.kalman.sequence_step(RSSI0_i, n_i, r_val, d_val)
        
        # Results should be floats
        self.assertIsInstance(RSSI0_j, float)
        self.assertIsInstance(n_j, float)
        
        # Results should be different from inputs (filter should adjust)
        self.assertNotEqual(RSSI0_j, RSSI0_i)
        self.assertNotEqual(n_j, n_i)
    
    def test_sequence_step_covariance_update(self):
        """Test that P matrix (covariance) is updated correctly."""
        initial_P = self.kalman.P.copy()
        
        self.kalman.sequence_step(-60.0, 2.0, -65.0, 2.0)
        
        # P should be different after update
        self.assertFalse(np.array_equal(self.kalman.P, initial_P))
        
        # P should still be 2x2 positive definite matrix
        self.assertEqual(self.kalman.P.shape, (2, 2))
        # Check positive definiteness by ensuring eigenvalues are positive
        eigenvals = np.linalg.eigvals(self.kalman.P)
        self.assertTrue(np.all(eigenvals > 0))
    
    def test_sequence_step_multiple_updates(self):
        """Test multiple sequential updates."""
        RSSI0 = -60.0
        n = 2.0
        
        # Apply multiple updates
        for i in range(5):
            r_val = -65.0 + i * 0.5  # Gradually changing measurement
            d_val = 2.0 + i * 0.1
            RSSI0, n = self.kalman.sequence_step(RSSI0, n, r_val, d_val)
        
        # Parameters should have evolved
        self.assertNotEqual(RSSI0, -60.0)
        self.assertNotEqual(n, 2.0)
        
        # Should still be reasonable values
        self.assertTrue(-80.0 < RSSI0 < -40.0)  # Reasonable RSSI_0 range
        self.assertTrue(1.0 < n < 4.0)          # Reasonable path loss exponent
    
    def test_sequence_step_perfect_measurement(self):
        """Test with measurement that exactly matches prediction."""
        RSSI0_i = -60.0
        n_i = 2.0
        d_val = 2.0
        
        # Calculate what the predicted measurement should be
        X = -10 * math.log10(d_val / self.kalman.d_0)
        predicted_rssi = RSSI0_i + X * n_i
        
        RSSI0_j, n_j = self.kalman.sequence_step(RSSI0_i, n_i, predicted_rssi, d_val)
        
        # With perfect measurement, changes should be small
        self.assertAlmostEqual(RSSI0_j, RSSI0_i, places=1)
        self.assertAlmostEqual(n_j, n_i, places=2)
    
    def test_sequence_step_extreme_distance(self):
        """Test with extreme distance values."""
        RSSI0_i = -60.0
        n_i = 2.0
        r_val = -80.0
        
        # Very small distance
        RSSI0_j1, n_j1 = self.kalman.sequence_step(RSSI0_i, n_i, r_val, 0.1)
        self.assertIsInstance(RSSI0_j1, float)
        self.assertIsInstance(n_j1, float)
        
        # Reset for next test
        self.kalman.P = self.initial_P.copy()
        
        # Very large distance
        RSSI0_j2, n_j2 = self.kalman.sequence_step(RSSI0_i, n_i, r_val, 100.0)
        self.assertIsInstance(RSSI0_j2, float)
        self.assertIsInstance(n_j2, float)
    
    def test_sequence_step_extreme_rssi(self):
        """Test with extreme RSSI values."""
        RSSI0_i = -60.0
        n_i = 2.0
        d_val = 2.0
        
        # Very strong signal
        RSSI0_j1, n_j1 = self.kalman.sequence_step(RSSI0_i, n_i, -30.0, d_val)
        self.assertIsInstance(RSSI0_j1, float)
        self.assertIsInstance(n_j1, float)
        
        # Reset for next test
        self.kalman.P = self.initial_P.copy()
        
        # Very weak signal
        RSSI0_j2, n_j2 = self.kalman.sequence_step(RSSI0_i, n_i, -100.0, d_val)
        self.assertIsInstance(RSSI0_j2, float)
        self.assertIsInstance(n_j2, float)
    
    def test_convergence_behavior(self):
        """Test that filter converges to true values with consistent measurements."""
        true_RSSI0 = -61.5
        true_n = 2.3
        true_distance = 3.0
        
        # Start with wrong initial guess
        RSSI0 = -59.0
        n = 2.0
        
        # Set random seed for reproducible test
        np.random.seed(42)
        
        # Generate measurements from true model with small noise
        for _ in range(30):  # More iterations for better convergence
            X = -10 * math.log10(true_distance / self.kalman.d_0)
            true_measurement = true_RSSI0 + X * true_n
            noisy_measurement = true_measurement + np.random.normal(0, 0.3)  # Less noise
            
            RSSI0, n = self.kalman.sequence_step(RSSI0, n, noisy_measurement, true_distance)
        
        # Should converge reasonably close to true values (relaxed tolerance)
        self.assertAlmostEqual(RSSI0, true_RSSI0, delta=3.0)  # Within 3 dB
        self.assertAlmostEqual(n, true_n, delta=0.5)          # Within 0.5
        
        # Verify parameters are in reasonable range
        self.assertTrue(-70.0 < RSSI0 < -50.0)
        self.assertTrue(1.5 < n < 3.5)
    
    def test_q_matrix_immutability(self):
        """Test that Q matrix remains constant across instances."""
        kalman1 = KalmanFilter()
        kalman2 = KalmanFilter()
        
        # Q should be the same for all instances
        np.testing.assert_array_equal(kalman1.Q, kalman2.Q)
        np.testing.assert_array_equal(kalman1.Q, self.expected_Q)
        
        # Q should be the class variable, not instance variable
        self.assertIs(kalman1.Q, kalman2.Q)  # Same object reference
    
    def test_mathematical_consistency(self):
        """Test mathematical consistency of Kalman filter equations."""
        RSSI0_i = -60.0
        n_i = 2.0
        r_val = -65.0
        d_val = 2.0
        
        # Store initial state
        P_initial = self.kalman.P.copy()
        
        # Manual calculation to verify
        x_ji = np.array([RSSI0_i, n_i])
        P_predict = P_initial + self.kalman.Q
        
        X = -10 * math.log10(d_val / self.kalman.d_0)
        H = np.array([1.0, X]).reshape(1, 2)
        
        r_predict = (H @ x_ji)[0]
        residual = r_val - r_predict
        
        S = (H @ P_predict @ H.T)[0, 0] + self.kalman.sigma**2
        K = P_predict @ H.T / S
        
        x_jj_expected = x_ji + K.flatten() * residual
        P_jj_expected = (np.eye(2) - K @ H) @ P_predict
        
        # Run the actual method
        RSSI0_j, n_j = self.kalman.sequence_step(RSSI0_i, n_i, r_val, d_val)
        
        # Compare results
        np.testing.assert_array_almost_equal([RSSI0_j, n_j], x_jj_expected, decimal=10)
        np.testing.assert_array_almost_equal(self.kalman.P, P_jj_expected, decimal=10)


if __name__ == '__main__':
    unittest.main()
    