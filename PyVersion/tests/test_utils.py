import unittest
import math
import sys
import os

# Add the parent directory to the path to import utils
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from utils import _R3_distance, _logpdf_student_t, _cep95_from_conf


class TestR3Distance(unittest.TestCase):
    """Test cases for the _R3_distance function."""
    
    def test_identical_points(self):
        """Test distance between identical points should be 0."""
        point = (1.0, 2.0, 3.0)
        self.assertEqual(_R3_distance(point, point), 0.0)
    
    def test_basic_distance(self):
        """Test basic distance calculation."""
        point1 = (0.0, 0.0, 0.0)
        point2 = (3.0, 4.0, 0.0)
        expected = 5.0  # sqrt(3² + 4² + 0²) = sqrt(9 + 16) = 5
        self.assertEqual(_R3_distance(point1, point2), expected)
    
    def test_negative_coordinates(self):
        """Test distance with negative coordinates."""
        point1 = (-1.0, -2.0, -3.0)
        point2 = (1.0, 2.0, 3.0)
        expected = math.sqrt(4 + 16 + 36)  # sqrt(56)
        self.assertAlmostEqual(_R3_distance(point1, point2), expected)
    
    def test_commutative_property(self):
        """Test that distance is commutative (a to b equals b to a)."""
        point1 = (1.0, 2.0, 3.0)
        point2 = (4.0, 5.0, 6.0)
        distance1 = _R3_distance(point1, point2)
        distance2 = _R3_distance(point2, point1)
        self.assertEqual(distance1, distance2)


class TestLogPdfStudentT(unittest.TestCase):
    """Test cases for the _logpdf_student_t function."""
    
    def test_zero_input_default_df(self):
        """Test logpdf at z=0 with default degrees of freedom (v=5)."""
        result = _logpdf_student_t(0.0)
        expected = math.lgamma(3) - math.lgamma(2.5) - 0.5 * math.log(5 * math.pi)
        self.assertAlmostEqual(result, expected)
    
    def test_zero_input_custom_df(self):
        """Test logpdf at z=0 with custom degrees of freedom."""
        result = _logpdf_student_t(0.0, v=3)
        expected = math.lgamma(2) - math.lgamma(1.5) - 0.5 * math.log(3 * math.pi)
        self.assertAlmostEqual(result, expected)
    
    def test_positive_input(self):
        """Test logpdf with positive z values."""
        z = 1.5
        v = 5
        result = _logpdf_student_t(z, v)
        expected = (
            math.lgamma((v + 1) / 2)
            - math.lgamma(v / 2)
            - 0.5 * math.log(v * math.pi)
            - (v + 1) / 2 * math.log1p(z * z / v)
        )
        self.assertAlmostEqual(result, expected)
    
    def test_negative_input(self):
        """Test logpdf with negative z values (should be symmetric)."""
        z_positive = 2.0
        z_negative = -2.0
        v = 5
        
        result_positive = _logpdf_student_t(z_positive, v)
        result_negative = _logpdf_student_t(z_negative, v)
        
        # Logpdf should be symmetric around 0
        self.assertAlmostEqual(result_positive, result_negative)
    
    def test_large_input(self):
        """Test logpdf with large z values."""
        z = 10.0
        v = 5
        result = _logpdf_student_t(z, v)
        # Should be a finite negative number (very small probability)
        self.assertIsInstance(result, float)
        self.assertLess(result, 0)
        self.assertTrue(math.isfinite(result))
    
    def test_different_degrees_of_freedom(self):
        """Test logpdf with different degrees of freedom values."""
        z = 1.0
        for v in [1, 3, 5, 10, 30]:
            result = _logpdf_student_t(z, v)
            self.assertIsInstance(result, float)
            self.assertTrue(math.isfinite(result))
    
    def test_symmetry_property(self):
        """Test that the function is symmetric around z=0."""
        v = 5
        for z in [0.5, 1.0, 2.0, 5.0]:
            result_pos = _logpdf_student_t(z, v)
            result_neg = _logpdf_student_t(-z, v)
            self.assertAlmostEqual(result_pos, result_neg, places=10)
    
    def test_monotonicity(self):
        """Test that logpdf decreases as |z| increases (for fixed v)."""
        v = 5
        z_values = [0.0, 0.5, 1.0, 2.0, 5.0]
        results = [_logpdf_student_t(z, v) for z in z_values]
        
        # Logpdf should decrease as |z| increases
        for i in range(1, len(results)):
            self.assertGreater(results[i-1], results[i])
    
    def test_edge_case_v_equals_one(self):
        """Test edge case with v=1 (Cauchy distribution)."""
        z = 1.0
        result = _logpdf_student_t(z, v=1)
        expected = (
            math.lgamma(1)
            - math.lgamma(0.5)
            - 0.5 * math.log(math.pi)
            - math.log1p(z * z)
        )
        self.assertAlmostEqual(result, expected)
    
    def test_edge_case_v_equals_two(self):
        """Test edge case with v=2."""
        z = 1.0
        result = _logpdf_student_t(z, v=2)
        expected = (
            math.lgamma(1.5)
            - math.lgamma(1)
            - 0.5 * math.log(2 * math.pi)
            - 1.5 * math.log1p(z * z / 2)
        )
        self.assertAlmostEqual(result, expected)
    
    def test_very_small_z(self):
        """Test logpdf with very small z values."""
        z = 1e-10
        v = 5
        result = _logpdf_student_t(z, v)
        # Should be very close to the value at z=0
        expected_at_zero = _logpdf_student_t(0.0, v)
        self.assertAlmostEqual(result, expected_at_zero, places=10)


class TestCep95FromConf(unittest.TestCase):
    """Test cases for the _cep95_from_conf function."""
    
    def test_exact_table_values(self):
        """Test that function returns exact values for table entries."""
        # Test with the default LOOKUP_CEP95 table values
        test_cases = [(0.05, 7.4), (0.17, 6.1), (0.43, 4.3), (0.80, 2.5)]
        
        for p_conf, expected_cep95 in test_cases:
            result = _cep95_from_conf(p_conf)
            self.assertAlmostEqual(result, expected_cep95, places=10)
    
    def test_interpolation_between_values(self):
        """Test linear interpolation between table values."""
        # Test interpolation between 0.05 and 0.17
        p_conf = 0.11  # Midpoint between 0.05 and 0.17
        result = _cep95_from_conf(p_conf)
        expected = 6.75  # Midpoint between 7.4 and 6.1
        self.assertAlmostEqual(result, expected, places=10)
        
        # Test interpolation between 0.17 and 0.43
        p_conf = 0.30  # Midpoint between 0.17 and 0.43
        result = _cep95_from_conf(p_conf)
        expected = 5.2  # Midpoint between 6.1 and 4.3
        self.assertAlmostEqual(result, expected, places=10)
    
    def test_below_minimum_value(self):
        """Test behavior when p_conf is below the minimum table value."""
        p_conf = 0.01  # Below minimum of 0.05
        result = _cep95_from_conf(p_conf)
        expected = 7.4  # Should return the maximum CEP95 value
        self.assertAlmostEqual(result, expected, places=10)
    
    def test_above_maximum_value(self):
        """Test behavior when p_conf is above the maximum table value."""
        p_conf = 0.95  # Above maximum of 0.80
        result = _cep95_from_conf(p_conf)
        expected = 2.5  # Should return the minimum CEP95 value
        self.assertAlmostEqual(result, expected, places=10)
    
    def test_edge_case_minimum(self):
        """Test edge case at the minimum table value."""
        p_conf = 0.05
        result = _cep95_from_conf(p_conf)
        expected = 7.4
        self.assertAlmostEqual(result, expected, places=10)
    
    def test_edge_case_maximum(self):
        """Test edge case at the maximum table value."""
        p_conf = 0.80
        result = _cep95_from_conf(p_conf)
        expected = 2.5
        self.assertAlmostEqual(result, expected, places=10)
    
    def test_custom_table(self):
        """Test function with a custom lookup table."""
        custom_table = [(0.1, 10.0), (0.5, 5.0), (0.9, 1.0)]
        
        # Test exact values
        self.assertAlmostEqual(_cep95_from_conf(0.1, custom_table), 10.0, places=10)
        self.assertAlmostEqual(_cep95_from_conf(0.5, custom_table), 5.0, places=10)
        self.assertAlmostEqual(_cep95_from_conf(0.9, custom_table), 1.0, places=10)
        
        # Test interpolation
        self.assertAlmostEqual(_cep95_from_conf(0.3, custom_table), 7.5, places=10)  # Midpoint between 0.1 and 0.5
        self.assertAlmostEqual(_cep95_from_conf(0.7, custom_table), 3.0, places=10)  # Midpoint between 0.5 and 0.9
        
        # Test boundary conditions
        self.assertAlmostEqual(_cep95_from_conf(0.05, custom_table), 10.0, places=10)  # Below minimum
        self.assertAlmostEqual(_cep95_from_conf(0.95, custom_table), 1.0, places=10)   # Above maximum
    
    def test_monotonicity(self):
        """Test that CEP95 decreases as confidence increases (monotonic behavior)."""
        p_values = [0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
        cep95_values = [_cep95_from_conf(p) for p in p_values]
        
        # CEP95 should decrease as confidence increases
        for i in range(1, len(cep95_values)):
            self.assertGreaterEqual(cep95_values[i-1], cep95_values[i])
    
    def test_zero_confidence(self):
        """Test behavior with zero confidence."""
        p_conf = 0.0
        result = _cep95_from_conf(p_conf)
        expected = 7.4  # Should return maximum CEP95 value
        self.assertAlmostEqual(result, expected, places=10)
    
    def test_one_confidence(self):
        """Test behavior with confidence of 1.0."""
        p_conf = 1.0
        result = _cep95_from_conf(p_conf)
        expected = 2.5  # Should return minimum CEP95 value
        self.assertAlmostEqual(result, expected, places=10)
    
    def test_negative_confidence(self):
        """Test behavior with negative confidence."""
        p_conf = -0.1
        result = _cep95_from_conf(p_conf)
        expected = 7.4  # Should return maximum CEP95 value
        self.assertAlmostEqual(result, expected, places=10)
    
    def test_confidence_greater_than_one(self):
        """Test behavior with confidence greater than 1.0."""
        p_conf = 1.5
        result = _cep95_from_conf(p_conf)
        expected = 2.5  # Should return minimum CEP95 value
        self.assertAlmostEqual(result, expected, places=10)
    
    def test_precision_accuracy(self):
        """Test that interpolation provides accurate results."""
        # Test a value that should be exactly interpolated
        p_conf = 0.31  # 31% of the way from 0.17 to 0.43
        result = _cep95_from_conf(p_conf)
        # Manual calculation using the same logic as _cep95_from_conf
        xs = [0.05, 0.17, 0.43, 0.80]
        ys = [7.4, 6.1, 4.3, 2.5]
        i = next(j for j, x in enumerate(xs) if x > p_conf) - 1
        t = (p_conf - xs[i]) / (xs[i+1] - xs[i])
        expected = ys[i] + t * (ys[i+1] - ys[i])
        self.assertAlmostEqual(result, expected, places=10)


if __name__ == '__main__':
    unittest.main()
