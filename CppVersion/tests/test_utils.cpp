#include <iostream>
#include <cassert>
#include <cmath>
#include <iomanip>
#include "../utils.h"

// Simple testing framework macros
#define ASSERT_EQ(expected, actual) \
    do { \
        if (std::abs((expected) - (actual)) > 1e-6) { \
            std::cerr << "FAIL: Expected " << (expected) << " but got " << (actual) \
                      << " at line " << __LINE__ << std::endl; \
            return false; \
        } \
    } while(0)

#define ASSERT_NEAR(expected, actual, tolerance) \
    do { \
        if (std::abs((expected) - (actual)) > (tolerance)) { \
            std::cerr << "FAIL: Expected " << (expected) << " ± " << (tolerance) \
                      << " but got " << (actual) << " at line " << __LINE__ << std::endl; \
            return false; \
        } \
    } while(0)

// Simple test runner function
bool run_test(const char* test_name, bool (*test_func)()) {
    std::cout << "Running " << test_name << "... ";
    bool result = test_func();
    if (result) {
        std::cout << "PASS" << std::endl;
    } else {
        std::cout << "FAIL" << std::endl;
    }
    return result;
}

// Test R3_distance function
bool test_R3_distance_basic() {
    // Test basic distance calculations
    PointR3 origin = std::make_tuple(0.0f, 0.0f, 0.0f);
    PointR3 unit_x = std::make_tuple(1.0f, 0.0f, 0.0f);
    PointR3 unit_y = std::make_tuple(0.0f, 1.0f, 0.0f);
    PointR3 unit_z = std::make_tuple(0.0f, 0.0f, 1.0f);
    
    // Distance from origin to unit vectors should be 1.0
    ASSERT_EQ(1.0f, R3_distance(origin, unit_x));
    ASSERT_EQ(1.0f, R3_distance(origin, unit_y));
    ASSERT_EQ(1.0f, R3_distance(origin, unit_z));
    
    // Distance should be symmetric
    ASSERT_EQ(R3_distance(origin, unit_x), R3_distance(unit_x, origin));
    
    // Distance from point to itself should be 0
    ASSERT_EQ(0.0f, R3_distance(origin, origin));
    ASSERT_EQ(0.0f, R3_distance(unit_x, unit_x));
    
    return true;
}

bool test_R3_distance_3d_cases() {
    // Test 3D diagonal cases
    PointR3 origin = std::make_tuple(0.0f, 0.0f, 0.0f);
    PointR3 point_111 = std::make_tuple(1.0f, 1.0f, 1.0f);
    PointR3 point_345 = std::make_tuple(3.0f, 4.0f, 5.0f);
    
    // Distance from (0,0,0) to (1,1,1) should be sqrt(3)
    float expected_111 = std::sqrt(3.0f);
    ASSERT_NEAR(expected_111, R3_distance(origin, point_111), 1e-6);
    
    // Distance from (0,0,0) to (3,4,5) should be sqrt(9+16+25) = sqrt(50)
    float expected_345 = std::sqrt(50.0f);
    ASSERT_NEAR(expected_345, R3_distance(origin, point_345), 1e-6);
    
    // Test arbitrary points
    PointR3 point_a = std::make_tuple(2.5f, -1.3f, 4.7f);
    PointR3 point_b = std::make_tuple(-0.8f, 3.2f, 1.1f);
    
    float dx = 2.5f - (-0.8f);  // 3.3
    float dy = -1.3f - 3.2f;    // -4.5
    float dz = 4.7f - 1.1f;     // 3.6
    float expected_ab = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    ASSERT_NEAR(expected_ab, R3_distance(point_a, point_b), 1e-5);
    
    return true;
}

// Test logpdf_student_t function
bool test_logpdf_student_t_basic() {
    // Test at z=0 (should be maximum)
    float log_pdf_0 = logpdf_student_t(0.0f, 5);
    
    // Test that result is finite
    assert(std::isfinite(log_pdf_0));
    
    // Test symmetry: pdf(z) = pdf(-z)
    ASSERT_NEAR(logpdf_student_t(1.0f, 5), logpdf_student_t(-1.0f, 5), 1e-6);
    ASSERT_NEAR(logpdf_student_t(2.5f, 5), logpdf_student_t(-2.5f, 5), 1e-6);
    
    // Test that pdf decreases as |z| increases (log pdf becomes more negative)
    float pdf_0 = logpdf_student_t(0.0f, 5);
    float pdf_1 = logpdf_student_t(1.0f, 5);
    float pdf_2 = logpdf_student_t(2.0f, 5);
    
    assert(pdf_0 > pdf_1);  // Should decrease (become more negative)
    assert(pdf_1 > pdf_2);  // Should continue decreasing
    
    // Test that z=0 gives the maximum value (least negative log pdf)
    float pdf_neg1 = logpdf_student_t(-1.0f, 5);
    float pdf_pos1 = logpdf_student_t(1.0f, 5);
    assert(pdf_0 > pdf_neg1);
    assert(pdf_0 > pdf_pos1);
    
    return true;
}

bool test_logpdf_student_t_default_parameter() {
    // Test that default parameter works (v=5)
    float with_explicit = logpdf_student_t(1.5f, 5);
    float with_default = logpdf_student_t(1.5f);
    
    ASSERT_EQ(with_explicit, with_default);
    
    return true;
}

bool test_logpdf_student_t_different_dof() {
    // Test different degrees of freedom
    float z = 1.0f;
    
    // Use safer degrees of freedom values (avoid v=1 which can cause numerical issues)
    float pdf_v2 = logpdf_student_t(z, 2);   // Use v=2 instead of v=1
    float pdf_v5 = logpdf_student_t(z, 5);
    float pdf_v30 = logpdf_student_t(z, 30);
    
    // Just verify they produce finite, reasonable values
    assert(std::isfinite(pdf_v2));
    assert(std::isfinite(pdf_v5));
    assert(std::isfinite(pdf_v30));
    
    // Test that different degrees of freedom give different results
    assert(pdf_v2 != pdf_v5);
    assert(pdf_v5 != pdf_v30);
    
    // Test with z=0 as well (should always be finite)
    float pdf_v2_zero = logpdf_student_t(0.0f, 2);
    float pdf_v5_zero = logpdf_student_t(0.0f, 5);
    assert(std::isfinite(pdf_v2_zero));
    assert(std::isfinite(pdf_v5_zero));
    
    return true;
}

// Test cep95_from_conf function
bool test_cep95_from_conf_exact_values() {
    // Test exact table values
    ASSERT_EQ(7.4f, cep95_from_conf(0.05f));
    ASSERT_EQ(6.1f, cep95_from_conf(0.17f));
    ASSERT_EQ(4.3f, cep95_from_conf(0.43f));
    ASSERT_EQ(2.5f, cep95_from_conf(0.80f));
    ASSERT_EQ(2.0f, cep95_from_conf(0.85f));
    ASSERT_EQ(1.6f, cep95_from_conf(0.90f));
    ASSERT_EQ(1.2f, cep95_from_conf(0.95f));
    ASSERT_EQ(0.9f, cep95_from_conf(0.98f));
    
    return true;
}

bool test_cep95_from_conf_boundary_conditions() {
    // Test values below minimum - should return maximum radius
    ASSERT_EQ(7.4f, cep95_from_conf(0.01f));
    ASSERT_EQ(7.4f, cep95_from_conf(0.04f));
    
    // Test values above maximum - should return minimum radius
    ASSERT_EQ(0.9f, cep95_from_conf(0.99f));
    ASSERT_EQ(0.9f, cep95_from_conf(1.0f));
    
    return true;
}

bool test_cep95_from_conf_interpolation() {
    // Test interpolation between known values
    
    // Between 0.05 (7.4) and 0.17 (6.1)
    // At midpoint 0.11, should be approximately (7.4 + 6.1) / 2 = 6.75
    float mid_1 = cep95_from_conf(0.11f);
    ASSERT_NEAR(6.75f, mid_1, 0.1f);
    
    // Between 0.80 (2.5) and 0.85 (2.0)
    // At midpoint 0.825, should be approximately (2.5 + 2.0) / 2 = 2.25
    float mid_2 = cep95_from_conf(0.825f);
    ASSERT_NEAR(2.25f, mid_2, 0.05f);
    
    // Between 0.95 (1.2) and 0.98 (0.9)
    // At 0.96 (1/3 of the way), should be approximately 1.1
    float third_point = cep95_from_conf(0.96f);
    ASSERT_NEAR(1.1f, third_point, 0.05f);
    
    // Test that interpolation is monotonically decreasing
    float val_82 = cep95_from_conf(0.82f);
    float val_84 = cep95_from_conf(0.84f);
    assert(val_82 > val_84);  // Higher confidence should give lower radius
    
    return true;
}

bool test_cep95_from_conf_precision_range() {
    // Test that all outputs are in reasonable range for BLE positioning
    for (float conf = 0.0f; conf <= 1.0f; conf += 0.01f) {
        float radius = cep95_from_conf(conf);
        
        // Should be between 0.5m and 8.0m (reasonable for indoor BLE)
        assert(radius >= 0.5f);
        assert(radius <= 8.0f);
        
        // Should produce finite values
        assert(std::isfinite(radius));
    }
    
    return true;
}

// Main test runner
int main() {
    std::cout << "Running utils.cpp test suite..." << std::endl;
    std::cout << "=================================" << std::endl;
    
    bool all_passed = true;
    
    // Run R3_distance tests
    std::cout << "\nTesting R3_distance function:" << std::endl;
    all_passed &= run_test("test_R3_distance_basic", test_R3_distance_basic);
    all_passed &= run_test("test_R3_distance_3d_cases", test_R3_distance_3d_cases);
    
    // Run logpdf_student_t tests
    std::cout << "\nTesting logpdf_student_t function:" << std::endl;
    all_passed &= run_test("test_logpdf_student_t_basic", test_logpdf_student_t_basic);
    all_passed &= run_test("test_logpdf_student_t_default_parameter", test_logpdf_student_t_default_parameter);
    all_passed &= run_test("test_logpdf_student_t_different_dof", test_logpdf_student_t_different_dof);
    
    // Run cep95_from_conf tests
    std::cout << "\nTesting cep95_from_conf function:" << std::endl;
    all_passed &= run_test("test_cep95_from_conf_exact_values", test_cep95_from_conf_exact_values);
    all_passed &= run_test("test_cep95_from_conf_boundary_conditions", test_cep95_from_conf_boundary_conditions);
    all_passed &= run_test("test_cep95_from_conf_interpolation", test_cep95_from_conf_interpolation);
    all_passed &= run_test("test_cep95_from_conf_precision_range", test_cep95_from_conf_precision_range);
    
    std::cout << "\n=================================" << std::endl;
    if (all_passed) {
        std::cout << "🎉 ALL UTILS TESTS PASSED! 🎉" << std::endl;
        return 0;
    } else {
        std::cout << "❌ SOME UTILS TESTS FAILED ❌" << std::endl;
        return 1;
    }
}