#include <iostream>
#include <cassert>
#include <cmath>
#include <iomanip>
#include <unordered_map>
#include <vector>
#include <string>
#include <stdexcept>
#include "../models.h"

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

#define ASSERT_STRING_EQ(expected, actual) \
    do { \
        if ((expected) != (actual)) { \
            std::cerr << "FAIL: Expected '" << (expected) << "' but got '" << (actual) \
                      << "' at line " << __LINE__ << std::endl; \
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

// Test Anchor class constructor and getters
bool test_anchor_constructor_and_getters() {
    // Create test anchor
    std::string mac = "AA:BB:CC:DD:EE:FF";
    PointR3 coord = std::make_tuple(1.5f, 2.3f, 0.0f);
    float timestamp = 1234567.89f;
    
    Anchor anchor(mac, coord, timestamp);
    
    // Test getters
    ASSERT_STRING_EQ(mac, anchor.get_mac_address());
    
    PointR3 retrieved_coord = anchor.get_coord();
    ASSERT_EQ(1.5f, std::get<0>(retrieved_coord));
    ASSERT_EQ(2.3f, std::get<1>(retrieved_coord));
    ASSERT_EQ(0.0f, std::get<2>(retrieved_coord));
    
    ASSERT_EQ(timestamp, anchor.get_last_seen());
    ASSERT_EQ(1.0f, anchor.get_ewma());  // Default value
    ASSERT_EQ(-59.0f, anchor.get_RSSI_0());  // Default value
    ASSERT_EQ(2.0f, anchor.get_n());  // Default value
    
    // Test that Kalman filter is accessible
    const KalmanFilter& kf = anchor.get_kalman();
    (void)kf;  // Suppress unused variable warning
    
    return true;
}

// Test Anchor health monitoring
bool test_anchor_health_monitoring() {
    Anchor anchor("test:mac", std::make_tuple(0.0f, 0.0f, 0.0f), 0.0f);
    
    // Initial state should be healthy
    assert(!anchor.is_warning());
    assert(!anchor.is_faulty());
    
    // Update health with moderate residual (using default LAMBDA=0.05)
    anchor.update_health(3.0f, 100.0f);  // z=3, EWMA should increase
    
    // Should still be healthy after one update
    assert(!anchor.is_warning());
    assert(!anchor.is_faulty());
    
    // Repeatedly update with high residuals to trigger warning (using default LAMBDA=0.05)
    for (int i = 0; i < 50; i++) {
        anchor.update_health(5.0f, 100.0f + i);
    }
    
    // Should now be in warning or faulty state
    bool is_warning_or_faulty = anchor.is_warning() || anchor.is_faulty();
    assert(is_warning_or_faulty);
    
    // Verify timestamp was updated
    ASSERT_EQ(149.0f, anchor.get_last_seen());
    
    return true;
}

// Test Anchor parameter updates
bool test_anchor_parameter_updates() {
    Anchor anchor("test:mac", std::make_tuple(0.0f, 0.0f, 0.0f), 0.0f);
    
    // Get initial parameters
    float initial_RSSI_0 = anchor.get_RSSI_0();
    float initial_n = anchor.get_n();
    
    // Update parameters with measurement
    anchor.update_parameters(-50.0f, 3.0f);
    
    // Parameters should have changed (due to Kalman filter)
    float updated_RSSI_0 = anchor.get_RSSI_0();
    float updated_n = anchor.get_n();
    
    // Values should be finite and in reasonable ranges
    assert(std::isfinite(updated_RSSI_0));
    assert(std::isfinite(updated_n));
    assert(updated_RSSI_0 >= -100.0f && updated_RSSI_0 <= 0.0f);
    assert(updated_n >= 0.5f && updated_n <= 10.0f);
    
    // Verify that parameters may have changed (Kalman filter effect)
    // Note: They might not change significantly with just one measurement
    (void)initial_RSSI_0;  // Suppress unused variable warning
    (void)initial_n;       // Suppress unused variable warning
    
    return true;
}

// Test Tag class constructor and getters
bool test_tag_constructor_and_getters() {
    // Create test data
    std::string mac = "11:22:33:44:55:66";
    PointR3 coord = std::make_tuple(5.0f, 3.0f, 1.0f);
    std::unordered_map<std::string, float> rssi_map = {
        {"anchor1", -45.0f},
        {"anchor2", -52.0f},
        {"anchor3", -38.0f}
    };
    
    Tag tag(mac, coord, rssi_map);
    
    // Test getters
    ASSERT_STRING_EQ(mac, tag.get_mac_address());
    
    PointR3 retrieved_coord = tag.get_est_coord();
    ASSERT_EQ(5.0f, std::get<0>(retrieved_coord));
    ASSERT_EQ(3.0f, std::get<1>(retrieved_coord));
    ASSERT_EQ(1.0f, std::get<2>(retrieved_coord));
    
    const auto& retrieved_map = tag.get_rssi_readings();
    assert(retrieved_map.size() == 3);
    assert(retrieved_map.at("anchor1") == -45.0f);
    assert(retrieved_map.at("anchor2") == -52.0f);
    assert(retrieved_map.at("anchor3") == -38.0f);
    
    return true;
}

// Test Tag RSSI methods
bool test_tag_rssi_methods() {
    std::unordered_map<std::string, float> rssi_map = {
        {"anchor_A", -60.0f},
        {"anchor_B", -45.0f},
        {"anchor_C", -55.0f}
    };
    
    Tag tag("test:tag", std::make_tuple(0.0f, 0.0f, 0.0f), rssi_map);
    
    // Test rssi_for_anchor
    ASSERT_EQ(-60.0f, tag.rssi_for_anchor("anchor_A"));
    ASSERT_EQ(-45.0f, tag.rssi_for_anchor("anchor_B"));
    ASSERT_EQ(-55.0f, tag.rssi_for_anchor("anchor_C"));
    
    // Test anchors_included
    std::vector<std::string> anchors = tag.anchors_included();
    assert(anchors.size() == 3);
    
    // Check that all expected anchors are included
    bool found_A = false, found_B = false, found_C = false;
    for (const auto& anchor : anchors) {
        if (anchor == "anchor_A") found_A = true;
        if (anchor == "anchor_B") found_B = true;
        if (anchor == "anchor_C") found_C = true;
    }
    assert(found_A && found_B && found_C);
    
    return true;
}

// Test Tag exception handling
bool test_tag_exception_handling() {
    std::unordered_map<std::string, float> rssi_map = {
        {"existing_anchor", -50.0f}
    };
    
    Tag tag("test:tag", std::make_tuple(0.0f, 0.0f, 0.0f), rssi_map);
    
    // Should work for existing anchor
    float rssi = tag.rssi_for_anchor("existing_anchor");
    ASSERT_EQ(-50.0f, rssi);
    
    // Should throw for non-existing anchor
    try {
        tag.rssi_for_anchor("non_existing_anchor");
        // Should not reach here
        assert(false);
    } catch (const std::out_of_range&) {
        // Expected exception
    }
    
    return true;
}

// Test PathLossModel constructor and getters
bool test_pathloss_constructor_and_getters() {
    PathLossModel model;
    
    // Test default values
    ASSERT_EQ(1.0f, model.get_d_0());
    ASSERT_EQ(4.0f, model.get_sigma());
    
    return true;
}

// Test PathLossModel mu calculation
bool test_pathloss_mu_calculation() {
    PathLossModel model;
    
    // Test with known values
    float RSSI_0 = -30.0f;
    float n = 2.0f;
    float distance = 10.0f;  // 10 meters
    
    float mu = model.mu(RSSI_0, n, distance);
    
    // Expected: -30 - (10 * 2 * log10(10/1)) = -30 - 20 = -50
    ASSERT_NEAR(-50.0f, mu, 0.1f);
    
    // Test with distance = 1 meter (should equal RSSI_0)
    float mu_at_1m = model.mu(RSSI_0, n, 1.0f);
    ASSERT_NEAR(RSSI_0, mu_at_1m, 0.1f);
    
    // Test edge case with very small distance
    float mu_small = model.mu(RSSI_0, n, 0.0f);
    assert(std::isfinite(mu_small));
    
    return true;
}

// Test PathLossModel z calculation
bool test_pathloss_z_calculation() {
    PathLossModel model;
    
    float RSSI_0 = -30.0f;
    float n = 2.0f;
    float distance = 10.0f;
    float observed_rssi = -52.0f;
    
    float z = model.z(observed_rssi, RSSI_0, n, distance);
    
    // Expected mu = -50, z = (-52 - (-50)) / 4 = -2/4 = -0.5
    ASSERT_NEAR(-0.5f, z, 0.1f);
    
    // Test with perfect measurement (z should be 0)
    float expected_rssi = model.mu(RSSI_0, n, distance);
    float z_perfect = model.z(expected_rssi, RSSI_0, n, distance);
    ASSERT_NEAR(0.0f, z_perfect, 0.01f);
    
    return true;
}

// Test PathLossModel mathematical properties
bool test_pathloss_mathematical_properties() {
    PathLossModel model;
    
    float RSSI_0 = -35.0f;
    float n = 2.5f;
    
    // Test that mu decreases with distance
    float mu_1m = model.mu(RSSI_0, n, 1.0f);
    float mu_2m = model.mu(RSSI_0, n, 2.0f);
    float mu_5m = model.mu(RSSI_0, n, 5.0f);
    
    assert(mu_1m > mu_2m);  // Signal gets weaker with distance
    assert(mu_2m > mu_5m);
    
    // Test symmetry of z-score
    float baseline_rssi = model.mu(RSSI_0, n, 3.0f);
    float offset = 2.0f;
    
    float z_positive = model.z(baseline_rssi + offset, RSSI_0, n, 3.0f);
    float z_negative = model.z(baseline_rssi - offset, RSSI_0, n, 3.0f);
    
    ASSERT_NEAR(z_positive, -z_negative, 0.01f);
    
    return true;
}

// Test integration between classes
bool test_classes_integration() {
    // Create anchor and path loss model
    Anchor anchor("test:anchor", std::make_tuple(0.0f, 0.0f, 0.0f), 0.0f);
    PathLossModel model;
    
    // Simulate measurement
    float distance = 5.0f;
    float expected_rssi = model.mu(anchor.get_RSSI_0(), anchor.get_n(), distance);
    float measured_rssi = expected_rssi + 1.0f;  // Add some noise
    
    // Calculate residual
    float z = model.z(measured_rssi, anchor.get_RSSI_0(), anchor.get_n(), distance);
    
    // Update anchor health with residual (using default LAMBDA=0.05)
    anchor.update_health(z, 100.0f);
    
    // Update anchor parameters
    anchor.update_parameters(measured_rssi, distance);
    
    // Verify everything worked without crashing
    assert(std::isfinite(anchor.get_ewma()));
    assert(std::isfinite(anchor.get_RSSI_0()));
    assert(std::isfinite(anchor.get_n()));
    
    return true;
}

// Test that Kalman filter updates RSSI_0 and n parameters in Anchor
bool test_anchor_kalman_parameter_updates() {
    // Create an anchor with default values
    std::string mac = "AA:BB:CC:DD:EE:KF";
    PointR3 coord = std::make_tuple(0.0f, 0.0f, 0.0f);
    float timestamp = 1000.0f;  // Current time
    Anchor anchor(mac, coord, timestamp);
    
    // Record initial values
    float initial_rssi_0 = anchor.get_RSSI_0();
    float initial_n = anchor.get_n();
    
    // Verify we have the expected default values
    ASSERT_EQ(-59.0f, initial_rssi_0);  // Should match the default RSSI_0
    ASSERT_EQ(2.0f, initial_n);         // Should match the default n
    
    // Simulate some measurements that should cause parameter updates
    // Use measurements that are significantly different from the model prediction
    // to force the Kalman filter to adjust the parameters
    
    // First measurement: stronger signal than expected (closer than estimated)
    float measured_rssi_1 = -45.0f;      // Stronger than default -59 dBm
    float estimated_distance_1 = 5.0f;   // Estimated 5 meters away
    
    anchor.update_parameters(measured_rssi_1, estimated_distance_1);
    
    float rssi_0_after_first = anchor.get_RSSI_0();
    float n_after_first = anchor.get_n();
    
    // Second measurement: weaker signal than expected (farther than estimated)
    float measured_rssi_2 = -75.0f;      // Weaker signal
    float estimated_distance_2 = 3.0f;   // But estimated closer
    
    anchor.update_parameters(measured_rssi_2, estimated_distance_2);
    
    float rssi_0_after_second = anchor.get_RSSI_0();
    float n_after_second = anchor.get_n();
    
    // Third measurement: confirm continued adaptation
    float measured_rssi_3 = -55.0f;
    float estimated_distance_3 = 4.0f;
    
    anchor.update_parameters(measured_rssi_3, estimated_distance_3);
    
    float final_rssi_0 = anchor.get_RSSI_0();
    float final_n = anchor.get_n();
    
    // Verify that parameters have changed from their initial values
    // The exact direction of change depends on Kalman filter internals,
    // but we should see some adaptation
    bool rssi_0_changed = (std::abs(final_rssi_0 - initial_rssi_0) > 1e-3);
    bool n_changed = (std::abs(final_n - initial_n) > 1e-3);
    
    if (!rssi_0_changed) {
        std::cerr << "FAIL: RSSI_0 did not change after Kalman updates. "
                  << "Initial: " << initial_rssi_0 << ", Final: " << final_rssi_0 << std::endl;
        return false;
    }
    
    if (!n_changed) {
        std::cerr << "FAIL: n parameter did not change after Kalman updates. "
                  << "Initial: " << initial_n << ", Final: " << final_n << std::endl;
        return false;
    }
    
    // Verify that the parameters remain within reasonable bounds
    // RSSI_0 should typically be negative (signal loss at 1m)
    if (final_rssi_0 > 0.0f) {
        std::cerr << "FAIL: Final RSSI_0 is positive (" << final_rssi_0 
                  << "), which is physically unrealistic" << std::endl;
        return false;
    }
    
    // Path loss exponent should be positive
    if (final_n <= 0.0f) {
        std::cerr << "FAIL: Final n parameter is non-positive (" << final_n 
                  << "), which is physically unrealistic" << std::endl;
        return false;
    }
    
    // Log the parameter evolution for debugging
    std::cout << "  Kalman parameter evolution:" << std::endl;
    std::cout << "    Initial: RSSI_0 = " << initial_rssi_0 << ", n = " << initial_n << std::endl;
    std::cout << "    After 1st update: RSSI_0 = " << rssi_0_after_first << ", n = " << n_after_first << std::endl;
    std::cout << "    After 2nd update: RSSI_0 = " << rssi_0_after_second << ", n = " << n_after_second << std::endl;
    std::cout << "    Final: RSSI_0 = " << final_rssi_0 << ", n = " << final_n << std::endl;
    
    return true;
}

// Main test runner
int main() {
    std::cout << "Running models.cpp test suite..." << std::endl;
    std::cout << "==================================" << std::endl;
    
    bool all_passed = true;
    
    // Run Anchor class tests
    std::cout << "\nTesting Anchor class:" << std::endl;
    all_passed &= run_test("test_anchor_constructor_and_getters", test_anchor_constructor_and_getters);
    all_passed &= run_test("test_anchor_health_monitoring", test_anchor_health_monitoring);
    all_passed &= run_test("test_anchor_parameter_updates", test_anchor_parameter_updates);
    all_passed &= run_test("test_anchor_kalman_parameter_updates", test_anchor_kalman_parameter_updates);
    
    // Run Tag class tests
    std::cout << "\nTesting Tag class:" << std::endl;
    all_passed &= run_test("test_tag_constructor_and_getters", test_tag_constructor_and_getters);
    all_passed &= run_test("test_tag_rssi_methods", test_tag_rssi_methods);
    all_passed &= run_test("test_tag_exception_handling", test_tag_exception_handling);
    
    // Run PathLossModel class tests
    std::cout << "\nTesting PathLossModel class:" << std::endl;
    all_passed &= run_test("test_pathloss_constructor_and_getters", test_pathloss_constructor_and_getters);
    all_passed &= run_test("test_pathloss_mu_calculation", test_pathloss_mu_calculation);
    all_passed &= run_test("test_pathloss_z_calculation", test_pathloss_z_calculation);
    all_passed &= run_test("test_pathloss_mathematical_properties", test_pathloss_mathematical_properties);
    
    // Run integration tests
    std::cout << "\nTesting class integration:" << std::endl;
    all_passed &= run_test("test_classes_integration", test_classes_integration);
    
    std::cout << "\n==================================" << std::endl;
    if (all_passed) {
        std::cout << "🎉 ALL MODELS TESTS PASSED! 🎉" << std::endl;
        return 0;
    } else {
        std::cout << "❌ SOME MODELS TESTS FAILED ❌" << std::endl;
        return 1;
    }
}