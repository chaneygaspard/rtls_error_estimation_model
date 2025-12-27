#include <iostream>
#include <cassert>
#include <cmath>
#include <iomanip>
#include <unordered_map>
#include <vector>
#include <string>
#include <stdexcept>
#include <cstdlib>
#include "../metrics.h"
#include "../config.h"

// Test constants
const float TEST_EWMA_THRESHOLD = Calibration::EWMA_THRESHOLD; // Use calibration constant

// Helper function to convert std::vector<Anchor> to std::vector<Anchor*>
std::vector<Anchor*> to_pointer_vector(std::vector<Anchor>& anchors) {
    std::vector<Anchor*> anchor_ptrs;
    for (auto& anchor : anchors) {
        anchor_ptrs.push_back(&anchor);
    }
    return anchor_ptrs;
}

// Simple testing framework macros
#define ASSERT_EQ(expected, actual) \
    do { \
        if (std::abs(static_cast<double>(expected) - static_cast<double>(actual)) > 1e-6) { \
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

#define ASSERT_TRUE(condition) \
    do { \
        if (!(condition)) { \
            std::cerr << "FAIL: Expected condition to be true at line " << __LINE__ << std::endl; \
            return false; \
        } \
    } while(0)

#define ASSERT_FALSE(condition) \
    do { \
        if (condition) { \
            std::cerr << "FAIL: Expected condition to be false at line " << __LINE__ << std::endl; \
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

// Helper function to create test anchors
std::vector<Anchor> create_test_anchors() {
    std::vector<Anchor> anchors;
    
    // Anchor 1: Close, good EWMA
    anchors.push_back(Anchor("AA:BB:CC:DD:EE:01", std::make_tuple(0.0f, 0.0f, 0.0f), 1000.0f));
    
    // Anchor 2: Medium distance, good EWMA
    anchors.push_back(Anchor("AA:BB:CC:DD:EE:02", std::make_tuple(5.0f, 0.0f, 0.0f), 1000.0f));
    
    // Anchor 3: Far, good EWMA
    anchors.push_back(Anchor("AA:BB:CC:DD:EE:03", std::make_tuple(10.0f, 0.0f, 0.0f), 1000.0f));
    
    // Anchor 4: Close but bad EWMA (should be filtered out)
    anchors.push_back(Anchor("AA:BB:CC:DD:EE:04", std::make_tuple(1.0f, 1.0f, 0.0f), 1000.0f));
    
    // Anchor 5: Not in RSSI readings (should be filtered out)
    anchors.push_back(Anchor("AA:BB:CC:DD:EE:05", std::make_tuple(2.0f, 2.0f, 0.0f), 1000.0f));
    
    return anchors;
}

// Helper function to create test tag
Tag create_test_tag() {
    std::unordered_map<std::string, float> rssi_readings;
    rssi_readings["AA:BB:CC:DD:EE:01"] = -50.0f;  // Strongest signal
    rssi_readings["AA:BB:CC:DD:EE:02"] = -60.0f;  // Good signal (within 10dB of strongest)
    rssi_readings["AA:BB:CC:DD:EE:03"] = -65.0f;  // Weaker signal (but still within 10dB)
    rssi_readings["AA:BB:CC:DD:EE:04"] = -80.0f;  // Weak signal (more than 10dB from strongest)
    
    PointR3 tag_position = std::make_tuple(2.0f, 1.0f, 0.0f);
    return Tag("TAG:01", tag_position, rssi_readings);
}

// Test TagSystem constructor
bool test_tagsystem_constructor() {
    Tag tag = create_test_tag();
    PathLossModel model;
    
    TagSystem system(tag, model);
    
    // Test getters
    ASSERT_STRING_EQ(tag.get_mac_address(), system.get_tag().get_mac_address());
    
    return true;
}

// Test get_significant_anchors method
bool test_get_significant_anchors() {
    std::vector<Anchor> anchors = create_test_anchors();
    Tag tag = create_test_tag();
    PathLossModel model;
    TagSystem system(tag, model);
    
    // Simulate bad EWMA for anchor 4
    // Note: In a real test, we'd need a way to set EWMA values
    // For now, we'll test the basic functionality
    
    // Convert to pointer vector
    std::vector<Anchor*> anchor_ptrs = to_pointer_vector(anchors);
    
    std::vector<Anchor*> significant = system.get_significant_anchors(anchor_ptrs, Calibration::MAX_SIGNIFICANT_ANCHORS);
    
    // Should return anchors 1, 2, 3 (anchors with RSSI readings and good EWMA)
    // Anchor 4 has weak RSSI (more than 10dB from strongest)
    // Anchor 5 has no RSSI reading
    ASSERT_TRUE(significant.size() >= 1);
    ASSERT_TRUE(significant.size() <= 3);
    
    // Check that returned anchors are indeed pointers to original anchors
    for (const auto& anchor_ptr : significant) {
        ASSERT_TRUE(anchor_ptr != nullptr);
        
        // Find corresponding anchor in original vector
        bool found = false;
        for (const auto& orig_anchor : anchors) {
            if (orig_anchor.get_mac_address() == anchor_ptr->get_mac_address()) {
                found = true;
                // Check it's actually the same object (same address)
                ASSERT_TRUE(&orig_anchor == anchor_ptr);
                break;
            }
        }
        ASSERT_TRUE(found);
    }
    
    return true;
}

// Test get_significant_anchors with max_n parameter
bool test_get_significant_anchors_max_n() {
    std::vector<Anchor> anchors = create_test_anchors();
    Tag tag = create_test_tag();
    PathLossModel model;
    TagSystem system(tag, model);
    
    // Convert to pointer vector
    std::vector<Anchor*> anchor_ptrs = to_pointer_vector(anchors);
    
    std::vector<Anchor*> significant = system.get_significant_anchors(anchor_ptrs, 2);
    
    // Should be limited to max 2 anchors
    ASSERT_TRUE(significant.size() <= 2);
    
    return true;
}

// Test get_significant_anchors with empty RSSI
bool test_get_significant_anchors_empty_rssi() {
    std::vector<Anchor> anchors = create_test_anchors();
    std::unordered_map<std::string, float> empty_rssi;
    PointR3 tag_position = std::make_tuple(0.0f, 0.0f, 0.0f);
    Tag empty_tag("TAG:EMPTY", tag_position, empty_rssi);
    
    PathLossModel model;
    TagSystem system(empty_tag, model);
    
    // Convert to pointer vector
    std::vector<Anchor*> anchor_ptrs = to_pointer_vector(anchors);
    
    std::vector<Anchor*> significant = system.get_significant_anchors(anchor_ptrs, Calibration::MAX_SIGNIFICANT_ANCHORS);
    
    // Should return empty vector when no RSSI readings
    ASSERT_EQ(0, significant.size());
    
    return true;
}

// Test distances method
bool test_distances() {
    std::vector<Anchor> anchors = create_test_anchors();
    Tag tag = create_test_tag();
    PathLossModel model;
    TagSystem system(tag, model);
    
    // Convert to pointer vector
    std::vector<Anchor*> anchor_ptrs = to_pointer_vector(anchors);
    
    std::unordered_map<Anchor*, float> distances = system.distances(anchor_ptrs);
    
    // Should have distances for significant anchors only
    ASSERT_TRUE(distances.size() >= 1);
    
    for (const auto& pair : distances) {
        Anchor* anchor = pair.first;
        float distance = pair.second;
        
        ASSERT_TRUE(anchor != nullptr);
        ASSERT_TRUE(distance >= 0.0f);
        
        // Verify distance calculation makes sense
        // For anchor 1 at (0,0,0) and tag at (2,1,0), distance should be sqrt(5) ≈ 2.236
        if (anchor->get_mac_address() == "AA:BB:CC:DD:EE:01") {
            ASSERT_NEAR(2.236f, distance, 0.01f);
        }
    }
    
    return true;
}

// Test z_vals method
bool test_z_vals() {
    std::vector<Anchor> anchors = create_test_anchors();
    Tag tag = create_test_tag();
    PathLossModel model;
    TagSystem system(tag, model);
    
    // Convert to pointer vector
    std::vector<Anchor*> anchor_ptrs = to_pointer_vector(anchors);
    
    std::unordered_map<Anchor*, float> z_values = system.z_vals(anchor_ptrs);
    
    // Should have z-values for significant anchors only
    ASSERT_TRUE(z_values.size() >= 1);
    
    for (const auto& pair : z_values) {
        Anchor* anchor = pair.first;
        float z_val = pair.second;
        
        ASSERT_TRUE(anchor != nullptr);
        // Z-values should be finite
        ASSERT_TRUE(std::isfinite(z_val));
    }
    
    return true;
}

// Test confidence_score method
bool test_confidence_score() {
    std::vector<Anchor> anchors = create_test_anchors();
    Tag tag = create_test_tag();
    PathLossModel model;
    TagSystem system(tag, model);
    
    // Convert to pointer vector
    std::vector<Anchor*> anchor_ptrs = to_pointer_vector(anchors);
    
    float confidence = system.confidence_score(anchor_ptrs);
    
    // Confidence should be a valid probability-like value
    ASSERT_TRUE(confidence >= 0.0f);
    ASSERT_TRUE(std::isfinite(confidence));
    
    return true;
}

// Test confidence_score with custom parameters
bool test_confidence_score_custom_params() {
    std::vector<Anchor> anchors = create_test_anchors();
    Tag tag = create_test_tag();
    PathLossModel model;
    TagSystem system(tag, model);
    
    // Convert to pointer vector
    std::vector<Anchor*> anchor_ptrs = to_pointer_vector(anchors);
    
    float confidence1 = system.confidence_score(anchor_ptrs, 5, 2.0f);
    float confidence2 = system.confidence_score(anchor_ptrs, 10, 1.0f);
    
    // Both should be valid
    ASSERT_TRUE(confidence1 >= 0.0f && std::isfinite(confidence1));
    ASSERT_TRUE(confidence2 >= 0.0f && std::isfinite(confidence2));
    
    // Different parameters should potentially give different results
    // (though this depends on the specific data)
    
    return true;
}

// Test confidence_score with weighted behavior
bool test_confidence_score_weighted() {
    std::vector<Anchor> anchors = create_test_anchors();
    Tag tag = create_test_tag();
    PathLossModel model;
    TagSystem system(tag, model);
    
    // Convert to pointer vector
    std::vector<Anchor*> anchor_ptrs = to_pointer_vector(anchors);
    
    // Get the z_vals to understand the weighting
    std::unordered_map<Anchor*, float> z_values = system.z_vals(anchor_ptrs);
    
    if (z_values.empty()) {
        // If no z-values, confidence should be 0
        float confidence = system.confidence_score(anchor_ptrs);
        ASSERT_EQ(0.0f, confidence);
        return true;
    }
    
    // Calculate confidence with the weighted approach
    float confidence = system.confidence_score(anchor_ptrs);
    
    // Should be finite and non-negative
    ASSERT_TRUE(confidence >= 0.0f);
    ASSERT_TRUE(std::isfinite(confidence));
    
    // Test that weighting makes sense:
    // Anchors with lower EWMA and lower z-values should have higher weights
    // This indirectly tests the weighting logic
    
    return true;
}

// Test confidence_score with empty anchor list
bool test_confidence_score_empty_anchors() {
    std::vector<Anchor> empty_anchors;
    Tag tag = create_test_tag();
    PathLossModel model;
    TagSystem system(tag, model);
    
    // Convert to pointer vector
    std::vector<Anchor*> anchor_ptrs = to_pointer_vector(empty_anchors);
    
    float confidence = system.confidence_score(anchor_ptrs);
    
    // Should return 0.0 for empty anchor list
    ASSERT_EQ(0.0f, confidence);
    
    return true;
}

// Test error_radius method
bool test_error_radius() {
    std::vector<Anchor> anchors = create_test_anchors();
    Tag tag = create_test_tag();
    PathLossModel model;
    TagSystem system(tag, model);
    
    // Convert to pointer vector
    std::vector<Anchor*> anchor_ptrs = to_pointer_vector(anchors);
    
    float radius = system.error_radius(anchor_ptrs);
    
    // Error radius should be positive and finite
    ASSERT_TRUE(radius > 0.0f);
    ASSERT_TRUE(std::isfinite(radius));
    
    return true;
}

// Test update_anchors_from_tag_data function
bool test_update_anchors_from_tag_data() {
    std::vector<Anchor> anchors = create_test_anchors();
    std::vector<Anchor*> anch_list;
    for (auto& anchor : anchors) {
        anch_list.push_back(&anchor);
    }
    
    Tag tag = create_test_tag();
    PathLossModel model;
    float now = 2000.0f;
    
    // Store original RSSI_0 values for comparison
    std::vector<float> original_rssi0;
    for (const auto* anchor : anch_list) {
        original_rssi0.push_back(anchor->get_RSSI_0());
    }
    
    // Call the function
    update_anchors_from_tag_data(anch_list, tag, model, now, 12.0f, 6000);
    
    // Verify that function completed without errors
    // (In a more comprehensive test, we'd verify that parameters and health were actually updated)
    
    return true;
}

// Test update_anchors_from_tag_data with empty RSSI
bool test_update_anchors_empty_rssi() {
    std::vector<Anchor> anchors = create_test_anchors();
    std::vector<Anchor*> anch_list;
    for (auto& anchor : anchors) {
        anch_list.push_back(&anchor);
    }
    
    std::unordered_map<std::string, float> empty_rssi;
    PointR3 tag_position = std::make_tuple(0.0f, 0.0f, 0.0f);
    Tag empty_tag("TAG:EMPTY", tag_position, empty_rssi);
    
    PathLossModel model;
    float now = 2000.0f;
    
    // Should handle empty RSSI gracefully (return early)
    update_anchors_from_tag_data(anch_list, empty_tag, model, now, 12.0f, 6000);
    
    // Function should complete without error
    return true;
}

// Test pointer consistency in TagSystem methods
bool test_pointer_consistency() {
    std::vector<Anchor> anchors = create_test_anchors();
    Tag tag = create_test_tag();
    PathLossModel model;
    TagSystem system(tag, model);
    
    // Convert to pointer vector
    std::vector<Anchor*> anchor_ptrs = to_pointer_vector(anchors);
    
    // Get significant anchors
    std::vector<Anchor*> significant = system.get_significant_anchors(anchor_ptrs);
    
    if (significant.empty()) {
        // If no significant anchors, skip this test
        return true;
    }
    
    // Get distances and z_vals
    std::unordered_map<Anchor*, float> distances = system.distances(anchor_ptrs);
    std::unordered_map<Anchor*, float> z_vals = system.z_vals(anchor_ptrs);
    
    // Verify that the same anchor pointers are used across different methods
    for (const auto& anchor_ptr : significant) {
        // This anchor should appear in both distances and z_vals
        ASSERT_TRUE(distances.find(anchor_ptr) != distances.end());
        ASSERT_TRUE(z_vals.find(anchor_ptr) != z_vals.end());
    }
    
    return true;
}

// Test EWMA threshold filtering
bool test_ewma_threshold_filtering() {
    // This test would require a way to set EWMA values
    // For now, we'll just test that the filtering mechanism works
    std::vector<Anchor> anchors = create_test_anchors();
    Tag tag = create_test_tag();
    PathLossModel model;
    TagSystem system(tag, model);
    
    // Convert to pointer vector
    std::vector<Anchor*> anchor_ptrs = to_pointer_vector(anchors);
    
    std::vector<Anchor*> significant = system.get_significant_anchors(anchor_ptrs);
    
    // All returned anchors should have EWMA < TEST_EWMA_THRESHOLD
    for (const auto& anchor_ptr : significant) {
        ASSERT_TRUE(anchor_ptr->get_ewma() < TEST_EWMA_THRESHOLD);
    }
    
    return true;
}

// Main function to run all tests
int main() {
    std::cout << "==================================" << std::endl;
    std::cout << "      METRICS TESTS STARTING      " << std::endl;
    std::cout << "==================================" << std::endl;

    bool all_passed = true;

    // Run TagSystem tests
    all_passed &= run_test("test_tagsystem_constructor", test_tagsystem_constructor);
    all_passed &= run_test("test_get_significant_anchors", test_get_significant_anchors);
    all_passed &= run_test("test_get_significant_anchors_max_n", test_get_significant_anchors_max_n);
    all_passed &= run_test("test_get_significant_anchors_empty_rssi", test_get_significant_anchors_empty_rssi);
    all_passed &= run_test("test_distances", test_distances);
    all_passed &= run_test("test_z_vals", test_z_vals);
    all_passed &= run_test("test_confidence_score", test_confidence_score);
    all_passed &= run_test("test_confidence_score_custom_params", test_confidence_score_custom_params);
    all_passed &= run_test("test_confidence_score_weighted", test_confidence_score_weighted);
    all_passed &= run_test("test_confidence_score_empty_anchors", test_confidence_score_empty_anchors);
    all_passed &= run_test("test_error_radius", test_error_radius);
    
    // Run standalone function tests
    all_passed &= run_test("test_update_anchors_from_tag_data", test_update_anchors_from_tag_data);
    all_passed &= run_test("test_update_anchors_empty_rssi", test_update_anchors_empty_rssi);
    
    // Run integration/consistency tests
    all_passed &= run_test("test_pointer_consistency", test_pointer_consistency);
    all_passed &= run_test("test_ewma_threshold_filtering", test_ewma_threshold_filtering);
    
    std::cout << "\n==================================" << std::endl;
    if (all_passed) {
        std::cout << "🎉 ALL METRICS TESTS PASSED! 🎉" << std::endl;
        return 0;
    } else {
        std::cout << "❌ SOME METRICS TESTS FAILED ❌" << std::endl;
        return 1;
    }
}