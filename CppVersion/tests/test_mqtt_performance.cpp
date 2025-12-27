#include <iostream>
#include <cassert>
#include <chrono>
#include <string>
#include <vector>
#include <iomanip>
#include <memory>
#include <unordered_map>
#include <algorithm>

// External libraries
#include <nlohmann/json.hpp>

// Include necessary headers
#include "../models.h"
#include "../metrics.h" 
#include "../utils.h"
#include "../config.h"

using json = nlohmann::json;

// Mock MQTT message structure (without requiring mosquitto library)
struct MockMQTTMessage {
    std::string payload;
    size_t payloadlen;
};

// Mock user data structure (same as in main.cpp)
struct MockMQTTUserData {
    std::unordered_map<std::string, std::unique_ptr<Anchor>> anchors;
    bool anchors_initialized = false;
    PathLossModel model;
};

// Sample MQTT message payloads for testing (using example format - actual data redacted)
std::vector<std::string> sample_mqtt_messages = {
    // Sample message 1: Real format with 3 used + 1 unused anchors
    R"({
        "is_moving": null,
        "location": {
            "dead_zones": [],
            "map_id": "<map_id_redacted>",
            "position": {
                "quality": "normal",
                "unused_anchors": [{"cart_d": 4.67, "id": "<anchor_id>", "mac": "<anchor_mac_redacted>", "rssi": -66.19}],
                "used_anchors": [
                    {"cart_d": 1.0, "id": "<anchor_id>", "mac": "<anchor_mac_redacted>", "rssi": -57.0},
                    {"cart_d": 2.07, "id": "<anchor_id>", "mac": "<anchor_mac_redacted>", "rssi": -59.47},
                    {"cart_d": 4.97, "id": "<anchor_id>", "mac": "<anchor_mac_redacted>", "rssi": -64.92}
                ],
                "x": 5.92,
                "y": 2.21,
                "z": 0.0
            },
            "strategy": "centroid",
            "zones": []
        },
        "tag": {
            "ble": 1,
            "id": "31955",
            "mac": "<tag_mac_redacted>",
            "uwb": 0
        },
        "timestamp": 1751374881169
    })",
    
    // Sample message 2: More anchors (6 used + 2 unused)
    R"({
        "is_moving": null,
        "location": {
            "dead_zones": [],
            "map_id": "<map_id_redacted>",
            "position": {
                "quality": "good",
                "unused_anchors": [
                    {"cart_d": 5.12, "id": "<anchor_id>", "mac": "<anchor_mac_redacted>", "rssi": -68.23},
                    {"cart_d": 6.45, "id": "f2c", "mac": "a1b2c3d4e5f6", "rssi": -71.88}
                ],
                "used_anchors": [
                    {"cart_d": 1.2, "id": "<anchor_id>", "mac": "<anchor_mac_redacted>", "rssi": -58.5},
                    {"cart_d": 2.1, "id": "<anchor_id>", "mac": "<anchor_mac_redacted>", "rssi": -60.12},
                    {"cart_d": 3.8, "id": "<anchor_id>", "mac": "<anchor_mac_redacted>", "rssi": -63.77},
                    {"cart_d": 4.2, "id": "7a8", "mac": "b8c9d0e1f2a3", "rssi": -65.34},
                    {"cart_d": 2.9, "id": "9e4", "mac": "f4e5d6c7b8a9", "rssi": -61.89},
                    {"cart_d": 5.1, "id": "3f7", "mac": "1a2b3c4d5e6f", "rssi": -67.15}
                ],
                "x": 7.83,
                "y": 4.56,
                "z": 0.0
            },
            "strategy": "centroid",
            "zones": []
        },
        "tag": {
            "ble": 1,
            "id": "31956",
            "mac": "a1b2c3d4e5f7",
            "uwb": 0
        },
        "timestamp": 1751374882234
    })",
    
    // Sample message 3: Many anchors (10 used + 4 unused)
    R"({
        "is_moving": null,
        "location": {
            "dead_zones": [],
            "map_id": "<map_id_redacted>",
            "position": {
                "quality": "excellent",
                "unused_anchors": [
                    {"cart_d": 6.78, "id": "<anchor_id>", "mac": "<anchor_mac_redacted>", "rssi": -72.45},
                    {"cart_d": 7.23, "id": "f2c", "mac": "a1b2c3d4e5f6", "rssi": -74.12},
                    {"cart_d": 8.91, "id": "b5d", "mac": "9f8e7d6c5b4a", "rssi": -76.89},
                    {"cart_d": 9.12, "id": "c8a", "mac": "2c3d4e5f6a7b", "rssi": -78.23}
                ],
                "used_anchors": [
                    {"cart_d": 0.8, "id": "<anchor_id>", "mac": "<anchor_mac_redacted>", "rssi": -55.2},
                    {"cart_d": 1.5, "id": "<anchor_id>", "mac": "<anchor_mac_redacted>", "rssi": -57.83},
                    {"cart_d": 2.3, "id": "<anchor_id>", "mac": "<anchor_mac_redacted>", "rssi": -59.67},
                    {"cart_d": 2.8, "id": "7a8", "mac": "b8c9d0e1f2a3", "rssi": -61.44},
                    {"cart_d": 3.2, "id": "9e4", "mac": "f4e5d6c7b8a9", "rssi": -62.78},
                    {"cart_d": 3.9, "id": "3f7", "mac": "1a2b3c4d5e6f", "rssi": -64.12},
                    {"cart_d": 4.1, "id": "8d2", "mac": "5e6f7a8b9c0d", "rssi": -65.89},
                    {"cart_d": 4.7, "id": "a6f", "mac": "c1d2e3f4a5b6", "rssi": -67.34},
                    {"cart_d": 5.2, "id": "e4b", "mac": "8b9c0d1e2f3a", "rssi": -68.91},
                    {"cart_d": 5.8, "id": "2c9", "mac": "4f5a6b7c8d9e", "rssi": -70.15}
                ],
                "x": 3.45,
                "y": 8.12,
                "z": 0.0
            },
            "strategy": "centroid",
            "zones": []
        },
        "tag": {
            "ble": 1,
            "id": "31957",
            "mac": "f7e6d5c4b3a2",
            "uwb": 0
        },
        "timestamp": 1751374883567
    })",
    
    // Sample message 4: Extreme case - many more anchors (15 used + 6 unused) 
    R"({
        "is_moving": null,
        "location": {
            "dead_zones": [],
            "map_id": "<map_id_redacted>",
            "position": {
                "quality": "excellent",
                "unused_anchors": [
                    {"cart_d": 7.89, "id": "<anchor_id>", "mac": "<anchor_mac_redacted>", "rssi": -75.12},
                    {"cart_d": 8.45, "id": "f2c", "mac": "a1b2c3d4e5f6", "rssi": -76.78},
                    {"cart_d": 9.23, "id": "b5d", "mac": "9f8e7d6c5b4a", "rssi": -78.45},
                    {"cart_d": 10.1, "id": "c8a", "mac": "2c3d4e5f6a7b", "rssi": -80.12},
                    {"cart_d": 11.2, "id": "d7f", "mac": "6e7f8a9b0c1d", "rssi": -82.34},
                    {"cart_d": 12.5, "id": "f9b", "mac": "3a4b5c6d7e8f", "rssi": -84.67}
                ],
                "used_anchors": [
                    {"cart_d": 0.7, "id": "<anchor_id>", "mac": "<anchor_mac_redacted>", "rssi": -54.1},
                    {"cart_d": 1.2, "id": "<anchor_id>", "mac": "<anchor_mac_redacted>", "rssi": -56.23},
                    {"cart_d": 1.8, "id": "<anchor_id>", "mac": "<anchor_mac_redacted>", "rssi": -58.45},
                    {"cart_d": 2.3, "id": "7a8", "mac": "b8c9d0e1f2a3", "rssi": -59.78},
                    {"cart_d": 2.9, "id": "9e4", "mac": "f4e5d6c7b8a9", "rssi": -61.12},
                    {"cart_d": 3.4, "id": "3f7", "mac": "1a2b3c4d5e6f", "rssi": -62.89},
                    {"cart_d": 3.8, "id": "8d2", "mac": "5e6f7a8b9c0d", "rssi": -64.23},
                    {"cart_d": 4.2, "id": "a6f", "mac": "c1d2e3f4a5b6", "rssi": -65.67},
                    {"cart_d": 4.7, "id": "e4b", "mac": "8b9c0d1e2f3a", "rssi": -67.12},
                    {"cart_d": 5.1, "id": "2c9", "mac": "4f5a6b7c8d9e", "rssi": -68.45},
                    {"cart_d": 5.6, "id": "6a3", "mac": "7c8d9e0f1a2b", "rssi": -69.89},
                    {"cart_d": 6.0, "id": "b8e", "mac": "0d1e2f3a4b5c", "rssi": -71.23},
                    {"cart_d": 6.4, "id": "f1a", "mac": "9a0b1c2d3e4f", "rssi": -72.67},
                    {"cart_d": 6.8, "id": "4d7", "mac": "2e3f4a5b6c7d", "rssi": -74.12},
                    {"cart_d": 7.2, "id": "9f2", "mac": "5b6c7d8e9f0a", "rssi": -75.45}
                ],
                "x": 6.78,
                "y": 1.89,
                "z": 0.0
            },
            "strategy": "centroid",
            "zones": []
        },
        "tag": {
            "ble": 1,
            "id": "31958",
            "mac": "8d9e0f1a2b3c",
            "uwb": 0
        },
        "timestamp": 1751374884892
    })"
};

// Mock anchor data for creating anchors (using example MAC addresses - actual data redacted)
std::unordered_map<std::string, PointR3> mock_anchor_positions = {
    // Example anchor positions (MAC addresses redacted)
    {"<anchor_mac_1>", std::make_tuple(0.0f, 0.0f, 2.5f)},
    {"<anchor_mac_2>", std::make_tuple(10.0f, 0.0f, 2.5f)},
    {"<anchor_mac_3>", std::make_tuple(10.0f, 8.0f, 2.5f)},
    {"<anchor_mac_4>", std::make_tuple(0.0f, 8.0f, 2.5f)},
    {"a1b2c3d4e5f6", std::make_tuple(5.0f, 4.0f, 2.5f)},
    {"b8c9d0e1f2a3", std::make_tuple(2.5f, 2.0f, 2.5f)},
    {"f4e5d6c7b8a9", std::make_tuple(7.5f, 6.0f, 2.5f)},
    {"1a2b3c4d5e6f", std::make_tuple(1.0f, 5.0f, 2.5f)},
    {"9f8e7d6c5b4a", std::make_tuple(9.0f, 1.0f, 2.5f)},
    {"2c3d4e5f6a7b", std::make_tuple(3.0f, 7.0f, 2.5f)},
    {"5e6f7a8b9c0d", std::make_tuple(6.0f, 3.0f, 2.5f)},
    {"c1d2e3f4a5b6", std::make_tuple(8.0f, 5.0f, 2.5f)},
    {"8b9c0d1e2f3a", std::make_tuple(4.0f, 1.0f, 2.5f)},
    {"4f5a6b7c8d9e", std::make_tuple(1.5f, 6.5f, 2.5f)},
    {"6e7f8a9b0c1d", std::make_tuple(8.5f, 2.5f, 2.5f)},
    {"3a4b5c6d7e8f", std::make_tuple(6.5f, 7.5f, 2.5f)},
    {"7c8d9e0f1a2b", std::make_tuple(2.0f, 4.0f, 2.5f)},
    {"0d1e2f3a4b5c", std::make_tuple(9.5f, 6.0f, 2.5f)},
    {"9a0b1c2d3e4f", std::make_tuple(3.5f, 0.5f, 2.5f)},
    {"2e3f4a5b6c7d", std::make_tuple(7.0f, 8.5f, 2.5f)},
    {"5b6c7d8e9f0a", std::make_tuple(0.5f, 3.5f, 2.5f)}
};

/**
 * @brief Create a mock anchor without requiring HTTP/API calls
 * @param anch_mac MAC address of the anchor
 * @return Unique pointer to an Anchor object
 */
std::unique_ptr<Anchor> create_mock_anchor(const std::string& anch_mac) {
    auto pos_it = mock_anchor_positions.find(anch_mac);
    if (pos_it == mock_anchor_positions.end()) {
        // Default position if not found
        PointR3 default_pos = std::make_tuple(0.0f, 0.0f, 2.5f);
        return std::make_unique<Anchor>(anch_mac, default_pos, 0.0f);
    }
    
    auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    return std::make_unique<Anchor>(anch_mac, pos_it->second, static_cast<float>(now));
}



// EXACT COPIES of functions from main.cpp for realistic testing

/**
 * @brief Extract all anchor MAC addresses from a tag position message (EXACT COPY from main.cpp)
 */
std::vector<std::string> extract_anchor_macs_from_message(const json& tag_data) {
    std::vector<std::string> anchor_macs;
    json position_data = tag_data["location"]["position"];
    
    // Get MACs from used anchors
    if (position_data.contains("used_anchors")) {
        json used_anchors = position_data["used_anchors"];
        for (const auto& anchor_dict : used_anchors) {
            anchor_macs.push_back(anchor_dict["mac"].get<std::string>());
        }
    }
    
    // Get MACs from unused anchors
    if (position_data.contains("unused_anchors")) {
        json unused_anchors = position_data["unused_anchors"];
        for (const auto& anchor_dict : unused_anchors) {
            anchor_macs.push_back(anchor_dict["mac"].get<std::string>());
        }
    }
    
    // Remove duplicates
    std::sort(anchor_macs.begin(), anchor_macs.end());
    anchor_macs.erase(std::unique(anchor_macs.begin(), anchor_macs.end()), anchor_macs.end());
    
    return anchor_macs;
}

/**
 * @brief Create a Tag object from MQTT message data (EXACT COPY from main.cpp)
 */
Tag create_tag_class(const json& tag_data) {
    // Get MAC address
    std::string tag_mac = tag_data["tag"]["mac"].get<std::string>();
    
    // Get position
    json position_data = tag_data["location"]["position"];
    float x = position_data["x"].get<float>();
    float y = position_data["y"].get<float>();
    float z = position_data["z"].get<float>();
    PointR3 tag_pos = std::make_tuple(x, y, z);
    
    // Get RSSI dictionary
    std::unordered_map<std::string, float> tag_rssi_dict;
    
    if (position_data.contains("used_anchors")) {
        json used_anchors = position_data["used_anchors"];
        for (const auto& anchor_dict : used_anchors) {
            std::string amac = anchor_dict["mac"].get<std::string>();
            float arssi = anchor_dict["rssi"].get<float>();
            tag_rssi_dict[amac] = arssi;
        }
    }
    
    return Tag(tag_mac, tag_pos, tag_rssi_dict);
}

/**
 * @brief Create tag info structure for output message (EXACT COPY from main.cpp)
 */
json create_tag_info(const std::string& tag_mac, float error_estimate) {
    return json{
        {"tag_mac", tag_mac},
        {"error_estimate", error_estimate}
    };
}

/**
 * @brief Create anchors info structure for output message (EXACT COPY from main.cpp)
 */
json create_anchors_info(const std::vector<Anchor*>& anch_list) {
    std::vector<std::string> warning_anchors;
    std::vector<std::string> faulty_anchors;
    json anchors_info_list = json::array();
    
    for (const auto& anchor : anch_list) {
        json anch_info_dict = {
            {"mac", anchor->get_mac_address()},
            {"n_var", anchor->get_n()},
            {"ewma", anchor->get_ewma()}
        };
        
        anchors_info_list.push_back(anch_info_dict);
        
        if (anchor->is_warning()) {
            warning_anchors.push_back(anchor->get_mac_address());
        }
        
        if (anchor->is_faulty()) {
            faulty_anchors.push_back(anchor->get_mac_address());
        }
    }
    
    return json{
        {"anchors_selected_for_estimation", anchors_info_list},
        {"warning_anchors", warning_anchors},
        {"faulty_anchors", faulty_anchors}
    };
}

/**
 * @brief Create complete output info combining tag and anchors data (EXACT COPY from main.cpp)
 */
json create_output_info(const std::string& tag_mac, float error_estimate, const std::vector<Anchor*>& anch_list) {
    json tag_return = create_tag_info(tag_mac, error_estimate);
    json anchors_return = create_anchors_info(anch_list);
    
    // Merge the two JSON objects
    json result = tag_return;
    for (auto& [key, value] : anchors_return.items()) {
        result[key] = value;
    }
    
    return result;
}

/**
 * @brief Mock implementation of the core MQTT message processing logic
 * This simulates the processing done in the on_message callback without MQTT dependencies
 */
std::chrono::microseconds process_mqtt_message_mock(const std::string& payload, MockMQTTUserData& userdata, bool debug_mode = false) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        // Parse JSON message (same as in main.cpp)
        json tag_data = json::parse(payload);
        
        if (debug_mode) {
            std::cout << "\n=== DEBUG: INPUT MQTT MESSAGE ===" << std::endl;
            std::cout << tag_data.dump(2) << std::endl; // Pretty print with 2-space indent
        }
        
        // Initialize anchors if first message
        if (!userdata.anchors_initialized) {
            std::vector<std::string> discovered_anchor_macs = extract_anchor_macs_from_message(tag_data);
            
            // Create all discovered anchors using mock data
            for (const auto& anch_mac : discovered_anchor_macs) {
                userdata.anchors[anch_mac] = create_mock_anchor(anch_mac);
            }
            userdata.anchors_initialized = true;
        }
        
        // Create Tag object from message (same as in main.cpp)
        Tag message_tag = create_tag_class(tag_data);
        float timestamp = tag_data["timestamp"].get<float>();
        
        if (debug_mode) {
            std::cout << "\n=== DEBUG: PARSED TAG INFO ===" << std::endl;
            std::cout << "Tag MAC: " << message_tag.get_mac_address() << std::endl;
            PointR3 tag_pos = message_tag.get_est_coord();
            std::cout << "Position: (" << std::get<0>(tag_pos) << ", " 
                      << std::get<1>(tag_pos) << ", " << std::get<2>(tag_pos) << ")" << std::endl;
            std::cout << "Timestamp: " << timestamp << std::endl;
            std::cout << "RSSI readings count: " << message_tag.get_rssi_readings().size() << std::endl;
            
            std::cout << "\nRSSI readings:" << std::endl;
            for (const auto& [mac, rssi] : message_tag.get_rssi_readings()) {
                std::cout << "  " << mac << ": " << rssi << " dBm" << std::endl;
            }
        }
        
        // Create vector of anchor pointers for anchors with RSSI readings
        std::vector<Anchor*> anch_list;
        const auto& rssi_readings = message_tag.get_rssi_readings();
        
        for (const auto& [anch_mac, rssi_val] : rssi_readings) {
            auto anch_it = userdata.anchors.find(anch_mac);
            if (anch_it != userdata.anchors.end()) {
                anch_list.push_back(anch_it->second.get());
            } else {
                // Handle new anchor discovered after initialization
                userdata.anchors[anch_mac] = create_mock_anchor(anch_mac);
                anch_list.push_back(userdata.anchors[anch_mac].get());
            }
        }
        
        // Core processing (same as in main.cpp)
        if (!anch_list.empty()) {
            if (debug_mode) {
                std::cout << "\n=== DEBUG: ANCHOR PROCESSING ===" << std::endl;
                std::cout << "Processing " << anch_list.size() << " anchors with RSSI readings:" << std::endl;
                for (const auto& anchor : anch_list) {
                    PointR3 anchor_pos = anchor->get_coord();
                    std::cout << "  Anchor " << anchor->get_mac_address() 
                              << " at (" << std::get<0>(anchor_pos) << ", " 
                              << std::get<1>(anchor_pos) << ", " << std::get<2>(anchor_pos) << ")" << std::endl;
                }
            }
            
            // Create TagSystem
            TagSystem message_system(message_tag, userdata.model);
            
            // Get error estimate (this is the main computation)
            float error_estimate = message_system.error_radius(anch_list);
            
            if (debug_mode) {
                std::cout << "\n=== DEBUG: ERROR CALCULATION ===" << std::endl;
                std::cout << "Calculated error estimate: " << error_estimate << " meters" << std::endl;
            }
            
            // Simulate anchor health updates (without actual updates to avoid side effects in testing)
            // In real code: update_anchors_from_tag_data(anch_list, message_tag, userdata.model, timestamp, Config::DEFAULT_DELTA_R, Config::DEFAULT_T_VIS);
            
            // Create output using the actual function from main.cpp
            json output_msg = create_output_info(message_tag.get_mac_address(), error_estimate, anch_list);
            std::string output_str = output_msg.dump();
            
            if (debug_mode) {
                std::cout << "\n=== DEBUG: OUTPUT MQTT MESSAGE ===" << std::endl;
                std::cout << output_msg.dump(2) << std::endl; // Pretty print with 2-space indent
            }
            
            // Simulate publishing latency (minimal string operation)
            volatile size_t output_len = output_str.length();
            (void)output_len; // Suppress unused variable warning
        } else {
            if (debug_mode) {
                std::cout << "\n=== DEBUG: WARNING ===" << std::endl;
                std::cout << "No anchors with RSSI readings found for processing!" << std::endl;
            }
        }
        
    } catch (const json::parse_error& e) {
        std::cerr << "JSON parse error in test: " << e.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error processing test message: " << e.what() << std::endl;
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
}

/**
 * @brief Test MQTT message processing performance
 */
bool test_mqtt_processing_performance() {
    std::cout << "Testing MQTT message processing performance..." << std::endl;
    
    MockMQTTUserData userdata;
    std::vector<std::chrono::microseconds> processing_times;
    
    // Process each sample message multiple times
    const int iterations_per_message = 100;
    const int target_time_us = 1000; // 1ms target
    
    for (size_t msg_idx = 0; msg_idx < sample_mqtt_messages.size(); msg_idx++) {
        std::cout << "  Testing message " << (msg_idx + 1) << "/" << sample_mqtt_messages.size() << "..." << std::endl;
        
        // Run first iteration with debug mode to show JSON format
        if (msg_idx == 0) {
            std::cout << "\n🔍 DEBUG MODE: Showing first message processing details..." << std::endl;
            auto debug_time = process_mqtt_message_mock(sample_mqtt_messages[msg_idx], userdata, true);
            processing_times.push_back(debug_time);
            std::cout << "🔍 DEBUG MODE: First message processing took " << debug_time.count() << "us" << std::endl;
            std::cout << "========================================" << std::endl;
        }
        
        for (int iter = 0; iter < iterations_per_message; iter++) {
            auto processing_time = process_mqtt_message_mock(sample_mqtt_messages[msg_idx], userdata, false);
            processing_times.push_back(processing_time);
        }
    }
    
    // Calculate statistics
    auto min_time = *std::min_element(processing_times.begin(), processing_times.end());
    auto max_time = *std::max_element(processing_times.begin(), processing_times.end());
    
    long long total_time = 0;
    for (const auto& time : processing_times) {
        total_time += time.count();
    }
    double avg_time = static_cast<double>(total_time) / processing_times.size();
    
    // Calculate 95th percentile
    std::sort(processing_times.begin(), processing_times.end());
    size_t p95_index = static_cast<size_t>(0.95 * processing_times.size());
    auto p95_time = processing_times[p95_index];
    
    // Calculate 99th percentile
    size_t p99_index = static_cast<size_t>(0.99 * processing_times.size());
    auto p99_time = processing_times[p99_index];
    
    // Print results
    std::cout << "\n=== MQTT Processing Performance Results ===" << std::endl;
    std::cout << "Total measurements: " << processing_times.size() << std::endl;
    std::cout << "Target time: <" << target_time_us << "us (<1ms)" << std::endl;
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "Minimum time: " << min_time.count() << "us" << std::endl;
    std::cout << "Average time: " << avg_time << "us" << std::endl;
    std::cout << "Maximum time: " << max_time.count() << "us" << std::endl;
    std::cout << "95th percentile: " << p95_time.count() << "us" << std::endl;
    std::cout << "99th percentile: " << p99_time.count() << "us" << std::endl;
    
    // Check performance criteria
    int violations = 0;
    for (const auto& time : processing_times) {
        if (time.count() > target_time_us) {
            violations++;
        }
    }
    
    double violation_rate = 100.0 * violations / processing_times.size();
    std::cout << "Target violations: " << violations << "/" << processing_times.size() 
              << " (" << std::setprecision(2) << violation_rate << "%)" << std::endl;
    
    // Performance thresholds
    bool avg_performance_ok = avg_time < target_time_us;
    bool p95_performance_ok = p95_time.count() < target_time_us;
    bool violation_rate_ok = violation_rate < 5.0; // Allow up to 5% violations
    
    std::cout << "\nPerformance Assessment:" << std::endl;
    std::cout << "  Average < " << target_time_us << "us: " << (avg_performance_ok ? "✓ PASS" : "✗ FAIL") << std::endl;
    std::cout << "  95th percentile < " << target_time_us << "us: " << (p95_performance_ok ? "✓ PASS" : "✗ FAIL") << std::endl;
    std::cout << "  Violation rate < 5%: " << (violation_rate_ok ? "✓ PASS" : "✗ FAIL") << std::endl;
    
    return avg_performance_ok && p95_performance_ok && violation_rate_ok;
}

/**
 * @brief Test with different message sizes and complexity
 */
bool test_message_size_impact() {
    std::cout << "\nTesting impact of message size and complexity..." << std::endl;
    
    // Test with different message sizes and complexity using real format
    std::vector<std::pair<std::string, std::string>> test_cases = {
        {"Small message (3 used + 1 unused)", sample_mqtt_messages[0]},
        {"Medium message (6 used + 2 unused)", sample_mqtt_messages[1]}, 
        {"Large message (10 used + 4 unused)", sample_mqtt_messages[2]},
        {"Extra large message (15 used + 6 unused)", sample_mqtt_messages[3]}
    };
    
    for (const auto& [test_name, message_payload] : test_cases) {
        MockMQTTUserData userdata;
        
        const int iterations = 50;
        std::vector<std::chrono::microseconds> times;
        
        // Show debug info for the first test case
        bool show_debug = (test_name.find("Small message") != std::string::npos);
        
        if (show_debug) {
            std::cout << "\n🔍 COMPLEXITY DEBUG: " << test_name << std::endl;
            auto debug_time = process_mqtt_message_mock(message_payload, userdata, true);
            times.push_back(debug_time);
            std::cout << "========================================" << std::endl;
        }
        
        for (int i = 0; i < iterations; i++) {
            auto time = process_mqtt_message_mock(message_payload, userdata, false);
            times.push_back(time);
        }
        
        // Calculate average
        long long total = 0;
        for (const auto& time : times) {
            total += time.count();
        }
        double avg = static_cast<double>(total) / times.size();
        
        std::cout << "  " << test_name << ": " << std::setprecision(1) << avg << "us average" << std::endl;
    }
    
    return true;
}

/**
 * @brief Run all performance tests
 */
int main() {
    std::cout << "BLE RSSI MQTT Processing Performance Test Suite" << std::endl;
    std::cout << "==============================================" << std::endl;
    
    bool all_passed = true;
    
    // Run main performance test
    all_passed &= test_mqtt_processing_performance();
    
    // Run message size impact test
    all_passed &= test_message_size_impact();
    
    std::cout << "\n==============================================" << std::endl;
    if (all_passed) {
        std::cout << "🎉 ALL PERFORMANCE TESTS PASSED! 🎉" << std::endl;
        std::cout << "MQTT message processing meets <1ms target." << std::endl;
        return 0;
    } else {
        std::cout << "⚠️  PERFORMANCE TESTS FAILED ⚠️" << std::endl;
        std::cout << "MQTT message processing exceeds 1ms target." << std::endl;
        return 1;
    }
}