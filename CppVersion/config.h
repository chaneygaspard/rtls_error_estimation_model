#pragma once

#include <string>
#include <array>

/**
 * @brief Configuration constants for the BLE RSSI positioning system
 * 
 * This file centralizes all configuration settings for easy modification
 * without changing the main application code.
 */

// Input environment
namespace ConfigInput {
    const std::string BROKER = "";  // MQTT broker address (redacted)
    const int PORT = 0;  // MQTT broker port (redacted)
    const std::string TOPIC = "";  // MQTT topic pattern (redacted)
    const std::string CLIENT_ID = "";  // MQTT client identifier (redacted)
    // API for input
    const std::string ANCHOR_INIT_BASE = "";  // API endpoint (redacted)
    const std::string API_USERNAME = "";  // API username (redacted)
    const std::string API_PASSWORD = "";  // API password (redacted)
}

// Output environment
namespace ConfigOutput {
    const std::string BROKER = "";  // MQTT broker address (redacted)
    const int PORT = 0;  // MQTT broker port (redacted)
    const std::string TOPIC = "";  // MQTT topic for output (redacted)
    const std::string CLIENT_ID = "";  // MQTT client identifier (redacted)
}

// Legacy config for compatibility (can be removed after refactor)
namespace Config {
    const std::string BROKER = ConfigOutput::BROKER;
    const int PORT = ConfigOutput::PORT;
    const std::string CLIENT_ID = ConfigOutput::CLIENT_ID;
    const std::string TAG_POSITION_STREAM = "";  // MQTT topic for tag positions (redacted)
    const std::string TOPIC_OUT = ConfigOutput::TOPIC;
    const std::string ANCHOR_INIT_BASE = ConfigInput::ANCHOR_INIT_BASE;
    const std::string API_USERNAME = ConfigInput::API_USERNAME;
    const std::string API_PASSWORD = ConfigInput::API_PASSWORD;
    const int STALE_T_SEC = 15 * 60;
    const int MQTT_KEEPALIVE = 60;
    const float DEFAULT_DELTA_R = 12.0f;
    const int DEFAULT_T_VIS = 6000;
    const bool ENABLE_DEBUG_LOGGING = true;
    const bool ENABLE_MQTT_LOGGING = false;
    const int MAX_ANCHORS_PER_TAG = 10;
    const int HTTP_TIMEOUT_SEC = 30;
    // Performance logging
    const bool ENABLE_PERFORMANCE_LOGGING = true;
    const int MAX_PROCESSING_TIME_MS = 2;
}

// Calibration Constants
// All algorithm tuning parameters centralized for easy adjustment
namespace Calibration {
    // === Anchor Selection Parameters ===
    const int MAX_SIGNIFICANT_ANCHORS = 5;         // Maximum number of anchors to use for positioning
    const float EWMA_THRESHOLD = 8.0f;             // EWMA health threshold for anchor filtering
    const float LAMBDA_EWMA = 0.05f;               // EWMA decay factor for anchor health monitoring
    
    // === Signal Processing Parameters ===
    const int STUDENT_T_DEGREES_OF_FREEDOM = 5;    // Degrees of freedom for Student's t-distribution
    const float RSSI_SIGNAL_STRENGTH_THRESHOLD = 10.0f; // dB threshold for signal strength filtering
    
    // === Path Loss Model Parameters ===
    const float DEFAULT_PATH_LOSS_EXPONENT = 2.0f; // Default path loss exponent (n)
    const float DEFAULT_RSSI0 = -59.0f;            // Default RSSI at 1 meter reference distance
    
    // === Kalman Filter Parameters ===
    const float KALMAN_PROCESS_NOISE_Q = 1.0f;     // Process noise covariance
    const float KALMAN_MEASUREMENT_NOISE_R = 1.0f; // Measurement noise covariance
    const float KALMAN_INITIAL_P = 10.0f;          // Initial error covariance
    
    // === CEP95 Confidence-to-Radius Mapping ===
    // Lookup table for converting confidence scores to CEP95 error radii
    const std::array<std::pair<float, float>, 8> CEP95_TABLE = {{
        {0.05f, 7.4f},  // 5% confidence -> 7.4m radius
        {0.17f, 6.1f},  // 17% confidence -> 6.1m radius
        {0.43f, 4.3f},  // 43% confidence -> 4.3m radius
        {0.80f, 2.5f},  // 80% confidence -> 2.5m radius
        {0.85f, 2.0f},  // 85% confidence -> 2.0m radius
        {0.90f, 1.6f},  // 90% confidence -> 1.6m radius
        {0.95f, 1.2f},  // 95% confidence -> 1.2m radius
        {0.98f, 0.9f}   // 98% confidence -> 0.9m radius
    }};
    
    // === Boundary Values ===
    const float MAX_CEP95_RADIUS = 8.0f;           // Maximum allowed CEP95 error radius (meters)
    const float MIN_CONFIDENCE_SCORE = 0.0f;       // Minimum confidence score
    const float MAX_CONFIDENCE_SCORE = 1.0f;       // Maximum confidence score
}