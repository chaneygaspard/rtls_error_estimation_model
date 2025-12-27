#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <tuple>
#include <cmath>

#include "utils.h"
#include "kalman.h"
#include "config.h"

//Anchor class
class Anchor {
    private:
        std::string mac_address;
        PointR3 coord;
        float ewma = 1.0;
        float last_seen = 0.0;
        float RSSI_0 = -59.0;
        float n = 2.0;
        KalmanFilter kalman = KalmanFilter();

    public:
        /**
         * @brief Construct a new Anchor object representing a BLE beacon
         * 
         * Initializes an anchor with its MAC address and 3D coordinates. Default values
         * are set for RSSI parameters (RSSI_0 = -59 dBm, n = 2.0), health monitoring
         * (ewma = 1.0), and a fresh Kalman filter for parameter estimation.
         * 
         * @param mac MAC address identifier for the anchor
         * @param coordinate 3D position (x, y, z) of the anchor in meters
         * @param timestamp Initial timestamp for when the anchor was last seen
         */
        Anchor(std::string mac, PointR3 coordinate, float timestamp);

        /**
         * @brief Gets the MAC address identifier of the anchor
         * @return std::string MAC address string
         */
        std::string get_mac_address() const;
        
        /**
         * @brief Gets the 3D coordinates of the anchor
         * @return PointR3 3D position (x, y, z) in meters
         */
        PointR3 get_coord() const;
        
        /**
         * @brief Gets the current EWMA health metric
         * @return float EWMA value (higher = less reliable)
         */
        float get_ewma() const;
        
        /**
         * @brief Gets the timestamp when anchor was last seen
         * @return float Timestamp in epoch time
         */
        float get_last_seen() const;
        
        /**
         * @brief Gets the RSSI at 1 meter parameter
         * @return float RSSI_0 value in dBm
         */
        float get_RSSI_0() const;
        
        /**
         * @brief Gets the path loss exponent parameter
         * @return float Path loss exponent n
         */
        float get_n() const;
        
        /**
         * @brief Gets a constant reference to the Kalman filter
         * @return const KalmanFilter& Reference to the internal Kalman filter
         */
        const KalmanFilter& get_kalman() const;

        /**
         * @brief Update the health monitoring metrics for this anchor
         * 
         * Uses an Exponentially Weighted Moving Average (EWMA) to track the health
         * of the anchor based on measurement residuals. Higher residuals indicate
         * potential anchor issues (movement, interference, failure).
         * 
         * @param z Measurement residual (difference between predicted and observed RSSI)
         * @param now Current timestamp (epoch time)
         * @param LAMBDA Smoothing factor for EWMA (default: 0.05, smaller = more smoothing)
         */
        void update_health(float z, float now, float LAMBDA = Calibration::LAMBDA_EWMA);
        
        /**
         * @brief Update RSSI propagation parameters using Kalman filter estimation
         * 
         * Uses the anchor's Kalman filter to refine estimates of RSSI_0 (signal strength
         * at 1 meter) and n (path loss exponent) based on new RSSI and distance measurements.
         * This enables adaptive calibration as environmental conditions change.
         * 
         * @param measured_rssi Observed RSSI value in dBm
         * @param estimated_distance Estimated distance to the measurement point in meters
         */
        void update_parameters(float measured_rssi, float estimated_distance);
        
        /**
         * @brief Check if anchor is in warning state based on health metrics
         * 
         * Warning state indicates moderate measurement inconsistencies that may
         * require attention but don't warrant excluding the anchor from positioning.
         * 
         * @return bool true if EWMA is between 4 and 8 (exclusive), false otherwise
         */
        bool is_warning();
        
        /**
         * @brief Check if anchor is in faulty state based on health metrics
         * 
         * Faulty state indicates severe measurement inconsistencies suggesting
         * the anchor should be excluded from positioning calculations and checked for displacement.
         * 
         * @return bool true if EWMA is 8 or higher, false otherwise
         */
        bool is_faulty();
        
        // Equality operator for use as map key
        bool operator==(const Anchor& other) const {
            return mac_address == other.mac_address;
        }
};

// Hash function for Anchor* to use as unordered_map key
namespace std {
    template<>
    struct hash<Anchor*> {
        size_t operator()(const Anchor* anchor) const {
            return hash<string>()(anchor->get_mac_address());
        }
    };
    
    template<>
    struct equal_to<Anchor*> {
        bool operator()(const Anchor* lhs, const Anchor* rhs) const {
            return lhs->get_mac_address() == rhs->get_mac_address();
        }
    };
}

//Tag class
class Tag {
    private:
        std::string mac_address;
        PointR3 est_coord;
        std::unordered_map<std::string, float> rssi_readings;
    
    public:
        /**
         * @brief Construct a new Tag object representing a mobile device being tracked
         * 
         * Initializes a tag with its MAC address, estimated position, and a map of
         * RSSI readings from various anchors. Used in indoor positioning systems
         * to track mobile devices using BLE signal measurements.
         * 
         * @param mac MAC address identifier for the tag
         * @param coord Estimated 3D position (x, y, z) of the tag in meters
         * @param rssi_map Map of anchor MAC addresses to their RSSI readings
         */
        Tag(std::string mac, PointR3 coord, std::unordered_map<std::string, float> rssi_map);

        /**
         * @brief Gets the MAC address identifier of the tag
         * @return std::string MAC address string
         */
        std::string get_mac_address() const;
        
        /**
         * @brief Gets the estimated 3D coordinates of the tag
         * @return PointR3 Estimated 3D position (x, y, z) in meters
         */
        PointR3 get_est_coord() const;
        
        /**
         * @brief Gets a constant reference to the RSSI readings map
         * @return const std::unordered_map<std::string, float>& Map of anchor MAC to RSSI values
         */
        const std::unordered_map<std::string, float>& get_rssi_readings() const;

        /**
         * @brief Retrieve RSSI reading for a specific anchor
         * 
         * Returns the measured RSSI value (in dBm) from the specified anchor.
         * Throws std::out_of_range if the anchor MAC address is not found.
         * 
         * @param anchor_mac MAC address of the anchor to query
         * @return float RSSI value in dBm for the specified anchor
         * @throws std::out_of_range if anchor_mac not found in rssi_readings
         */
        float rssi_for_anchor(std::string anchor_mac);
        
        /**
         * @brief Get list of all anchor MAC addresses that provided RSSI readings
         * 
         * Returns a vector containing the MAC addresses of all anchors from which
         * this tag has received RSSI measurements. Useful for determining which
         * anchors are in communication range.
         * 
         * @return std::vector<std::string> Vector of anchor MAC addresses as strings
         */
        std::vector<std::string> anchors_included();
};

class PathLossModel {
    private:
        float d_0 = 1.0;
        float sigma = 4.0;

    public:
        /**
         * @brief Construct a new PathLossModel object for RSSI-distance modeling
         * 
         * Initializes with default parameters: d_0 = 1.0 meter (reference distance)
         * and sigma = 4.0 dB (measurement noise standard deviation). These values
         * are typical for indoor BLE environments.
         */
        PathLossModel();

        /**
         * @brief Gets the reference distance parameter
         * @return float Reference distance d_0 in meters
         */
        float get_d_0() const;
        
        /**
         * @brief Gets the measurement noise standard deviation
         * @return float Sigma value in dB
         */
        float get_sigma() const;

        /**
         * @brief Calculate expected RSSI using the log-distance path loss model
         * 
         * Computes the theoretical RSSI value at a given distance using the formula:
         * RSSI = RSSI_0 - 10*n*log10(d/d_0)
         * 
         * This represents the mean (expected value) of RSSI measurements before
         * adding measurement noise.
         * 
         * @param RSSI_0 Signal strength at reference distance (1 meter) in dBm
         * @param n Path loss exponent (typically 2.0-4.0 for indoor environments)
         * @param est_dist Estimated distance in meters
         * @return float Expected RSSI value in dBm
         */
        float mu(float RSSI_0, float n, float est_dist);
        
        /**
         * @brief Calculate standardized residual (z-score) for RSSI measurement
         * 
         * Computes the normalized difference between observed and expected RSSI:
         * z = (observed - expected) / sigma
         * 
         * Used for statistical analysis, outlier detection, and measurement validation.
         * Values beyond ±2 typically indicate potential measurement issues.
         * 
         * @param rssi_freq Observed RSSI measurement in dBm
         * @param RSSI_0 Signal strength at reference distance in dBm
         * @param n Path loss exponent
         * @param est_dist Estimated distance in meters
         * @return float Standardized residual (dimensionless)
         */
        float z(float rssi_freq, float RSSI_0, float n, float est_dist);
};


