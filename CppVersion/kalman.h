#pragma once 

#include <array>
#include <cmath>
#include <tuple>
#include <vector>


class KalmanFilter {
    private:
        std::array<std::array<float, 2>, 2> Q = {{
            {std::pow(0.0025f, 2.0f), 0.0f}, 
            {0.0f, std::pow(0.0001f, 2.0f)}
        }};
        std::array<std::array<float, 2>, 2> P = {{
            {1.0f, 0.0f}, 
            {0.0f, 0.1f}
        }};
        float d_0 = 1.0;
        float sigma = 4.0;
        
        // Adaptive filtering parameters
        std::vector<float> residuals;           // Store residuals for variance computation
        std::vector<float> rssi_vals;           // Store RSSI values for std dev computation
        const size_t min_required_points = 5;  // Minimum points needed for statistics
        const size_t max_buffer = 50;          // Maximum buffer size for stored values
        const float alpha = 0.1f;              // Process noise adaptation factor
        const float beta = 0.8f;               // RSSI std dev scaling factor
    
    public:
        /**
         * @brief Default constructor for KalmanFilter
         * 
         * Initializes the Kalman filter with default process noise matrix Q,
         * initial covariance matrix P, reference distance d_0, and measurement noise sigma.
         */
        KalmanFilter();


        float computeResidualVariance();
        float computeRSSIStdDev();

        /**
         * @brief Perform one step of the Kalman filter for RSSI-based distance estimation
         * 
         * This method implements a Kalman filter to estimate and update the RSSI parameters
         * (RSSI0 and path loss exponent n) based on new RSSI and distance measurements.
         * It uses the log-distance path loss model: RSSI = RSSI0 + n * (-10) * log10(d/d0)
         * 
         * The filter maintains and updates:
         * - State vector: [RSSI0, n] representing signal strength at 1m and path loss exponent
         * - Covariance matrix P: uncertainty in the state estimates
         * 
         * Algorithm steps:
         * 1. Predict: Update covariance with process noise Q
         * 2. Measure: Compare predicted vs actual RSSI measurement
         * 3. Update: Correct state estimates using Kalman gain
         * 4. Update: Correct covariance matrix
         * 
         * @param RSSI0_i Current estimate of RSSI at 1 meter (dBm)
         * @param n_i Current estimate of path loss exponent 
         * @param r_val Measured RSSI value (dBm)
         * @param d_val Measured distance (meters)
         * @return std::tuple<float, float> Updated (RSSI0, n) estimates
         */
        std::tuple<float, float> sequence_step(float RSSI0_i, float n_i, float r_val, float d_val);
        
        // getters
        /**
         * @brief Get current Q[0][0] value (process noise for RSSI0)
         * @return float Current Q[0][0] value
         */
        float get_Q_00() const { return Q[0][0]; }
        
        /**
         * @brief Get current Q[1][1] value (process noise for n)
         * @return float Current Q[1][1] value
         */
        float get_Q_11() const { return Q[1][1]; }
        
        /**
         * @brief Get current sigma value (measurement noise)
         * @return float Current sigma value
         */
        float get_sigma() const { return sigma; }
        
        /**
         * @brief Get number of stored residuals
         * @return size_t Number of residuals in buffer
         */
        size_t get_residuals_count() const { return residuals.size(); }
        
        /**
         * @brief Get number of stored RSSI values
         * @return size_t Number of RSSI values in buffer
         */
        size_t get_rssi_count() const { return rssi_vals.size(); }
};