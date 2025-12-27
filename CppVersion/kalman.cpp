#include <cmath>
#include <numeric>
#include "kalman.h"

//constructor
KalmanFilter::KalmanFilter() {}

//methods
float KalmanFilter::computeResidualVariance() {
    if (residuals.size() < min_required_points) return std::pow(0.0025f, 2.0f);
    float mean = std::accumulate(residuals.begin(), residuals.end(), 0.0f) / residuals.size();
    float var = 0.0f;
    for (float r : residuals) var += std::pow(r - mean, 2.0f);
    return var / residuals.size();
}

float KalmanFilter::computeRSSIStdDev() {
    if (rssi_vals.size() < min_required_points) return 4.0f;
    float mean = std::accumulate(rssi_vals.begin(), rssi_vals.end(), 0.0f) / rssi_vals.size();
    float var = 0.0f;
    for (float r : rssi_vals) var += std::pow(r - mean, 2.0f);
    return std::sqrt(var / rssi_vals.size());
}

std::tuple<float, float> KalmanFilter::sequence_step(float RSSI0_i, float n_i, 
    float r_val, float d_val){

    // x_ji designates x{i+1|i},  x{i+1|i+1} is designated by x_jj
    std::array<float, 2> x_ji = {RSSI0_i, n_i}; 

    // Store RSSI and trim
    rssi_vals.push_back(r_val);
    if (rssi_vals.size() > max_buffer) rssi_vals.erase(rssi_vals.begin());
    
    // Update sigma based on RSSI std dev (adaptive) - only if we have enough data
    if (rssi_vals.size() >= min_required_points) {
        sigma = beta * computeRSSIStdDev();
    }

    // Update Q based on residual variance (adaptive) - only if we have enough data
    if (residuals.size() >= min_required_points) {
        float resid_var = computeResidualVariance();
        Q[0][0] = alpha * resid_var;      // process noise for RSSI0
        Q[1][1] = alpha * (resid_var / 100.0f); // smaller scale for n
    }

    //P{i+1|i} = P{i|i} + Q
    P[0][0] += Q[0][0];
    P[0][1] += Q[0][1]; 
    P[1][0] += Q[1][0];
    P[1][1] += Q[1][1];

    //vect H = [1 X] in R^{1*2}
    float safe_d_val = std::max(d_val, 1e-6f); 
    float X = (-10) * std::log10(safe_d_val / d_0);
    std::array<float, 2> H = {1.0f, X};

    //predicted r_val & residual
    float r_predict = H[0] * x_ji[0] + H[1] * x_ji[1];
    float resid = r_val - r_predict;
    
    // Store residual and trim buffer
    residuals.push_back(resid);
    if (residuals.size() > max_buffer) residuals.erase(residuals.begin());

    //S val & K matrices
    float S = 
        H[0] * (P[0][0] * H[0] + P[0][1] * H[1]) + 
        H[1] * (P[1][0] * H[0] + P[1][1] * H[1]) + 
        std::pow(sigma, 2);
    
    std::array<float, 2> K = {
        (P[0][0] * H[0] + P[0][1] * H[1]) / S,
        (P[1][0] * H[0] + P[1][1] * H[1]) / S
    };

    // x_jj, P_jj
    std::array<float, 2> x_jj = {
        x_ji[0] + K[0] * resid, 
        x_ji[1] + K[1] * resid
    };

    float KH00 = K[0] * H[0];
    float KH01 = K[0] * H[1];
    float KH10 = K[1] * H[0];
    float KH11 = K[1] * H[1];

    std::array<std::array<float, 2>, 2> I_minus_KH = {{
        {1.0f - KH00,    -KH01},
        {   -KH10,    1.0f - KH11}
    }};

    std::array<std::array<float, 2>, 2> newP;

    newP[0][0] = I_minus_KH[0][0]*P[0][0] + I_minus_KH[0][1]*P[1][0];
    newP[0][1] = I_minus_KH[0][0]*P[0][1] + I_minus_KH[0][1]*P[1][1];
    newP[1][0] = I_minus_KH[1][0]*P[0][0] + I_minus_KH[1][1]*P[1][0];
    newP[1][1] = I_minus_KH[1][0]*P[0][1] + I_minus_KH[1][1]*P[1][1];
    P = newP;

    //output
    return std::make_tuple(x_jj[0], x_jj[1]);
}