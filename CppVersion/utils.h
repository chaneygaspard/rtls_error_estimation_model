#pragma once

#include <tuple>
#include <vector>

//type constructor
using PointR3 = std::tuple<float, float, float>;

/**
 * @brief Calculate the Euclidean distance between two 3D points
 * 
 * @param a First 3D point as (x, y, z) coordinates
 * @param b Second 3D point as (x, y, z) coordinates
 * @return float Euclidean distance between points a and b
 */
float R3_distance(const PointR3& a, const PointR3& b);

/**
 * @brief Calculate the log probability density function of the Student's t-distribution
 * 
 * Computes log(pdf(z)) for a Student's t-distribution with v degrees of freedom.
 * Uses the formula: log(Γ((v+1)/2)) - log(Γ(v/2)) - 0.5*log(v*π) - ((v+1)/2)*log(1 + z²/v)
 * 
 * @param z The value at which to evaluate the log pdf
 * @param v Degrees of freedom (default: 5)
 * @return float Log probability density value
 */
float logpdf_student_t(float z, int v = 5);

/**
 * @brief Derive 95% confidence radius from probability confidence value
 * 
 * Uses linear interpolation on a lookup table to convert confidence probability
 * values to 95% confidence error probabilities (CEP95). The lookup table contains
 * empirically derived values for specific confidence levels.
 * 
 * @param p_conf Probability confidence value (between 0.0 and 1.0)
 * @return float 95% confidence error probability radius
 */
float cep95_from_conf(float p_conf);