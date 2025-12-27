#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <chrono>

#include "models.h"   
#include "config.h"
#include "utils.h"    

// Note: EWMA_THRESHOLD is now defined in config.h under Calibration::EWMA_THRESHOLD

/**
 * @brief System for analyzing tag-anchor relationships and calculating positioning metrics
 * 
 * The TagSystem class encapsulates a single tag and path loss model to perform various
 * calculations related to anchor selection, distance estimation, and confidence scoring.
 * It works with pointer-based containers for efficient memory usage and direct modification
 * of original anchor objects.
 */
class TagSystem {
    private:
        Tag tag; 
        PathLossModel model;
    
    public:
        /**
         * @brief Constructs a TagSystem with the given tag and path loss model
         * @param inpt_tag The tag object containing position and RSSI readings
         * @param inpt_model The path loss model used for signal propagation calculations
         */
        TagSystem(const Tag& inpt_tag, const PathLossModel& inpt_model);

        /**
         * @brief Gets a constant reference to the encapsulated tag
         * @return const Tag& Reference to the tag object
         */
        const Tag& get_tag() const;
        
        /**
         * @brief Gets a constant reference to the encapsulated path loss model
         * @return const PathLossModel& Reference to the path loss model
         */
        const PathLossModel& get_model() const;

        /**
         * @brief Filters anchors to find the most significant ones for positioning
         * 
         * Selects anchors based on:
         * - Presence in tag's RSSI readings
         * - RSSI within 10dB of strongest signal
         * - EWMA health metric below threshold
         * Results are sorted by RSSI strength (strongest first)
         * 
         * @param anchors_listed Reference to vector of all available anchors
         * @param max_n Maximum number of anchors to return (default: 5)
         * @return std::vector<Anchor*> Pointers to the most significant anchors
         */
        std::vector<Anchor*> get_significant_anchors(std::vector<Anchor*>& anch_list, int max_n = Calibration::MAX_SIGNIFICANT_ANCHORS);
        
        /**
         * @brief Calculates distances between significant anchors and the tag
         * 
         * Computes 3D Euclidean distances using the R3_distance utility function.
         * Only processes anchors identified as significant by get_significant_anchors().
         * 
         * @param anch_list Vector of anchor pointers to process
         * @return std::unordered_map<Anchor*, float> Map of anchor pointers to distances in meters
         */
        std::unordered_map<Anchor*, float> distances(std::vector<Anchor*>& anch_list);
        
        /**
         * @brief Calculates z-values (standardized residuals) for significant anchors
         * 
         * Computes z-values using the path loss model, comparing predicted vs actual RSSI.
         * Z-values indicate how well the anchor's RSSI fits the expected signal propagation.
         * Lower absolute z-values indicate better fit.
         * 
         * @param anch_list Vector of anchor pointers to process
         * @return std::unordered_map<Anchor*, float> Map of anchor pointers to z-values
         */
        std::unordered_map<Anchor*, float> z_vals(std::vector<Anchor*>& anch_list);
        
        /**
         * @brief Calculates a confidence score for the positioning estimate
         * 
         * Uses Student's t-distribution to assess the quality of the positioning solution
         * based on z-values from all significant anchors. Higher scores indicate better
         * confidence in the position estimate.
         * 
         * @param anch_list Vector of anchor pointers to process
         * @param v Degrees of freedom for Student's t-distribution (default: 5)
         * @param scale Scaling factor for the exponential transform (default: 2.0)
         * @return float Confidence score (higher = more confident)
         */
        float confidence_score(std::vector<Anchor*>& anch_list, int v = 5, float scale = 2.0);
        
        /**
         * @brief Calculates the 95% confidence error radius for the position estimate
         * 
         * Converts the confidence score into a circular error probable (CEP95) radius.
         * This represents the radius within which the true position is expected to lie
         * with 95% probability.
         * 
         * @param anch_list Vector of anchor pointers to process
         * @return float Error radius in meters (95% confidence)
         */
        float error_radius(std::vector<Anchor*>& anch_list);
};


/**
 * @brief Updates anchor parameters and health metrics based on tag measurements
 * 
 * This function performs a comprehensive update of all anchors using data from a single tag.
 * It operates in two phases:
 * 
 * 1. Parameter Update: Updates path loss parameters (RSSI_0, n) for significant anchors
 *    that have RSSI measurements from the tag.
 * 
 * 2. Health Update: Updates EWMA health metrics for anchors that pass quality gates:
 *    - RSSI delta from strongest signal must be ≤ deltaR
 *    - Time since last seen must be ≤ T_vis
 * 
 * Uses efficient pointer-based operations to modify original anchor objects directly.
 * 
 * @param anch_list Reference to vector of anchor pointers to update (modified in-place)
 * @param inpt_tag Constant reference to tag providing RSSI measurements and position
 * @param inpt_model Constant reference to path loss model for calculations
 * @param now Current timestamp for health calculations
 * @param deltaR Maximum RSSI difference from strongest signal (dB, default: 12.0)
 * @param T_vis Maximum time since anchor was last seen (ms, default: 6000)
 */
void update_anchors_from_tag_data(
    std::vector<Anchor*>& anch_list, 
    const Tag& inpt_tag, 
    const PathLossModel& inpt_model, 
    float now, 
    float deltaR = 12.0, 
    int T_vis = 6000
);