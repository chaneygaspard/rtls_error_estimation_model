#include <algorithm>    
#include <cmath>        
#include <numeric>
#include <vector>
#include <unordered_map>
#include <string>
#include <limits>

#include "metrics.h"
#include "config.h"

/*TAGSYSTEM*/
//constructor:
TagSystem::TagSystem(const Tag& inpt_tag, const PathLossModel& inpt_model) : tag(inpt_tag), model(inpt_model) {
}

//getters:
const Tag& TagSystem::get_tag() const {
    return tag;
}

const PathLossModel& TagSystem::get_model() const {
    return model;
}

//methods
std::vector<Anchor*> TagSystem::get_significant_anchors(std::vector<Anchor*>& anch_list, int max_n) {
    const std::unordered_map<std::string, float>& rssi_dict = tag.get_rssi_readings();
    
    if (rssi_dict.empty()) {
        return {};
    }
    
    float max_rssi = std::numeric_limits<float>::lowest();
    for (const auto& [key, value] : rssi_dict) {
        if (value > max_rssi) {
            max_rssi = value;
        }
    }

    std::vector<Anchor*> keep;
    
    for (auto* anchor : anch_list) {
        std::string anchor_mac = anchor->get_mac_address();
        auto rssi_it = rssi_dict.find(anchor_mac);
        
        if (rssi_it != rssi_dict.end() && 
            rssi_it->second >= (max_rssi - 10) && 
            anchor->get_ewma() < Calibration::EWMA_THRESHOLD) {
            keep.push_back(anchor);
        }
    }

    std::sort(keep.begin(), keep.end(),
        [&rssi_dict](const Anchor* a1, const Anchor* a2) {
            return rssi_dict.at(a1->get_mac_address()) > rssi_dict.at(a2->get_mac_address());
        }
    );

    // Truncate to top max_n
    if (keep.size() > static_cast<size_t>(max_n)) {
        keep.resize(max_n);
    }
    return keep;
}

std::unordered_map<Anchor*, float> TagSystem::distances(std::vector<Anchor*>& anch_list) {
    std::unordered_map<Anchor*, float> result;
    std::vector<Anchor*> significant_anchors = get_significant_anchors(anch_list);

    for (const auto& anchor : significant_anchors) {
        float dist = R3_distance(anchor->get_coord(), tag.get_est_coord());
        result[anchor] = dist;
    }
    return result;
}

std::unordered_map<Anchor*, float> TagSystem::z_vals(std::vector<Anchor*>& anch_list) {
    std::unordered_map<Anchor*, float> result;
    std::unordered_map<Anchor*, float> dist_dict = distances(anch_list);
    const std::unordered_map<std::string, float>& rssi_dict = tag.get_rssi_readings();

    for (const auto& pair : dist_dict) {
        float rssi_value = rssi_dict.at(pair.first->get_mac_address());
        result[pair.first] = model.z(rssi_value, pair.first->get_RSSI_0(), pair.first->get_n(), pair.second);
    }
    
    return result;
}

float TagSystem::confidence_score(std::vector<Anchor*>& anch_list, int v, float scale) {
    std::unordered_map<Anchor*, float> z_dict = z_vals(anch_list);
    if (z_dict.empty()) {
        return 0.0;
    }

    float weighted_sig = 0.0f;
    float total_weight = 0.0f;

    for (const auto& [anchor, z_val] : z_dict){    
        float anchor_ewma = anchor->get_ewma();
        float anchor_weight = 1.0f / (1.0f + anchor_ewma + std::pow(z_val, 2));
        weighted_sig += anchor_weight * logpdf_student_t(z_val, v);
        total_weight += anchor_weight;
    }

    float l = weighted_sig / total_weight;
    return std::exp(l / scale);
}

float TagSystem::error_radius(std::vector<Anchor*>& anch_list) {
    float p_val = confidence_score(anch_list);
    return cep95_from_conf(p_val);
}


/*METHOD*/
void update_anchors_from_tag_data(
    std::vector<Anchor*>& anch_list, 
    const Tag& inpt_tag, 
    const PathLossModel& inpt_model, 
    float now, 
    float deltaR, 
    int T_vis
){
    TagSystem moment_system = TagSystem(inpt_tag, inpt_model);
    const std::unordered_map<std::string, float>& rssi_dict = moment_system.get_tag().get_rssi_readings();

    //paramaters update
    if (rssi_dict.empty()) {
        return;
    }
    
    // Parameter updates - work directly with pointers to original anchors
    std::vector<Anchor*> significant_anchors = moment_system.get_significant_anchors(anch_list);
    std::unordered_map<Anchor*, float> distance_dict = moment_system.distances(anch_list);
    
    for (const auto& sign_anchor : significant_anchors) {
        float sign_anchor_rssi = rssi_dict.at(sign_anchor->get_mac_address());
        float sign_anchor_dist = distance_dict.at(sign_anchor);
        sign_anchor->update_parameters(sign_anchor_rssi, sign_anchor_dist);
    }

    //health update
    std::unordered_map<Anchor*, float> z_dict = moment_system.z_vals(anch_list);
    
    float max_rssi = std::numeric_limits<float>::lowest();
    for (const auto& pair : rssi_dict) {
        if (pair.second > max_rssi) {
            max_rssi = pair.second;
        }
    }

    for (const auto& pair : z_dict) {
        Anchor* sign_anchor = pair.first;
        float sign_anchor_rssi = rssi_dict.at(sign_anchor->get_mac_address());
        float rssi_delta = max_rssi - sign_anchor_rssi;
        float sign_anchor_z_val = pair.second;

        float time_since_last_seen = 0.0;
        if (sign_anchor->get_last_seen() != 0.0) {
            time_since_last_seen = now - sign_anchor->get_last_seen();
        }

        if (time_since_last_seen > T_vis || rssi_delta > deltaR) {
            continue;
        }
        
        // Directly update the anchor via pointer - no find_if needed!
        sign_anchor->update_health(sign_anchor_z_val, now);
    }
}