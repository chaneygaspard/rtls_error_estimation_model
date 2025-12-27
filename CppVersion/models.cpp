#include <iostream>

#include "models.h"

/*ANCHOR:*/
Anchor::Anchor(std::string mac, PointR3 coordinate, float timestamp) {
    mac_address = mac; 
    coord = coordinate;
    last_seen = timestamp;
}

//getters:
std::string Anchor::get_mac_address() const {
    return mac_address;
}

PointR3 Anchor::get_coord() const {
    return coord;
}

float Anchor::get_ewma() const {
    return ewma;
}

float Anchor::get_last_seen() const {
    return last_seen;
}

float Anchor::get_RSSI_0() const {
    return RSSI_0;
}

float Anchor::get_n() const {
    return n;
}

const KalmanFilter& Anchor::get_kalman() const {
    return kalman;
}

//methods:
void Anchor::update_health(float z, float now, float LAMBDA) {
    ewma = LAMBDA * std::pow(z, 2) + (1 - LAMBDA) * ewma;
    last_seen = now;
}

void Anchor::update_parameters(float measured_rssi, float estimated_distance){
    std::tuple<float, float> kaloutpt = kalman.sequence_step(RSSI_0, n, measured_rssi, estimated_distance);
    RSSI_0 = std::get<0>(kaloutpt);
    n = std::get<1>(kaloutpt);
}

bool Anchor::is_warning() {
    return ewma >= 4 && ewma < 8;
}

bool Anchor::is_faulty() {
    return ewma >= 8;
}


/*TAG:*/
Tag::Tag(std::string mac, PointR3 coord, std::unordered_map<std::string, float> rssi_map) {
    mac_address = mac;
    est_coord = coord;
    rssi_readings = rssi_map;
}

//getters:
std::string Tag::get_mac_address() const {
    return mac_address;
}

PointR3 Tag::get_est_coord() const {
    return est_coord;
}

const std::unordered_map<std::string, float>& Tag::get_rssi_readings() const {
    return rssi_readings;
}

//methods:
float Tag::rssi_for_anchor(std::string anchor_mac){
    return rssi_readings.at(anchor_mac);
}

std::vector<std::string> Tag::anchors_included(){
    std::vector<std::string> anchs;

    for (const auto& pair : rssi_readings) {
        anchs.push_back(pair.first);
    }

    return anchs;
}

/*PATHLOSSMODEL*/
PathLossModel::PathLossModel() {}

//getters:
float PathLossModel::get_d_0() const {
    return d_0;
}

float PathLossModel::get_sigma() const {
    return sigma;
}

//methods:
float PathLossModel::mu(float RSSI_0, float n, float est_dist) {
    float safe_dist = std::max(est_dist, 1e-6f);
    return RSSI_0 - (10 * n * std::log10(safe_dist / d_0));
}

float PathLossModel::z(float rssi_freq, float RSSI_0, float n, float est_dist) {
    float mu_val = mu(RSSI_0, n, est_dist);
    return ((rssi_freq - mu_val) / sigma);
}