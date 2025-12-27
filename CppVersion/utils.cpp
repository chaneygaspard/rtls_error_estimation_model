#include <cmath>
#include "utils.h"
#include "config.h"

// Define pi constant for C++17 compatibility
constexpr float PI = 3.14159265358979323846f;

float  R3_distance(const PointR3& a, const PointR3& b){
    float delta_0 = std::pow((std::get<0>(a) - std::get<0>(b)), 2);
    float delta_1 = std::pow((std::get<1>(a) - std::get<1>(b)), 2);
    float delta_2 = std::pow((std::get<2>(a) - std::get<2>(b)), 2);

    return std::sqrt(delta_0 + delta_1 + delta_2);
}


float logpdf_student_t(float z, int v) {
    float pi = PI;

    float a_1 = std::lgamma((v + 1) / 2);
    float a_2 = std::lgamma(v / 2);
    float a_3 = 0.5 * std::log(v * pi);
    float a_4 = (static_cast<float>(v + 1) / 2.0f) 
    * (std::log1p((z * z) / static_cast<float>(v)));   
    
    return a_1 - a_2 - a_3 - a_4;
}


float cep95_from_conf(float p_conf) {
    // Use the calibration table from config.h
    const auto& table = Calibration::CEP95_TABLE;

    if (p_conf <= table.front().first) {
        return table.front().second;
    }

    if (p_conf >= table.back().first) {
        return table.back().second;
    }

    size_t i = 0;
    for (; i < table.size() - 1; ++i) {
        float x_next = table[i + 1].first;
        if (x_next > p_conf)
            break;
    }

    float x0 = table[i].first;
    float x1 = table[i + 1].first;
    float y0 = table[i].second;
    float y1 = table[i + 1].second;

    float t = (p_conf - x0) / (x1 - x0);
    return y0 + t * (y1 - y0);
}