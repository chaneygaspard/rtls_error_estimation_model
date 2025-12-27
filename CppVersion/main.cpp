#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <thread>
#include <chrono>
#include <exception>
#include <sstream>
#include <iomanip>
#include <atomic>
#include <mutex>

// External libraries (you'll need to install these)
#include <mosquitto.h>
#include <nlohmann/json.hpp>
#include <curl/curl.h>

// Local includes
#include "models.h"
#include "metrics.h"
#include "utils.h"
#include "config.h"

using json = nlohmann::json;
using namespace ConfigInput;
using namespace ConfigOutput;

std::mutex anchor_mutex;

// Debug logging macro
#define DEBUG_LOG(msg) \
    do { \
        if (Config::ENABLE_DEBUG_LOGGING) { \
            std::cout << "[DEBUG] " << msg << std::endl; \
        } \
    } while(0)

// Global state structure for MQTT userdata
struct MQTTUserData {
    std::unordered_map<std::string, std::unique_ptr<Anchor>> anchors;
    bool anchors_initialized = false;
    PathLossModel model;
};

// Curl callback function for HTTP responses
struct HTTPResponse {
    std::string data;
};

static size_t WriteCallback(void* contents, size_t size, size_t nmemb, HTTPResponse* response) {
    size_t total_size = size * nmemb;
    response->data.append(static_cast<char*>(contents), total_size);
    return total_size;
}

/**
 * @brief Make HTTP GET request with basic authentication
 * 
 * @param url The URL to request
 * @param username Basic auth username
 * @param password Basic auth password
 * @return std::string Response body
 * @throws std::runtime_error if request fails
 */
std::string http_get_request(const std::string& url, const std::string& username, const std::string& password) {
    CURL* curl;
    CURLcode res;
    HTTPResponse response;
    
    curl = curl_easy_init();
    if (!curl) {
        throw std::runtime_error("Failed to initialize CURL");
    }
    
    // Set URL
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    
    // Set basic authentication
    std::string auth = username + ":" + password;
    curl_easy_setopt(curl, CURLOPT_USERPWD, auth.c_str());
    
    // Set callback function
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    
    // Set timeout
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, Config::HTTP_TIMEOUT_SEC);
    
    // Perform request
    res = curl_easy_perform(curl);
    
    // Check for errors
    if (res != CURLE_OK) {
        curl_easy_cleanup(curl);
        throw std::runtime_error("CURL request failed: " + std::string(curl_easy_strerror(res)));
    }
    
    // Check HTTP status code
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
    curl_easy_cleanup(curl);
    
    if (http_code != 200) {
        throw std::runtime_error("HTTP request failed with status: " + std::to_string(http_code));
    }
    
    return response.data;
}

/**
 * @brief Create an Anchor object by fetching anchor configuration from the API
 * 
 * @param anch_mac MAC address of the anchor to initialize
 * @return std::unique_ptr<Anchor> Configured Anchor object with position and MAC address from API
 * @throws std::runtime_error If API call fails
 */
std::unique_ptr<Anchor> create_anchor_class(const std::string& anch_mac) {
    std::cout << "Creating anchor for MAC: " << anch_mac << std::endl;
    
    // Replace {} in URL template with actual MAC address
    std::string api_url = ConfigInput::ANCHOR_INIT_BASE;
    size_t pos = api_url.find("{}");
    if (pos != std::string::npos) {
        api_url.replace(pos, 2, anch_mac);
    }
    
    try {
        std::string response = http_get_request(api_url, ConfigInput::API_USERNAME, ConfigInput::API_PASSWORD);
        
        // Parse JSON response
        json anch_data_list = json::parse(response);
        
        if (anch_data_list.empty()) {
            throw std::runtime_error("No anchor found for MAC address: " + anch_mac);
        }
        
        // Get the first (and only) anchor object
        json anch_data = anch_data_list[0];
        
        // Extract coordinates
        float x = anch_data["x"].get<float>();
        float y = anch_data["y"].get<float>();
        float z = anch_data["z"].get<float>();
        PointR3 coord = std::make_tuple(x, y, z);
        
        // Create and return anchor (using current timestamp)
        auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        
        return std::make_unique<Anchor>(anch_mac, coord, static_cast<float>(now));
        
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to create anchor " + anch_mac + ": " + e.what());
    }
}

/**
 * @brief Create multiple Anchor objects by fetching anchor configurations from the API
 * 
 * @param anch_macs List of MAC addresses of anchors to initialize
 * @return std::unordered_map<std::string, std::unique_ptr<Anchor>> Map of MAC addresses to Anchor objects
 */
std::unordered_map<std::string, std::unique_ptr<Anchor>> create_anchor_classes(const std::vector<std::string>& anch_macs) {
    std::unordered_map<std::string, std::unique_ptr<Anchor>> anchors;
    
    for (const auto& anch_mac : anch_macs) {
        try {
            anchors[anch_mac] = create_anchor_class(anch_mac);
            std::cout << "Successfully created anchor: " << anch_mac << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Failed to create anchor " << anch_mac << ": " << e.what() << std::endl;
            // Continue with other anchors even if one fails
        }
    }
    
    return anchors;
}

/**
 * @brief Create a Tag object from MQTT message data
 * 
 * @param tag_data Parsed JSON data from MQTT message
 * @return Tag object with position and RSSI readings
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
 * @brief Extract all anchor MAC addresses from a tag position message
 * 
 * @param tag_data The parsed JSON data from an MQTT tag position message
 * @return std::vector<std::string> List of unique anchor MAC addresses found in the message
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
 * @brief Create tag info structure for output message
 */
json create_tag_info(const std::string& tag_mac, float error_estimate) {
    return json{
        {"tag_mac", tag_mac},
        {"error_estimate", error_estimate}
    };
}

/**
 * @brief Create anchors info structure for output message
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
 * @brief Create complete output info combining tag and anchors data
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
 * @brief MQTT connection callback for input client
 */
void on_connect_input(struct mosquitto* mosq, void* userdata, int result) {
    (void)userdata; // Suppress unused parameter warning
    std::cout << "Connected to INPUT MQTT broker with result: " << result << std::endl;
    
    if (result == 0) {
        // Subscribe to tag position stream using input config
        int sub_result = mosquitto_subscribe(mosq, nullptr, ConfigInput::TOPIC.c_str(), 0);
        if (sub_result == MOSQ_ERR_SUCCESS) {
            std::cout << "Successfully subscribed to: " << ConfigInput::TOPIC << std::endl;
        } else {
            std::cerr << "Failed to subscribe to topic: " << sub_result << std::endl;
        }
    }
}

/**
 * @brief MQTT connection callback for output client
 */
void on_connect_output(struct mosquitto* mosq, void* userdata, int result) {
    (void)mosq; // Suppress unused parameter warning
    (void)userdata; // Suppress unused parameter warning
    std::cout << "Connected to OUTPUT MQTT broker with result: " << result << std::endl;
}

// Global output client for publishing
struct mosquitto* g_pub_client = nullptr;

/**
 * @brief MQTT message callback - main processing logic
 */
void on_message(struct mosquitto* mosq, void* userdata, const struct mosquitto_message* message) {
    (void)mosq; // Suppress unused parameter warning
    // Start timing for performance measurement
    auto perf_start = std::chrono::high_resolution_clock::now();
    
    // Thread safety: Lock mutex for all shared state access
    std::lock_guard<std::mutex> lock(anchor_mutex);
    
    try {
        MQTTUserData* data = static_cast<MQTTUserData*>(userdata);
        
        // Parse JSON message
        std::string payload_str(static_cast<char*>(message->payload), message->payloadlen);
        json tag_data = json::parse(payload_str);
        
        // Check if this is the first message and we need to initialize anchors
        if (!data->anchors_initialized) {
            std::cout << "First message received - discovering and initializing anchors..." << std::endl;
            
            // Extract all anchor MAC addresses from this message
            std::vector<std::string> discovered_anchor_macs = extract_anchor_macs_from_message(tag_data);
            std::cout << "Discovered anchor MACs: ";
            for (const auto& mac : discovered_anchor_macs) {
                std::cout << mac << " ";
            }
            std::cout << std::endl;
            DEBUG_LOG("Discovered " << discovered_anchor_macs.size() << " anchor MACs from first message");
            
            // Initialize all discovered anchors
            data->anchors = create_anchor_classes(discovered_anchor_macs);
            data->anchors_initialized = true;
            
            std::cout << "Initialized " << data->anchors.size() << " anchors" << std::endl;
        }
        
        // Create Tag object from message
        Tag message_tag = create_tag_class(tag_data);
        float timestamp = tag_data["timestamp"].get<float>();
        
        // Create vector of anchor pointers for anchors that have RSSI readings
        std::vector<Anchor*> anch_list;
        const auto& rssi_readings = message_tag.get_rssi_readings();
        
        for (const auto& [anch_mac, rssi_val] : rssi_readings) {
            auto anch_it = data->anchors.find(anch_mac);
            if (anch_it != data->anchors.end()) {
                anch_list.push_back(anch_it->second.get());
            } else {
                // Handle new anchor discovered after initialization
                std::cout << "Warning: Found new anchor " << anch_mac << " after initialization" << std::endl;
                try {
                    data->anchors[anch_mac] = create_anchor_class(anch_mac);
                    anch_list.push_back(data->anchors[anch_mac].get());
                } catch (const std::exception& e) {
                    std::cerr << "Failed to create new anchor " << anch_mac << ": " << e.what() << std::endl;
                }
            }
        }
        
        // Only proceed if we have at least some anchors
        if (!anch_list.empty()) {
            // Create TagSystem
            TagSystem message_system(message_tag, data->model);
            
            // Get error estimate
            float error_estimate = message_system.error_radius(anch_list);
            
            // Update anchor health and parameters
            update_anchors_from_tag_data(anch_list, message_tag, data->model, timestamp, Config::DEFAULT_DELTA_R, Config::DEFAULT_T_VIS);
            
            // Create and publish output message using OUTPUT client
            json output_msg = create_output_info(message_tag.get_mac_address(), error_estimate, anch_list);
            std::string output_str = output_msg.dump();
            
            int pub_result = mosquitto_publish(g_pub_client, nullptr, ConfigOutput::TOPIC.c_str(), 
                                             output_str.length(), output_str.c_str(), 0, false);
            
            if (pub_result == MOSQ_ERR_SUCCESS) {
                std::cout << "Published result for tag: " << message_tag.get_mac_address() 
                         << " with error estimate: " << error_estimate << std::endl;
                DEBUG_LOG("Message published to topic: " << ConfigOutput::TOPIC);
            } else {
                std::cerr << "Failed to publish message: " << pub_result << std::endl;
            }
        } else {
            std::cout << "No initialized anchors found for tag " << message_tag.get_mac_address() << std::endl;
        }
        
    } catch (const json::parse_error& e) {
        std::cerr << "JSON parse error: " << e.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error processing message: " << e.what() << std::endl;
    }
    // End timing and print performance info
    auto perf_end = std::chrono::high_resolution_clock::now();
    auto perf_us = std::chrono::duration_cast<std::chrono::microseconds>(perf_end - perf_start).count();
    if (Config::ENABLE_PERFORMANCE_LOGGING) {
        if (perf_us > 2000) {
            std::cerr << "[PERF WARNING] Processing took " << perf_us << "us (>2ms)" << std::endl;
        } else {
            std::cout << "[PERF] Processing took " << perf_us << "us" << std::endl;
        }
    }
}

/**
 * @brief MQTT logging callback
 */
void on_log(struct mosquitto* mosq, void* userdata, int level, const char* str) {
    (void)mosq; // Suppress unused parameter warning
    (void)userdata; // Suppress unused parameter warning
    std::cout << "MQTT Log [" << level << "]: " << str << std::endl;
}

/**
 * @brief Main MQTT runner function
 */
int mqtt_runner() {
    // Initialize libraries
    mosquitto_lib_init();
    curl_global_init(CURL_GLOBAL_DEFAULT);
    
    // Initialize user data
    MQTTUserData userdata;
    
    // Create INPUT MQTT client (for subscribing)
    struct mosquitto* sub_client = mosquitto_new(ConfigInput::CLIENT_ID.c_str(), true, &userdata);
    if (!sub_client) {
        std::cerr << "Failed to create INPUT MQTT client" << std::endl;
        return 1;
    }
    
    // Create OUTPUT MQTT client (for publishing)
    g_pub_client = mosquitto_new(ConfigOutput::CLIENT_ID.c_str(), true, nullptr);
    if (!g_pub_client) {
        std::cerr << "Failed to create OUTPUT MQTT client" << std::endl;
        mosquitto_destroy(sub_client);
        return 1;
    }
    
    // Set callbacks for INPUT client
    mosquitto_connect_callback_set(sub_client, on_connect_input);
    mosquitto_message_callback_set(sub_client, on_message);
    
    // Set callbacks for OUTPUT client
    mosquitto_connect_callback_set(g_pub_client, on_connect_output);
    
    // Enable MQTT logging only if configured
    if (Config::ENABLE_MQTT_LOGGING) {
        mosquitto_log_callback_set(sub_client, on_log);
        mosquitto_log_callback_set(g_pub_client, on_log);
    }
    
    // Connect INPUT client to input broker
    std::cout << "Connecting to INPUT MQTT broker: " << ConfigInput::BROKER << ":" << ConfigInput::PORT << std::endl;
    int sub_conn_result = mosquitto_connect(sub_client, ConfigInput::BROKER.c_str(), ConfigInput::PORT, Config::MQTT_KEEPALIVE);
    if (sub_conn_result != MOSQ_ERR_SUCCESS) {
        std::cerr << "Failed to connect to INPUT MQTT broker: " << sub_conn_result << std::endl;
        mosquitto_destroy(sub_client);
        mosquitto_destroy(g_pub_client);
        return 1;
    }
    
    // Connect OUTPUT client to output broker
    std::cout << "Connecting to OUTPUT MQTT broker: " << ConfigOutput::BROKER << ":" << ConfigOutput::PORT << std::endl;
    int pub_conn_result = mosquitto_connect(g_pub_client, ConfigOutput::BROKER.c_str(), ConfigOutput::PORT, Config::MQTT_KEEPALIVE);
    if (pub_conn_result != MOSQ_ERR_SUCCESS) {
        std::cerr << "Failed to connect to OUTPUT MQTT broker: " << pub_conn_result << std::endl;
        mosquitto_destroy(sub_client);
        mosquitto_destroy(g_pub_client);
        return 1;
    }
    
    // Start the main loop (using input client for message processing)
    std::cout << "Starting MQTT loop..." << std::endl;
    int loop_result = mosquitto_loop_forever(sub_client, -1, 1);
    
    // Cleanup
    mosquitto_destroy(sub_client);
    mosquitto_destroy(g_pub_client);
    mosquitto_lib_cleanup();
    curl_global_cleanup();
    
    return loop_result == MOSQ_ERR_SUCCESS ? 0 : 1;
}

/**
 * @brief Main function
 */
int main() {
    // Display application banner
    std::cout << "BLE RSSI Probability Model - C++ Version" << std::endl;
    std::cout << "===========================================" << std::endl;
    
    try {
        // Start the main MQTT processing loop
        return mqtt_runner();
    } catch (const std::exception& e) {
        // Handle any unhandled exceptions at the top level
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
}