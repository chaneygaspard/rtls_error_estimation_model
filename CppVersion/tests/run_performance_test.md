# MQTT Performance Test

This performance test measures the speed of MQTT message processing in the BLE RSSI positioning system without requiring an actual MQTT connection.

## Overview

The test simulates the complete MQTT message processing pipeline:
1. JSON parsing of incoming MQTT messages
2. Anchor initialization (first message only)
3. Tag object creation
4. Error radius calculation using the TagSystem
5. Output JSON generation

## Target Performance

- **Target**: < 1ms (1000 microseconds) per message
- **Measurement**: End-to-end processing time from JSON parse to output generation

## Sample Messages Included

The test includes realistic BLE positioning messages with varying anchor counts:
- **Small message**: 3 used + 1 unused anchors (example format)
- **Medium message**: 6 used + 2 unused anchors 
- **Large message**: 10 used + 4 unused anchors
- **Extra large message**: 15 used + 6 unused anchors

## Building and Running

```bash
# Build the performance test
cd ble_error_estimation/CppVersion/tests
make test-mqtt-perf

# Or build all tests
make all

# Run just the performance test
./test_mqtt_performance

# Run all tests (including performance)
make test
```

## Test Output

The test provides detailed performance statistics:

```
=== MQTT Processing Performance Results ===
Total measurements: 300
Target time: <1000us (<1ms)
Minimum time: 145.2us
Average time: 287.3us
Maximum time: 1205.8us
95th percentile: 456.1us
99th percentile: 823.4us
Target violations: 12/300 (4.0%)

Performance Assessment:
  Average < 1000us: ✓ PASS
  95th percentile < 1000us: ✓ PASS
  Violation rate < 5%: ✓ PASS
```

## Test Criteria

The test passes if:
1. **Average processing time** < 1000us
2. **95th percentile** < 1000us  
3. **Violation rate** < 5% (allows occasional spikes)

## What's Measured

- **Core Algorithm**: TagSystem.error_radius() calculation
- **JSON Processing**: Parse input, generate output
- **Anchor Management**: Lookup and filtering
- **Memory Operations**: Object creation and data access

## What's NOT Measured

- Network latency (MQTT broker communication)
- File I/O operations
- HTTP API calls for anchor initialization
- Actual MQTT publishing overhead

## Adding Custom Messages

You can add your own test messages by editing the `sample_mqtt_messages` vector in `test_mqtt_performance.cpp`. Each message should follow your system's BLE positioning format:

```json
{
    "is_moving": null,
    "location": {
        "dead_zones": [],
        "map_id": "<map_id_redacted>",
        "position": {
            "quality": "normal",
            "unused_anchors": [{"cart_d": 4.67, "id": "<anchor_id>", "mac": "<anchor_mac_redacted>", "rssi": -66.19}],
            "used_anchors": [
                {"cart_d": 1.0, "id": "<anchor_id>", "mac": "<anchor_mac_redacted>", "rssi": -57.0}
            ],
            "x": <x_coord>, "y": <y_coord>, "z": <z_coord>
        },
        "strategy": "centroid",
        "zones": []
    },
    "tag": {
        "ble": 1,
        "id": "<tag_id>", 
        "mac": "<tag_mac_redacted>",
        "uwb": 0
    },
    "timestamp": 1751374881169
}
```

## Dependencies

The test requires the same dependencies as the main application:
- nlohmann/json (for JSON processing)
- C++20 compiler
- Standard library with chrono support

Note: This test does **not** require mosquitto, curl, or network connectivity.