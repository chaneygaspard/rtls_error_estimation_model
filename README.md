# RTLS Error Estimation Model

A real-time Bluetooth Low Energy (BLE) positioning system that estimates position error bounds for Real-Time Location Systems (RTLS) using RSSI (Received Signal Strength Indicator) measurements. Developed as an intern project for Ubudu, this system processes live positioning data streams to provide statistical confidence intervals for tag position estimates with adaptive anchor health monitoring.

## Overview

This project implements a probabilistic error estimation model for RTLS systems that:
- Processes real-time positioning data via MQTT streams
- Dynamically discovers and initializes anchor infrastructure
- Adaptively learns path-loss parameters using Kalman filtering
- Monitors anchor health and automatically filters faulty anchors
- Calculates 95% confidence error bounds (CEP95) for position estimates

## Core Methods & Algorithms

### Path-Loss Model
The system uses a log-distance path-loss model to relate RSSI measurements to distance:
```
RSSI = RSSI₀ - 10n·log₁₀(d/d₀) + noise
```
Where:
- `RSSI₀`: Reference signal strength at distance `d₀`
- `n`: Path-loss exponent (environment-dependent)
- `d`: Distance between tag and anchor
- `d₀`: Reference distance (1.0m)
- `σ`: Measurement noise standard deviation (4.0 dB)

### Kalman Filter Parameter Estimation
Each anchor maintains adaptive path-loss parameters (`RSSI₀` and `n`) that are continuously updated using a Kalman filter:
- **State vector**: `[RSSI₀, n]`
- **Process noise**: Tuned for parameter drift
- **Measurement model**: Linearized path-loss equation
- **Adaptive learning**: Parameters converge over 10-15 measurements

### Anchor Health Monitoring
An EWMA (Exponentially Weighted Moving Average) filter tracks anchor reliability:
```
ewma = λ·z² + (1-λ)·ewma_prev
```
Where `z` is the normalized residual from the path-loss model. Anchors are classified as:
- **Healthy** (`ewma < 4.0`): Used in calculations
- **Warning** (`4.0 ≤ ewma < 8.0`): Flagged but still used
- **Faulty** (`ewma ≥ 8.0`): Automatically excluded from calculations

### Error Radius Calculation
The system computes 95% confidence error bounds (CEP95) using:
- **Student's t-distribution**: Statistical assessment of measurement consistency
- **Confidence scoring**: Aggregated log-probability across significant anchors
- **Lookup table interpolation**: Maps confidence scores to error radii

### Anchor Filtering Logic
Anchors are excluded from calculations if:
1. Health threshold exceeded (`ewma ≥ 8.0`)
2. Signal too weak (`RSSI difference > 7.0 dB`)
3. Stale data (`time since last seen > 6000 seconds`)

## Technologies & Tools

### Python Implementation
- **MQTT Client**: `paho-mqtt` for real-time data streaming
- **Numerical Computing**: `numpy` for matrix operations and Kalman filtering
- **Statistics**: `scipy` for statistical distributions
- **Visualization**: `matplotlib` for system analysis and debugging plots
- **HTTP Client**: `requests` for API integration
- **Testing**: `pytest` for comprehensive unit testing

### C++ Implementation
- **MQTT Client**: `libmosquitto` for high-performance message processing
- **HTTP Client**: `libcurl` for anchor configuration API calls
- **JSON Processing**: `nlohmann-json` for modern C++ JSON handling
- **Performance**: 10x faster than Python version (~500+ msg/sec vs ~50 msg/sec)
- **Memory Efficiency**: ~5MB vs ~50MB in Python version

### Infrastructure
- **MQTT**: Real-time message broker for positioning data streams
- **REST API**: Anchor configuration and position retrieval
- **JSON**: Data serialization for MQTT messages

## Architecture

### Data Flow
```
MQTT Message → JSON Parse → Tag Creation → Anchor Processing → 
Error Calculation → Health Updates → JSON Response → MQTT Publish
```

### Key Components
1. **TagSystem**: Processes tag-anchor relationships and calculates error estimates
2. **Anchor**: Manages anchor state, health metrics, and adaptive parameters
3. **PathLossModel**: RSSI-distance calculations using path-loss equation
4. **KalmanFilter**: Adaptive parameter estimation for RSSI₀ and path-loss exponent
5. **MQTT Runner**: Real-time processing engine for live data streams

### Dynamic Anchor Discovery
The system automatically discovers anchors from the first MQTT message:
1. Extracts MAC addresses from `used_anchors` and `unused_anchors` arrays
2. Queries API for each anchor's position coordinates
3. Initializes `Anchor` objects with default parameters
4. Parameters adaptively evolve via Kalman filtering as data arrives

## Implementation Highlights

- **Dual Implementation**: Both Python (prototype) and C++ (production) versions
- **Real-time Processing**: Sub-second latency for message processing
- **Adaptive Learning**: System automatically adapts to environment changes
- **Fault Tolerance**: Automatic detection and filtering of unreliable anchors
- **Statistical Rigor**: Confidence intervals based on Student's t-distribution
- **Comprehensive Testing**: Unit tests for all core components (1,500+ lines of test code)

## Project Structure

```
rtls_error_estimation_model/
├── PyVersion/          # Python implementation (prototype)
│   ├── models.py       # Anchor, Tag, PathLossModel classes
│   ├── kalman.py       # Kalman filter implementation
│   ├── metrics.py      # TagSystem and health monitoring
│   ├── mqtt_runner.py  # Real-time MQTT processing
│   ├── utils.py        # Distance and statistical utilities
│   └── tests/          # Comprehensive test suite
├── CppVersion/         # C++ implementation (production)
│   ├── models.h/cpp    # Core data models
│   ├── kalman.h/cpp    # Kalman filter
│   ├── metrics.h/cpp   # Error estimation and health monitoring
│   ├── main.cpp        # MQTT runner application
│   └── tests/          # C++ test suite
└── README.md           # This file
```

## Performance Characteristics

- **Learning Time**: 10-15 messages for parameter convergence
- **Health Detection**: Responds to anchor failures within 5-10 messages
- **Real-time Processing**: Sub-second message processing latency
- **Error Convergence**: CEP95 bounds stabilize as bad anchors are filtered
- **C++ Performance**: 10x throughput improvement over Python version

## Key Achievements

- Developed probabilistic error estimation model for RTLS systems
- Implemented adaptive Kalman filtering for environment-specific parameter learning
- Designed robust anchor health monitoring system with automatic fault detection
- Created dual Python/C++ implementations demonstrating software engineering versatility
- Achieved real-time processing capabilities suitable for production deployment
- Delivered comprehensive test coverage ensuring system reliability
