# C++ BLE RSSI Test Suite

This directory contains comprehensive tests for the C++ BLE RSSI positioning library, covering both utility functions and Kalman filter implementations.

## Modules Tested

## 📐 Utils Module (`utils.cpp`)

### `R3_distance` - 3D Distance Calculation
- **Basic distance calculations**: Unit vectors, origin distances
- **3D diagonal cases**: Complex coordinate calculations  
- **Symmetry**: Verifies distance(a,b) = distance(b,a)
- **Self-distance**: Confirms distance(a,a) = 0

### `logpdf_student_t` - Student's t-Distribution
- **Basic properties**: Symmetry, monotonicity 
- **Default parameter**: Tests v=5 default
- **Different degrees of freedom**: Various v values
- **Mathematical correctness**: PDF properties validation

### `cep95_from_conf` - Confidence to Precision Mapping
- **Exact table values**: All 8 research-based lookup table entries
- **Boundary conditions**: Below/above table limits
- **Linear interpolation**: Mid-point and fractional interpolation
- **Precision range**: Output bounds validation (0.5m - 8.0m)
- **Monotonicity**: Higher confidence → lower radius

## 📡 Kalman Module (`kalman.cpp`)

### `KalmanFilter` - RSSI Parameter Estimation
- **Constructor**: Initialization and default values
- **Basic functionality**: Core sequence_step operation
- **Edge cases**: Zero/negative distances, extreme RSSI values
- **Convergence**: Filter convergence with consistent measurements
- **State persistence**: Internal state maintenance across calls
- **Mathematical properties**: Log-distance model validation
- **Noise handling**: Stability with noisy measurements
- **Independence**: Multiple filter instance isolation

## 🏗️ Models Module (`models.cpp`)

### `Anchor` - BLE Beacon Management
- **Constructor**: MAC address, coordinates, timestamp initialization
- **Health monitoring**: EWMA-based anchor reliability tracking
- **Parameter updates**: Kalman filter-based RSSI calibration
- **State classification**: Warning and faulty state detection
- **Getters**: Complete access to all anchor properties

### `Tag` - Mobile Device Tracking
- **Constructor**: MAC, position, RSSI readings initialization
- **RSSI methods**: Anchor-specific RSSI retrieval and anchor listing
- **Exception handling**: Proper error handling for missing anchors
- **Getters**: Access to position estimates and RSSI data

### `PathLossModel` - RSSI-Distance Modeling
- **Constructor**: Default parameter initialization (d_0=1.0m, sigma=4.0dB)
- **mu calculation**: Expected RSSI using log-distance path loss model
- **z calculation**: Standardized residual for statistical analysis
- **Mathematical properties**: Distance-RSSI relationship validation
- **Integration**: Cross-class compatibility with Anchor system

## Building and Running

### Quick Start - All Tests
```bash
# Build and run all test suites
make test
```

### Individual Test Suites
```bash
# Run only utils tests
make test-utils

# Run only kalman tests  
make test-kalman

# Run only models tests
make test-models

# Build all without running
make all

# Get help
make help
```

### Manual Execution
```bash
# Build individual test executables
make test_utils
make test_kalman
make test_models

# Run tests manually
./test_utils
./test_kalman
./test_models

# Clean build artifacts
make clean
```

## Test Results

The comprehensive test suite validates:
- ✅ **Mathematical accuracy** of all utility and filter functions
- ✅ **Edge cases** and boundary conditions for robust operation
- ✅ **Research-based lookup tables** with interpolation accuracy
- ✅ **Kalman filter convergence** and stability properties
- ✅ **Real-world precision bounds** for BLE indoor positioning
- ✅ **Cross-module integration** (Kalman filter uses utils functions)

All tests must pass before deploying the BLE positioning system in production.

## Requirements

- **C++20 compiler** (g++ 10+ recommended)
- **Standard libraries**: `<cmath>`, `<numbers>`, `<array>`, `<tuple>`
- **Make** for build automation
- **Linux/Windows** compatible

## Test Coverage Summary

### Utils Module
- **R3_distance**: 2 test functions, 15+ assertions
- **logpdf_student_t**: 3 test functions, 10+ assertions  
- **cep95_from_conf**: 4 test functions, 25+ assertions
- **Utils Total**: 9 test functions, 50+ assertions

### Kalman Module  
- **KalmanFilter**: 8 test functions, 40+ assertions
- **Integration tests**: State persistence, convergence, noise handling
- **Mathematical validation**: Log-distance model, matrix operations
- **Kalman Total**: 8 test functions, 40+ assertions

### Models Module
- **Anchor class**: 3 test functions, 25+ assertions
- **Tag class**: 3 test functions, 20+ assertions
- **PathLossModel class**: 4 test functions, 30+ assertions
- **Integration tests**: Cross-class compatibility and data flow
- **Models Total**: 10 test functions, 75+ assertions

### Overall Coverage
- **Total modules**: 3 (utils + kalman + models)
- **Total test functions**: 27
- **Total assertions**: 165+
- **Test files**: `test_utils.cpp`, `test_kalman.cpp`, `test_models.cpp`

## Integration Notes

The test suite ensures comprehensive cross-module compatibility:
- **Kalman filter tests** depend on the utils module for distance calculations
- **Models tests** integrate with both utils (distance calculations) and kalman (parameter estimation)
- **Cross-class integration** validates the complete BLE positioning pipeline from raw RSSI measurements to health monitoring and parameter adaptation

This layered testing approach ensures mathematical consistency and functional integration across the entire positioning system.