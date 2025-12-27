# Makefile for C++ BLE RSSI tests
CXX = g++
CXXFLAGS = -std=c++20 -Wall -Wextra -O2 -I..
LDFLAGS = 

# Source files
UTILS_SRC = ../utils.cpp
KALMAN_SRC = ../kalman.cpp
MODELS_SRC = ../models.cpp
METRICS_SRC = ../metrics.cpp
UTILS_TEST_SRC = test_utils.cpp
KALMAN_TEST_SRC = test_kalman.cpp
MODELS_TEST_SRC = test_models.cpp
METRICS_TEST_SRC = test_metrics.cpp
MQTT_PERF_TEST_SRC = test_mqtt_performance.cpp

# Targets
UTILS_TARGET = test_utils
KALMAN_TARGET = test_kalman
MODELS_TARGET = test_models
METRICS_TARGET = test_metrics
MQTT_PERF_TARGET = test_mqtt_performance
ALL_TARGETS = $(UTILS_TARGET) $(KALMAN_TARGET) $(MODELS_TARGET) $(METRICS_TARGET) $(MQTT_PERF_TARGET)

# Default target - build all tests
all: $(ALL_TARGETS)

# Build utils test executable
$(UTILS_TARGET): $(UTILS_TEST_SRC) $(UTILS_SRC)
	$(CXX) $(CXXFLAGS) $(UTILS_TEST_SRC) $(UTILS_SRC) -o $(UTILS_TARGET) $(LDFLAGS)

# Build kalman test executable  
$(KALMAN_TARGET): $(KALMAN_TEST_SRC) $(KALMAN_SRC) $(UTILS_SRC)
	$(CXX) $(CXXFLAGS) $(KALMAN_TEST_SRC) $(KALMAN_SRC) $(UTILS_SRC) -o $(KALMAN_TARGET) $(LDFLAGS)

# Build models test executable
$(MODELS_TARGET): $(MODELS_TEST_SRC) $(MODELS_SRC) $(KALMAN_SRC) $(UTILS_SRC)
	$(CXX) $(CXXFLAGS) $(MODELS_TEST_SRC) $(MODELS_SRC) $(KALMAN_SRC) $(UTILS_SRC) -o $(MODELS_TARGET) $(LDFLAGS)

# Build metrics test executable
$(METRICS_TARGET): $(METRICS_TEST_SRC) $(METRICS_SRC) $(MODELS_SRC) $(KALMAN_SRC) $(UTILS_SRC)
	$(CXX) $(CXXFLAGS) $(METRICS_TEST_SRC) $(METRICS_SRC) $(MODELS_SRC) $(KALMAN_SRC) $(UTILS_SRC) -o $(METRICS_TARGET) $(LDFLAGS)

# Build MQTT performance test executable 
$(MQTT_PERF_TARGET): $(MQTT_PERF_TEST_SRC) $(METRICS_SRC) $(MODELS_SRC) $(KALMAN_SRC) $(UTILS_SRC)
	$(CXX) $(CXXFLAGS) $(MQTT_PERF_TEST_SRC) $(METRICS_SRC) $(MODELS_SRC) $(KALMAN_SRC) $(UTILS_SRC) -o $(MQTT_PERF_TARGET) $(LDFLAGS)


# Run all tests
test: $(ALL_TARGETS)
	@echo "Running utils tests..."
	./$(UTILS_TARGET)
	@echo ""
	@echo "Running kalman tests..."
	./$(KALMAN_TARGET)
	@echo ""
	@echo "Running models tests..."
	./$(MODELS_TARGET)
	@echo ""
	@echo "Running metrics tests..."
	./$(METRICS_TARGET)
	@echo ""
	@echo "Running MQTT performance tests..."
	./$(MQTT_PERF_TARGET)
	@echo ""
	@echo "🎉 All test suites completed!"

# Run individual test suites
test-utils: $(UTILS_TARGET)
	./$(UTILS_TARGET)

test-kalman: $(KALMAN_TARGET)
	./$(KALMAN_TARGET)

test-models: $(MODELS_TARGET)
	./$(MODELS_TARGET)

test-metrics: $(METRICS_TARGET)
	./$(METRICS_TARGET)

test-mqtt-perf: $(MQTT_PERF_TARGET)
	./$(MQTT_PERF_TARGET)

# Clean build artifacts
clean:
	rm -f $(ALL_TARGETS)

# Rebuild everything
rebuild: clean all

# Help target
help:
	@echo "Available targets:"
	@echo "  all          - Build all test executables"
	@echo "  test         - Build and run all tests"
	@echo "  test-utils   - Build and run utils tests only"
	@echo "  test-kalman  - Build and run kalman tests only"
	@echo "  test-models  - Build and run models tests only"
	@echo "  test-metrics - Build and run metrics tests only"
	@echo "  test-mqtt-perf - Build and run MQTT performance tests only"
	@echo "  clean        - Remove all build artifacts"
	@echo "  rebuild      - Clean and rebuild everything"
	@echo "  help         - Show this help message"

.PHONY: all test test-utils test-kalman test-models test-metrics test-mqtt-perf clean rebuild help