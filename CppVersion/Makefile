# Makefile for BLE RSSI C++ Application
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -I.
LDFLAGS = -lmosquitto -lcurl -lpthread

# Source files
UTILS_SRC = utils.cpp
KALMAN_SRC = kalman.cpp
MODELS_SRC = models.cpp
METRICS_SRC = metrics.cpp
MAIN_SRC = main.cpp

# Header files
HEADERS = utils.h kalman.h models.h metrics.h config.h

# All source files for the main application
ALL_SRC = $(MAIN_SRC) $(METRICS_SRC) $(MODELS_SRC) $(KALMAN_SRC) $(UTILS_SRC)

# Target executable
TARGET = ble_rssi_runner

# Default target - build the main application
all: $(TARGET)

# Build main executable
$(TARGET): $(ALL_SRC) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(ALL_SRC) -o $(TARGET) $(LDFLAGS)

# Clean build artifacts
clean:
	rm -f $(TARGET)

# Install dependencies (Ubuntu/Debian)
install-deps:
	@echo "Installing required dependencies..."
	sudo apt-get update
	sudo apt-get install -y libmosquitto-dev libcurl4-openssl-dev nlohmann-json3-dev

# Install dependencies (macOS with Homebrew)
install-deps-mac:
	@echo "Installing required dependencies for macOS..."
	brew install mosquitto curl nlohmann-json

# Run the application
run: $(TARGET)
	./$(TARGET)

# Debug build
debug: CXXFLAGS += -g -DDEBUG
debug: $(TARGET)

# Help target
help:
	@echo "Available targets:"
	@echo "  all           - Build the main application"
	@echo "  $(TARGET)     - Build the main executable"
	@echo "  clean         - Remove build artifacts"
	@echo "  install-deps  - Install dependencies (Ubuntu/Debian)"
	@echo "  install-deps-mac - Install dependencies (macOS)"
	@echo "  run           - Build and run the application"
	@echo "  debug         - Build with debug symbols"
	@echo "  help          - Show this help message"

.PHONY: all clean install-deps install-deps-mac run debug help