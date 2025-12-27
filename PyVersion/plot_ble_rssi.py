import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import json
import time
from collections import defaultdict
from typing import Dict, List, Tuple, Any
import paho.mqtt.client as mqtt
import requests

# MQTT Configuration
BROKER = ""  # MQTT broker address (redacted)
PORT = 0  # MQTT broker port (redacted)
POSITION_TOPIC = ""  # MQTT topic for position data (redacted)
ERROR_TOPIC = ""  # MQTT topic for error estimates (redacted)
CLIENT_ID = ""  # MQTT client identifier (redacted)

# Collection settings
COLLECTION_TIME = 30  # Data collection period in seconds

# API Configuration
ANCHOR_INIT_BASE = ""  # API endpoint for anchor configuration (redacted)
anch_api_auth = ("", "")  # API authentication credentials (redacted)

# Data storage
position_data = defaultdict(list)  # {tag_mac: [(timestamp, x, y, z), ...]}
error_data = defaultdict(list)     # {tag_mac: [(timestamp, error), ...]}
anchor_n_var_data = defaultdict(list)  # {anchor_mac: [(timestamp, n_var), ...]}
anchor_positions = {}              # {anchor_mac: (x, y, z)}
latest_tag_positions = {}          # {tag_mac: (x, y, z)}
latest_anchor_states = {}          # {anchor_mac: {'n_var': float, 'ewma': float}}

# Control variables
start_time = None
data_collection_complete = False
message_count = 0

def fetch_anchor_position(mac_address: str) -> Tuple[float, float, float]:
    """Fetch anchor position from API."""
    try:
        api_url = ANCHOR_INIT_BASE.format(mac_address)
        response = requests.get(api_url, auth=anch_api_auth)
        if response.status_code == 200:
            data = response.json()
            if data:
                anchor_data = data[0]
                return (anchor_data["x"], anchor_data["y"], anchor_data["z"])
    except Exception as e:
        print(f"Error fetching anchor {mac_address}: {e}")
    return (0, 0, 0)

def on_connect(client: mqtt.Client, userdata: Any, flags: Dict[str, Any], rc: int) -> None:
    """Callback when client connects to broker."""
    print(f"Connected to MQTT broker with result code {rc}")
    if rc != 0:
        print(f"Connection failed! Result code: {rc}")
        return
    
    client.subscribe(POSITION_TOPIC)
    client.subscribe(ERROR_TOPIC)
    print(f"Subscribed to {POSITION_TOPIC} and {ERROR_TOPIC}")
    print(f"Collecting data for {COLLECTION_TIME} seconds...")
    print("Waiting for messages... (make sure mqtt_runner.py is running!)")

def on_message(client: mqtt.Client, userdata: Any, msg: mqtt.MQTTMessage) -> None:
    """Callback when a message is received."""
    global start_time, data_collection_complete, message_count
    
    if data_collection_complete:
        return
    
    try:
        current_time = time.time()
        
        # Initialize start time on first message
        if start_time is None:
            start_time = current_time
            print("Data collection started...")
        
        # Check if collection period is over
        if current_time - start_time >= COLLECTION_TIME:
            data_collection_complete = True
            print(f"\nData collection complete! Processed {message_count} messages.")
            client.disconnect()
            return
        
        payload = json.loads(msg.payload.decode())
        message_count += 1
        
        if msg.topic == POSITION_TOPIC:
            # Process position data
            tag_mac = payload["tag"]["mac"]
            position = payload["location"]["position"]
            tag_pos = (position["x"], position["y"], position["z"])
            
            print(f"📍 Position data for tag {tag_mac[-6:]}: {tag_pos}")
            
            # Store position data with timestamp
            position_data[tag_mac].append((current_time, *tag_pos))
            latest_tag_positions[tag_mac] = tag_pos
            
            # Fetch anchor positions if we haven't seen them before
            for anchor in position["used_anchors"]:
                anchor_mac = anchor["mac"]
                if anchor_mac not in anchor_positions:
                    anchor_positions[anchor_mac] = fetch_anchor_position(anchor_mac)
                    print(f"Fetched anchor {anchor_mac}: {anchor_positions[anchor_mac]}")
        
        elif msg.topic == ERROR_TOPIC:
            # Process error estimate data (new format with anchor data)
            tag_mac = payload["tag_mac"]
            error_estimate = payload["error_estimate"]
            
            print(f"📊 Error data for tag {tag_mac[-6:]}: {error_estimate:.2f}m")
            
            # Store error data with timestamp
            error_data[tag_mac].append((current_time, error_estimate))
            
            # Process anchor data
            if "anchors" in payload:
                for anchor_data in payload["anchors"]:
                    anchor_mac = anchor_data["mac"]
                    n_var = anchor_data["n_var"]
                    ewma = anchor_data["ewma"]
                    
                    # Store n_var data with timestamp
                    anchor_n_var_data[anchor_mac].append((current_time, n_var))
                    
                    # Update latest anchor states
                    latest_anchor_states[anchor_mac] = {
                        "n_var": n_var,
                        "ewma": ewma
                    }
        
        # Progress indicator
        elapsed = current_time - start_time
        remaining = COLLECTION_TIME - elapsed
        if message_count % 10 == 0:  # Print every 10 messages
            print(f"Messages: {message_count}, Time remaining: {remaining:.1f}s")
            
    except Exception as e:
        print(f"Error processing message: {e}")

def create_plots():
    """Create static plots after data collection is complete."""
    if not position_data and not error_data:
        print("No data collected!")
        return
    
    print("Creating plots...")
    
    # Create figure with subplots
    fig, axes = plt.subplots(3, 2, figsize=(15, 15))
    fig.suptitle(f'BLE RSSI Analysis - {COLLECTION_TIME}s Collection Period')
    ax1, ax2 = axes[0]
    ax3, ax4 = axes[1]
    ax5, ax6 = axes[2]
    
    # Colors for different tags and anchors
    all_items = list(set(list(position_data.keys()) + list(error_data.keys()) + list(anchor_n_var_data.keys())))
    colors = plt.cm.get_cmap('tab10')(np.linspace(0, 1, len(all_items)))
    color_map = {item: colors[i] for i, item in enumerate(all_items)}
    
    # Plot 1: Error radius over time
    ax1.set_title('Error Radius Over Time')
    ax1.set_xlabel('Time (seconds)')
    ax1.set_ylabel('Error Radius (meters)')
    
    for tag_mac, data in error_data.items():
        if data:
            timestamps, errors = zip(*data)
            relative_times = [(t - start_time) for t in timestamps]
            ax1.plot(relative_times, errors, 
                    label=f'Tag {tag_mac[-6:]}', color=color_map[tag_mac], 
                    marker='o', markersize=3, linewidth=2)
    
    if error_data:
        ax1.legend()
    ax1.grid(True)
    
    # Plot 2: Tag trajectories
    ax2.set_title('Tag Movement Trajectories')
    ax2.set_xlabel('X (meters)')
    ax2.set_ylabel('Y (meters)')
    
    for tag_mac, data in position_data.items():
        if data and len(data) > 1:
            positions = [(x, y, z) for _, x, y, z in data]
            x_coords = [pos[0] for pos in positions]
            y_coords = [pos[1] for pos in positions]
            color = color_map[tag_mac]
            
            # Plot trajectory
            ax2.plot(x_coords, y_coords, color=color, alpha=0.7, 
                    label=f'Tag {tag_mac[-6:]}', linewidth=2)
            
            # Mark start and end points
            ax2.scatter(x_coords[0], y_coords[0], color=color, s=100, 
                       marker='s', edgecolor='black', linewidth=2, label=f'{tag_mac[-6:]} Start')
            ax2.scatter(x_coords[-1], y_coords[-1], color=color, s=150, 
                       marker='o', edgecolor='black', linewidth=2, label=f'{tag_mac[-6:]} End')
    
    if position_data:
        ax2.legend()
    ax2.grid(True)
    ax2.set_aspect('equal')
    ax2.invert_yaxis()  # Fix Y-axis inversion to match system coordinates
    
    # Plot 3: Final system state map
    ax3.set_title('Final System State with Error Circles')
    ax3.set_xlabel('X (meters)')
    ax3.set_ylabel('Y (meters)')
    
    # Plot anchors
    if anchor_positions:
        anchor_coords = list(anchor_positions.values())
        anchor_x = [coord[0] for coord in anchor_coords]
        anchor_y = [coord[1] for coord in anchor_coords]
        ax3.scatter(anchor_x, anchor_y, c='blue', marker='^', s=300, 
                   label='Anchors', edgecolor='black', linewidth=2, zorder=5)
        
        # Label anchors
        for mac, (x, y, z) in anchor_positions.items():
            ax3.annotate(mac[-4:], (x, y), xytext=(5, 5), 
                       textcoords='offset points', fontsize=12, color='blue', weight='bold')
    
    # Add real position marker (center of anchors)
    if anchor_positions and len(anchor_positions) >= 4:
        anchor_coords = list(anchor_positions.values())
        center_x = np.mean([coord[0] for coord in anchor_coords])
        center_y = np.mean([coord[1] for coord in anchor_coords])
        ax3.scatter(center_x, center_y, c='red', marker='X', s=400, 
                   label='Real Position', edgecolor='black', linewidth=3, zorder=6)
        ax3.annotate('REAL', (center_x, center_y), xytext=(0, -25), 
                   textcoords='offset points', fontsize=12, color='red', 
                   weight='bold', ha='center')

    # Plot final tag positions with error circles
    for tag_mac, pos in latest_tag_positions.items():
        color = color_map[tag_mac]
        ax3.scatter(pos[0], pos[1], c=color, marker='o', s=250, 
                   label=f'Tag {tag_mac[-6:]}', edgecolor='black', linewidth=2, zorder=4)
        
        # Add error circle if we have error data
        if tag_mac in error_data and error_data[tag_mac]:
            latest_error = error_data[tag_mac][-1][1]  # Get error from last (timestamp, error) tuple
            circle = mpatches.Circle((pos[0], pos[1]), latest_error, 
                                   color=color, fill=False, alpha=0.6, linewidth=3)
            ax3.add_patch(circle)
            
            # Add error value as text
            ax3.annotate(f'{latest_error:.2f}m', (pos[0], pos[1]), 
                       xytext=(15, -20), textcoords='offset points', 
                       fontsize=10, color=color, weight='bold',
                       bbox=dict(boxstyle="round,pad=0.3", facecolor='white', alpha=0.8))
    
    if latest_tag_positions or anchor_positions:
        ax3.legend()
    ax3.grid(True)
    ax3.set_aspect('equal')
    ax3.invert_yaxis()  # Fix Y-axis inversion to match system coordinates
    
    # Plot 4: Error statistics summary
    ax4.set_title('Error Statistics Summary')
    ax4.set_xlabel('Tag')
    ax4.set_ylabel('Error Radius (meters)')
    
    if error_data:
        tag_names = []
        min_errors = []
        max_errors = []
        avg_errors = []
        final_errors = []
        
        for tag_mac, data in error_data.items():
            if data:
                errors = [err for _, err in data]
                tag_names.append(tag_mac[-6:])
                min_errors.append(min(errors))
                max_errors.append(max(errors))
                avg_errors.append(np.mean(errors))
                final_errors.append(errors[-1])
        
        if tag_names:
            x_pos = np.arange(len(tag_names))
            width = 0.2
            
            ax4.bar(x_pos - width*1.5, min_errors, width, label='Min', alpha=0.8)
            ax4.bar(x_pos - width*0.5, avg_errors, width, label='Average', alpha=0.8)
            ax4.bar(x_pos + width*0.5, max_errors, width, label='Max', alpha=0.8)
            ax4.bar(x_pos + width*1.5, final_errors, width, label='Final', alpha=0.8)
            
            ax4.set_xticks(x_pos)
            ax4.set_xticklabels(tag_names)
            ax4.legend()
    
    ax4.grid(True)
    
    # Plot 5: Anchor n_var over time
    ax5.set_title('Anchor n_var Over Time')
    ax5.set_xlabel('Time (seconds)')
    ax5.set_ylabel('n_var')
    
    for anchor_mac, data in anchor_n_var_data.items():
        if data:
            timestamps, n_vars = zip(*data)
            relative_times = [(t - start_time) for t in timestamps]
            ax5.plot(relative_times, n_vars, 
                    label=f'Anchor {anchor_mac[-6:]}', color=color_map[anchor_mac], 
                    marker='o', markersize=3, linewidth=2)
    
    if anchor_n_var_data:
        ax5.legend()
    ax5.grid(True)
    
    # Plot 6: Current Anchor States (n_var vs ewma)
    ax6.set_title('Current Anchor States (n_var vs EWMA)')
    ax6.set_xlabel('EWMA')
    ax6.set_ylabel('n_var')
    
    if latest_anchor_states:
        for anchor_mac, state in latest_anchor_states.items():
            ax6.scatter(state["ewma"], state["n_var"], 
                       s=150, label=f'Anchor {anchor_mac[-6:]}',
                       alpha=0.7, edgecolor='black', linewidth=2)
            
            # Add anchor label next to point
            ax6.annotate(anchor_mac[-4:], 
                       (state["ewma"], state["n_var"]), 
                       xytext=(5, 5), textcoords='offset points',
                       fontsize=10, weight='bold')
        
        ax6.legend()
    
    ax6.grid(True)
    
    plt.tight_layout()
    print("Displaying plots... Close the plot window when done.")
    plt.show()

def main():
    """Main function to collect data and create plots."""
    global data_collection_complete
    
    print("=" * 60)
    print("BLE RSSI Data Collection and Analysis")
    print("=" * 60)
    print("⚠️  IMPORTANT: Make sure mqtt_runner.py is running first!")
    print(f"📡 Connecting to broker: {BROKER}:{PORT}")
    print(f"📋 Collection time: {COLLECTION_TIME} seconds")
    print(f"📂 Subscribing to:")
    print(f"   - {POSITION_TOPIC}")
    print(f"   - {ERROR_TOPIC}")
    print("=" * 60)
    
    # Set up MQTT client
    client = mqtt.Client(client_id=CLIENT_ID)
    client.on_connect = on_connect
    client.on_message = on_message
    
    try:
        # Connect to broker
        client.connect(BROKER, PORT, 60)
        
        print("Connected! Starting data collection...")
        
        # Run MQTT loop until data collection is complete
        timeout_counter = 0
        while not data_collection_complete:
            client.loop(timeout=1.0)
            timeout_counter += 1
            
            # Check if we haven't received any messages after 10 seconds
            if timeout_counter >= 10 and message_count == 0:
                print("WARNING: No messages received after 10 seconds!")
                print("Check that:")
                print("1. mqtt_runner.py is running")
                print("2. Tags are active and moving")
                print("3. Network connection is working")
                print("Continuing to wait...")
                timeout_counter = 0  # Reset counter
        
        # Create and show plots
        create_plots()
        
        print("\nData collection summary:")
        print(f"- Total messages processed: {message_count}")
        print(f"- Tags with position data: {len(position_data)}")
        print(f"- Tags with error data: {len(error_data)}")
        print(f"- Anchors discovered: {len(anchor_positions)}")
        print(f"- Anchors with n_var data: {len(anchor_n_var_data)}")
        
    except KeyboardInterrupt:
        print("\nData collection interrupted by user")
        if start_time:
            create_plots()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        client.disconnect()

if __name__ == "__main__":
    main() 