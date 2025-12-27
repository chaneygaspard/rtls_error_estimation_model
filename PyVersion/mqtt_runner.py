import json 
import time
import paho.mqtt.client as mqtt
import requests
from typing import Tuple, List, Dict, Optional, Any

from models  import Anchor, Tag, PathLossModel
from metrics import TagSystem, update_anchors_from_tag_data
from utils import PointR3


ANCHOR_STREAM_BASE: str = ""  # MQTT topic pattern for anchor data (redacted)
anch_api_auth: Tuple[str, str] = ("", "")  # API authentication credentials (redacted)
ANCHOR_INIT_BASE: str = ""  # API endpoint for anchor configuration (redacted)
TAG_POSITION_STREAM: str = ""  # MQTT topic for tag position data (redacted)

BROKER: str = ""  # MQTT broker address (redacted)
PORT: int = 0  # MQTT broker port (redacted)
TOPIC_OUT = ""  # MQTT topic for error estimates output (redacted)
CLIENT_ID: str = ""  # MQTT client identifier (redacted)
STALE_T_SEC = 15 * 60  # 15 minutes

ANCHOR_MACS: List[str] = []  # List of anchor MAC addresses (redacted - dynamically discovered at runtime)

# Note: Anchor MAC addresses are now dynamically discovered from the first MQTT message,
#       extracting from both used_anchors and unused_anchors arrays.
#sample: anchor mqqt topic for mac adress x:str =  ANCHOR_BASE.format(x)
#      all key anchor info (ewma, x-vector, etc...) lives inside the python process
#      mqtt receives: comp. tag info: (macadress, cep95)

def create_anchor_class(anch_mac: str) -> Anchor:
    """
    Create an Anchor object by fetching anchor configuration from the API.   
    Args:
        anch_mac: MAC address of the anchor to initialize        
    Returns:
        Anchor: Configured Anchor object with position and MAC address from API        
    Raises:
        Exception: If API call fails (non-200 status code)
    """
    api_url = ANCHOR_INIT_BASE.format(anch_mac)
    anch_info = requests.get(api_url, auth=anch_api_auth)

    if anch_info.status_code != 200: 
        raise Exception(f"API call failed: {anch_info.status_code}, {anch_info.text}")

    anch_data_list = anch_info.json()
    
    # The API returns an array with one object inside
    if not anch_data_list:
        raise Exception(f"No anchor found for MAC address: {anch_mac}")
    
    anch_data = anch_data_list[0]  # Get the first (and only) anchor object

    coord: PointR3 = (anch_data["x"], anch_data["y"], anch_data["z"])

    return Anchor(anch_mac, coord)

def create_anchor_classes(anch_macs: List[str]) -> Dict[str, Anchor]:
    """
    Create multiple Anchor objects by fetching anchor configurations from the API.   
    Args:
        anch_macs: List of MAC addresses of anchors to initialize       
    Returns:
        List[Anchor]: List of configured Anchor objects with positions and MAC addresses from API        
    Raises:
        Exception: If any API call fails (non-200 status code) - will be raised by create_anchor_class()
    """
    anchors: Dict[str, Anchor] = {}
    for anch_mac in anch_macs:
        anchors[anch_mac] = create_anchor_class(anch_mac)

    return anchors

def create_tag_class(tag_data: Dict[str, Any]) -> Tag:

    #get mac:
    tag_mac: str = tag_data["tag"]["mac"]

    #get position
    position_data: Dict[str, Any] = tag_data["location"]["position"]
    tag_pos: PointR3 = (position_data["x"], position_data["y"], position_data["z"])

    #get rssi dict
    tag_rssi_dict: Dict[str, float] = {}
    used_anchors: List[Dict[str, Any]] = position_data["used_anchors"]
    for anchor_dict in used_anchors:
        amac: str = anchor_dict["mac"]
        arssi: float = anchor_dict["rssi"]
        tag_rssi_dict[amac] = arssi
    
    return Tag(tag_mac, tag_pos, tag_rssi_dict)
   
def tag_info(tag_mac: str, error_estimate: float) -> Dict[str, Any]:

    return_mess = {
        "tag_mac": tag_mac, 
        "error_estimate": error_estimate
    }

    return return_mess

def anchors_info(anch_list: List[Anchor]) -> Dict[str, Any]:

    warning_anchors: List[str] = []
    faulty_anchors: List[str] = []
    anchors_info_list: List[Dict[str, Any]] = []

    for anchor in anch_list:
        anch_info_dict: Dict[str, Any] = {}
        anch_info_dict["mac"] = anchor.macadress
        anch_info_dict["n_var"] = anchor.n
        anch_info_dict["ewma"] = anchor.ewma

        anchors_info_list.append(anch_info_dict)

        if anchor.is_warning():
            warning_anchors.append(anchor.macadress)

        if anchor.is_faulty():
            faulty_anchors.append(anchor.macadress)
        
    return_mess = {
        "anchors": anchors_info_list,
        "warning_anchors": warning_anchors,
        "faulty_anchors": faulty_anchors
    }

    return return_mess

def output_info(tag_mac: str, error_estimate: float, anch_list: List[Anchor]) -> Dict[str, Any]:
    tag_return = tag_info(tag_mac, error_estimate)
    anchors_return = anchors_info(anch_list)

    return tag_return | anchors_return
    
def on_connect(client: mqtt.Client, userdata: Any, flags: Dict[str, Any], rc: int) -> None:
    print("Connected to server", rc)
    client.subscribe(TAG_POSITION_STREAM)

def extract_anchor_macs_from_message(tag_data: Dict[str, Any]) -> List[str]:
    """
    Extract all anchor MAC addresses from a tag position message.
    
    Args:
        tag_data: The parsed JSON data from an MQTT tag position message
        
    Returns:
        List[str]: List of unique anchor MAC addresses found in the message
    """
    anchor_macs: List[str] = []
    position_data: Dict[str, Any] = tag_data["location"]["position"]
    
        # Get MACs from used anchors
    used_anchors: List[Dict[str, Any]] = position_data.get("used_anchors", [])
    unused_anchors: List[Dict[str, Any]] = position_data.get("unused_anchors", [])
    
    # Debug: Show what's actually in the message
    used_macs = [anchor["mac"] for anchor in used_anchors]
    unused_macs = [anchor["mac"] for anchor in unused_anchors]
    print(f"DEBUG - Used anchors in message: {used_macs}")
    print(f"DEBUG - Unused anchors in message: {unused_macs}")
    
    all_anchors: List[Dict[str, Any]] = used_anchors + unused_anchors
    for anchor_dict in all_anchors:
        anchor_macs.append(anchor_dict["mac"])
    
    # Return unique MAC addresses
    return list(set(anchor_macs))

def on_message(client: mqtt.Client, userdata: Any, msg: mqtt.MQTTMessage) -> None:
    try: 
        #create Tag class from inbound message and get timestamp
        payload_str: str = msg.payload.decode("utf-8")
        tag_data: Dict[str, Any] = json.loads(payload_str)
        
        # Check if this is the first message and we need to initialize anchors
        if not userdata["anchors_initialized"]:
            print("First message received - discovering and initializing anchors...")
            
            # Extract all anchor MAC addresses from this message
            discovered_anchor_macs = extract_anchor_macs_from_message(tag_data)
            print(f"Discovered anchor MACs: {discovered_anchor_macs}")
            
            # Initialize all discovered anchors
            userdata["anchors"] = create_anchor_classes(discovered_anchor_macs)
            userdata["anchors_initialized"] = True
            
            print(f"Initialized {len(userdata['anchors'])} anchors")
        
        message_tag: Tag = create_tag_class(tag_data)
        timestamp: float = tag_data["timestamp"]

        #create reduced list of used anchors (only include anchors we have initialized)
        anch_dict: Dict[str, Anchor] = userdata["anchors"]
        anch_list: List[Anchor] = []
        for anch_mac in message_tag.rssi_dict.keys():
            if anch_mac in anch_dict:  # Only add if we have this anchor initialized
                anch_list.append(anch_dict[anch_mac])
            else:
                # This shouldn't happen after initial discovery, but handle it gracefully
                print(f"Warning: Found new anchor {anch_mac} after initialization")
                new_anch = create_anchor_class(anch_mac)
                anch_dict[anch_mac] = new_anch
                anch_list.append(new_anch)

        # Only proceed if we have at least some anchors
        if anch_list:
            #create Tag system 
            message_system = TagSystem(message_tag, PathLossModel())

            #get error value:
            error_estimate = message_system.error_radius(anch_list)

            #update health, kalman, and variables:
            update_anchors_from_tag_data(anch_list, message_tag, PathLossModel(), timestamp)

            #publish outbound message:
            client.publish(
                TOPIC_OUT, 
                json.dumps(output_info(message_tag.macadress, error_estimate, anch_list))
            )
        else:
            print(f"No initialized anchors found for tag {message_tag.macadress}")
  
    except (json.JSONDecodeError, KeyError, ValueError) as e:
        print("Error parsing message:", e)


def mqtt_runner() -> None:
    # Initialize with empty anchors - they will be discovered from first message
    init_userdata = {
        "anchors": {},
        "anchors_initialized": False
    }
    
    #run mqtt client
    client = mqtt.Client(client_id=CLIENT_ID, userdata=init_userdata)
    
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(BROKER, PORT, 60)

    client.loop_forever()

if __name__ == "__main__":
    mqtt_runner()

