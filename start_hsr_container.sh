#!/bin/bash

# Script to start the HSR Docker container with proper configuration

# Default values
HSR_IP=169.254.4.231
IMAGE_NAME=hsr-demo-lab
NETWORK_IF="enp1s0"  # Default to enp1s0

# Display usage information
function show_usage {
    echo "Usage: $0 [OPTIONS]"
    echo "Options:"
    echo "  -i, --interface INTERFACE  Specify network interface to use (default: enp1s0)"
    echo "  -r, --robot-ip IP          Specify HSR robot IP address (default: 169.254.4.231)"
    echo "  -n, --image-name NAME      Specify Docker image name (default: hsr-demo-lab)"
    echo "  -h, --help                 Show this help message"
    echo ""
    echo "Example:"
    echo "  $0 --interface eth0 --robot-ip 169.254.4.231"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        -i|--interface)
            NETWORK_IF="$2"
            shift 2
            ;;
        -r|--robot-ip)
            HSR_IP="$2"
            shift 2
            ;;
        -n|--image-name)
            IMAGE_NAME="$2"
            shift 2
            ;;
        -h|--help)
            show_usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

# Check if the default interface exists
if ! ip link show "$NETWORK_IF" &>/dev/null; then
    echo "Default interface $NETWORK_IF not found, attempting to auto-detect..."
    NETWORK_IF=$(ip -o -4 link | grep -E 'eth|en|eno|ens|enp' | awk '{print $2}' | sed 's/://' | head -1)
    
    if [ -z "$NETWORK_IF" ]; then
        echo "No Ethernet interface detected. Please specify one with --interface."
        echo "Available interfaces:"
        ip -o -4 link | grep -v "lo" | awk '{print $2}' | sed 's/://'
        exit 1
    fi
    
    echo "Detected Ethernet interface: $NETWORK_IF"
fi

# Get interface IP address
INTERFACE_IP=$(ip -o -4 addr show $NETWORK_IF | grep -Eo '([0-9]{1,3}\.){3}[0-9]{1,3}' | head -1)
if [ -z "$INTERFACE_IP" ]; then
    echo "Could not determine IP address for interface $NETWORK_IF"
    exit 1
fi
echo "Interface $NETWORK_IF has IP: $INTERFACE_IP"

#
# Remove existing container if it exists
existing_container=$(docker ps -aq -f name=hsr_container)
if [ -n "$existing_container" ]; then
    echo "Removing existing container hsr_container..."
    docker rm -f hsr_container
fi
# Start container
echo "Starting HSR Docker container..."
echo "- Network interface: $NETWORK_IF"
echo "- Robot IP: $HSR_IP"
echo "- Local IP: $INTERFACE_IP"

docker run --platform linux/amd64 \
    --network=host \
    --cap-add SYS_TIME \
    -e NETWORK_INTERFACE=$NETWORK_IF \
    -e ROS_IP=$INTERFACE_IP \
    --add-host=hsrb.local:$HSR_IP \
    -v "$(pwd)":/root/hsr-demo-lab/ \
    --name hsr_container \
    -it $IMAGE_NAME

# Note: --network=host gives direct access to host network interfaces
# If you prefer port mapping instead, replace --network=host with:
# -p 5900:5900 -p 8081:8081 -p 9113:9113 -p 11311:11311