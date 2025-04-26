#!/bin/bash
set -e

# HSR robot credentials
HSR_USER="administrator"
HSR_PASSWORD="password"
HSR_HOST="hsrb.local"

echo "========================================"
echo "HSR Robot Time Synchronization Utility"
echo "========================================"

# Check if HSR is reachable
echo "Checking connection to HSR robot..."
if ! ping -c 1 -W 2 $HSR_HOST >/dev/null 2>&1; then
    echo "ERROR: Cannot reach HSR robot at $HSR_HOST"
    echo "Check network connection and hostname resolution"
    exit 1
fi
echo "HSR robot is reachable"

# Get HSR current time
echo "Fetching current time from HSR robot..."
HSR_TIME=$(sshpass -p "$HSR_PASSWORD" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 $HSR_USER@$HSR_HOST 'date +"%Y-%m-%d %H:%M:%S.%N"')

if [ -z "$HSR_TIME" ]; then
    echo "ERROR: Failed to get time from HSR robot. Check SSH connection."
    exit 1
fi

CONTAINER_TIME=$(date +"%Y-%m-%d %H:%M:%S.%N")
echo "HSR robot time    : $HSR_TIME"
echo "Container time    : $CONTAINER_TIME"

# Calculate time difference before sync
HSR_SECONDS=$(date -d "$HSR_TIME" +%s.%N)
CONTAINER_SECONDS=$(date -d "$CONTAINER_TIME" +%s.%N)
DIFF=$(echo "$HSR_SECONDS - $CONTAINER_SECONDS" | bc)
echo "Time difference   : $DIFF seconds"

# Check if chrony is running on HSR
echo "Checking chrony status on HSR robot..."
CHRONY_STATUS=$(sshpass -p "$HSR_PASSWORD" ssh -o StrictHostKeyChecking=no $HSR_USER@$HSR_HOST "echo '$HSR_PASSWORD' | sudo -S systemctl is-active chrony")
if [ "$CHRONY_STATUS" != "active" ]; then
    echo "WARNING: Chrony is not active on HSR robot ($CHRONY_STATUS)"
    echo "Attempting to start chrony on HSR..."
    sshpass -p "$HSR_PASSWORD" ssh -o StrictHostKeyChecking=no $HSR_USER@$HSR_HOST "echo '$HSR_PASSWORD' | sudo -S systemctl restart chrony"
fi

# Configure local chrony to use HSR as time source
echo "Configuring local chrony to use HSR as time source..."
sudo service chrony stop
echo "# Use HSR robot as time source" | sudo tee /etc/chrony/chrony.conf > /dev/null
echo "server hsrb.local iburst" | sudo tee -a /etc/chrony/chrony.conf > /dev/null
echo "driftfile /var/lib/chrony/chrony.drift" | sudo tee -a /etc/chrony/chrony.conf > /dev/null
echo "keyfile /etc/chrony/chrony.keys" | sudo tee -a /etc/chrony/chrony.conf > /dev/null
echo "commandkey 1" | sudo tee -a /etc/chrony/chrony.conf > /dev/null
echo "log tracking measurements statistics" | sudo tee -a /etc/chrony/chrony.conf > /dev/null
echo "logdir /var/log/chrony" | sudo tee -a /etc/chrony/chrony.conf > /dev/null
echo "maxupdateskew 100.0" | sudo tee -a /etc/chrony/chrony.conf > /dev/null
echo "logchange 0.5" | sudo tee -a /etc/chrony/chrony.conf > /dev/null
echo "makestep 1.0 3" | sudo tee -a /etc/chrony/chrony.conf > /dev/null

# Restart chrony service
echo "Restarting chrony service..."
sudo service chrony restart

# Wait for chrony to connect to HSR
echo "Waiting for chrony to connect to HSR robot (10 seconds)..."
sleep 10

# Force immediate time synchronization
echo "Forcing chrony to step the time..."
sudo chronyc makestep > /dev/null

# Check time sync status
echo "Checking time synchronization status..."
SYNC_STATUS=$(chronyc tracking | grep "System time" | awk '{print $4 " " $5}')
echo "Chrony sync status: $SYNC_STATUS"

SOURCES_STATUS=$(chronyc sources | grep "hsrb.local")
echo "Source status: $SOURCES_STATUS"

# Verify final time difference
echo "Verifying final time difference with HSR..."
HSR_CURRENT=$(sshpass -p "$HSR_PASSWORD" ssh -o StrictHostKeyChecking=no $HSR_USER@$HSR_HOST 'date +%s.%N')
CONTAINER_CURRENT=$(date +%s.%N)
FINAL_DIFF=$(echo "$HSR_CURRENT - $CONTAINER_CURRENT" | bc)

echo "Final time difference: $FINAL_DIFF seconds"

if (( $(echo "${FINAL_DIFF#-} < 0.5" | bc -l) )); then
    echo "✓ Time synchronized successfully!"
else
    echo "⚠ Time difference still greater than 0.5 seconds."
    echo "  You may need to rerun this script or check system time settings."
fi

echo "========================================"
echo "Time sync completed at $(date)"
echo "========================================"