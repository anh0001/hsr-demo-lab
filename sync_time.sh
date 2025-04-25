#!/bin/bash
set -e

echo "Fetching current time from HSR robot..."
HSR_TIME=$(ssh administrator@hsrb.local 'date +"%Y-%m-%d %H:%M:%S"')

if [ -z "$HSR_TIME" ]; then
    echo "Failed to get time from HSR robot. Check SSH connection."
    exit 1
fi

echo "HSR robot time is: $HSR_TIME"
echo "Setting container time to match HSR..."

# Set the system time to match HSR
sudo date -s "$HSR_TIME"

echo "Restarting chrony service..."
sudo service chrony restart

# Force immediate time synchronization
echo "Forcing chrony to step the time..."
sudo chronyc makestep

echo "Time synchronization complete."
echo "Current container time: $(date)"

# Verify time difference
echo "Verifying time difference with HSR..."
HSR_CURRENT=$(ssh administrator@hsrb.local 'date +%s')
CONTAINER_CURRENT=$(date +%s)
DIFF=$((HSR_CURRENT - CONTAINER_CURRENT))

echo "Time difference is now: $DIFF seconds"

if [ ${DIFF#-} -lt 5 ]; then
    echo "Time synchronized successfully!"
else
    echo "Time difference still greater than 5 seconds. May need further adjustment."
fi