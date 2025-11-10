#!/bin/bash

# Activate conda environment
source activate go2_rc

# Change to project directory
cd /home/unitree/projects/ModelServer/

# Start both scripts in background processes
echo "Starting go2_foxy.py in background..."
python scripts/quickstart_server/go2_foxy.py &
FOXY_PID=$!

source /opt/ros/noetic/setup.bash
echo "Starting go2_noetic.py in background..."
python scripts/quickstart_server/go2_noetic.py &
NOETIC_PID=$!

echo "Both processes started:"
echo "go2_noetic.py PID: $NOETIC_PID"
echo "go2_foxy.py PID: $FOXY_PID"

# Function to handle cleanup on script termination
cleanup() {
    echo "Stopping processes..."
    kill $NOETIC_PID $FOXY_PID 2>/dev/null
    echo "Processes stopped."
    exit 0
}

# Set up signal handlers for graceful shutdown
trap cleanup SIGINT SIGTERM

echo "Press Ctrl+C to stop both processes"

# Wait for both background processes
wait $NOETIC_PID $FOXY_PID
