#!/bin/bash

set -e

# Start system bus
sudo /etc/init.d/dbus start

# Start Xvfb with more clear logging and error checking
echo "Starting Xvfb..."
Xvfb :99 -screen 0 1024x768x16 &
XVFB_PID=$!

# Wait for Xvfb to be ready
echo "Waiting for Xvfb to start..."
sleep 5
if ! ps -p $XVFB_PID > /dev/null; then
    echo "Xvfb failed to start"
    exit 1
fi

# Set DISPLAY environment variable
export DISPLAY=:99
echo "Display is set to $DISPLAY"

# Verify X server is working
if ! xdpyinfo >/dev/null 2>&1; then
    echo "ERROR: X server at display $DISPLAY is not working"
    exit 1
fi
echo "X server is working properly"

# Start a minimal window manager
echo "Starting window manager..."
twm &
sleep 2

# Start X11VNC with verbose logging
echo "Starting x11vnc..."
x11vnc -display :99 -forever -usepw -create -v &
sleep 2

# Start noVNC - use 127.0.0.1 instead of localhost
echo "Starting noVNC proxy..."
/opt/novnc/utils/novnc_proxy --vnc 127.0.0.1:5900 --listen 0.0.0.0:8081 &

# Create a script to run in xterm that ensures tmux is started
cat << EOF > /root/start_tmux.sh
#!/bin/bash
if tmux has-session -t ros_session 2>/dev/null; then
    tmux attach-session -t ros_session
else
    tmux new-session -s ros_session
fi
EOF
chmod +x /root/start_tmux.sh

# Start a terminal with tmux
xterm -e "/root/start_tmux.sh" &

# Source ROS setup
echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Start chrony
sudo service chrony start

# Configure Jupyter Notebook
mkdir -p /root/.jupyter
cat <<EOT > /root/.jupyter/jupyter_notebook_config.py
c = get_config()
c.NotebookApp.ip = '0.0.0.0'
c.NotebookApp.port = 9113
c.NotebookApp.open_browser = False
c.NotebookApp.token = ''
c.NotebookApp.password = ''
EOT

# Start Jupyter Notebook with ROS env
source /opt/ros/noetic/setup.bash
export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH
source /root/hsr_env/bin/activate
jupyter notebook --allow-root --config=/root/.jupyter/jupyter_notebook_config.py &

# Keep the container running
tail -f /dev/null