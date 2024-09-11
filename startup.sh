#!/bin/bash

set -e

# Start system bus
sudo /etc/init.d/dbus start

# Start Xvfb
Xvfb :1 -screen 0 1024x768x16 &
sleep 5

# Export display for X applications
export DISPLAY=:1

# Start a minimal window manager
twm &

# Start X11VNC
x11vnc -forever -usepw -create -display :1 &
sleep 10

# Start noVNC
/opt/novnc/utils/novnc_proxy --vnc localhost:5900 --listen 0.0.0.0:8080 &

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
c.NotebookApp.port = 8888
c.NotebookApp.open_browser = False
c.NotebookApp.token = ''
c.NotebookApp.password = ''
EOT

# Start Jupyter Notebook
source /root/hsr_env/bin/activate
jupyter notebook --allow-root --config=/root/.jupyter/jupyter_notebook_config.py &

# Keep the container running
tail -f /dev/null