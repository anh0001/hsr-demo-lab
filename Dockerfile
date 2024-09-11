# Use Ubuntu 20.04 AMD64 as the base image
FROM ubuntu:20.04

# Prevents errors during package installations
ARG DEBIAN_FRONTEND=noninteractive

# Update and install necessary tools
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg2 \
    curl \
    net-tools \
    git \
    dbus-x11 \
    sudo \
    ca-certificates \
    x11vnc \
    xvfb \
    twm \
    xterm \
    python3-pip \
    python3-dev \
    tmux \
    wget \
    chrony \
    && rm -rf /var/lib/apt/lists/*

# Add ROS Noetic sources
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Add HSR repositories
RUN sh -c 'echo "deb [arch=amd64] https://hsr-user:jD3k4G2e@packages.hsr.io/ros/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/tmc.list' && \
    sh -c 'echo "deb [arch=amd64] https://hsr-user:jD3k4G2e@packages.hsr.io/tmc/ubuntu $(lsb_release -cs) multiverse main" >> /etc/apt/sources.list.d/tmc.list' && \
    sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'

# Add keys
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    wget https://hsr-user:jD3k4G2e@packages.hsr.io/tmc.key -O - | apt-key add - && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

# Set up authentication
RUN mkdir -p /etc/apt/auth.conf.d && \
    echo -e "machine packages.hsr.io\nlogin hsr-user\npassword jD3k4G2e" > /etc/apt/auth.conf.d/auth.conf

# Install ROS Noetic Desktop-Full and HSR packages
RUN apt-get update && \
    apt-get install -y ros-noetic-desktop-full && \
    apt-get install -y ros-noetic-tmc-desktop-full && \
    rm -rf /var/lib/apt/lists/*

# Install and initialize rosdep
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init \
    && rosdep update

# Set up environment
RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc

# Add HSR-specific environment setup to .bashrc
RUN echo '# please set network-interface' >> /root/.bashrc && \
    echo 'network_if=eno1' >> /root/.bashrc && \
    echo 'if [ -e /opt/ros/noetic/setup.bash ] ; then' >> /root/.bashrc && \
    echo '    source /opt/ros/noetic/setup.bash' >> /root/.bashrc && \
    echo 'else' >> /root/.bashrc && \
    echo '    echo "ROS packages are not installed."' >> /root/.bashrc && \
    echo 'fi' >> /root/.bashrc && \
    echo 'export TARGET_IP=$(LANG=C /sbin/ip address show $network_if | grep -Eo '"'"'inet (addr:)?([0-9]*\.){3}[0-9]*'"'"' | grep -Eo '"'"'([0-9]*\.){3}[0-9]*'"'"')' >> /root/.bashrc && \
    echo 'if [ -z "$TARGET_IP" ] ; then' >> /root/.bashrc && \
    echo '    echo "ROS_IP is not set."' >> /root/.bashrc && \
    echo 'else' >> /root/.bashrc && \
    echo '    export ROS_IP=$TARGET_IP' >> /root/.bashrc && \
    echo 'fi' >> /root/.bashrc && \
    echo 'export ROS_HOME=~/.ros' >> /root/.bashrc && \
    echo 'alias sim_mode='"'"'export ROS_MASTER_URI=http://localhost:11311 export PS1="\[\033[44;1;37m\]<local>\[\033[0m\]\w$ "'"'"'' >> /root/.bashrc && \
    echo 'alias hsrb_mode='"'"'export ROS_MASTER_URI=http://hsrb.local:11311 export PS1="\[\033[41;1;37m\]<hsrb>\[\033[0m\]\w$ "'"'"'' >> /root/.bashrc

# Configure chrony
RUN mv /etc/chrony/chrony.conf /etc/chrony/chrony.conf.orig && \
    echo "server hsrb.local" > /etc/chrony/chrony.conf && \
    echo "driftfile /var/lib/chrony/chrony.drift" >> /etc/chrony/chrony.conf && \
    echo "keyfile /etc/chrony/chrony.keys" >> /etc/chrony/chrony.conf && \
    echo "generatecommandkey" >> /etc/chrony/chrony.conf && \
    echo "log tracking measurements statistics" >> /etc/chrony/chrony.conf && \
    echo "logdir /var/log/chrony" >> /etc/chrony/chrony.conf && \
    echo "local stratum 10" >> /etc/chrony/chrony.conf && \
    echo "allow hsrb.local" >> /etc/chrony/chrony.conf && \
    echo "logchange 0.5" >> /etc/chrony/chrony.conf && \
    echo "initstepslew 20 hsrb.local" >> /etc/chrony/chrony.conf

# Install Jupyter Notebook
RUN pip3 install --upgrade pip && \
    pip3 install virtualenv && \
    virtualenv --system-site-packages /root/hsr_env && \
    . /root/hsr_env/bin/activate && \
    pip install jupyter

# Set up Jupyter configuration directory
RUN mkdir -p /root/.jupyter

# Install noVNC
RUN git clone https://github.com/novnc/noVNC.git /opt/novnc \
    && git clone https://github.com/novnc/websockify /opt/novnc/utils/websockify \
    && ln -s /opt/novnc/vnc.html /opt/novnc/index.html

# Set up VNC password
RUN mkdir ~/.vnc && x11vnc -storepasswd 1234 ~/.vnc/passwd

# Copy startup script
COPY startup.sh /startup.sh
RUN chmod +x /startup.sh

# Set default working directory
WORKDIR /root

# Expose the ports for VNC, noVNC, and Jupyter Notebook
EXPOSE 5900 8080 8888

# Run the startup script
CMD ["/startup.sh"]