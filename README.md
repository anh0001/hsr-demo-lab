# HSR Demo Lab

This repository contains tools and instructions for developing programs that interface with the real Toyota Human Support Robot (HSR). It provides a development environment and examples for creating applications that can control and interact with the HSR in real-world scenarios.

## Repository Description

The HSR Demo Lab is designed to facilitate the development of software for the Toyota HSR. It includes:

- Setup instructions for the development environment
- Guidelines for connecting to and communicating with the physical HSR
- Examples and tools for common HSR operations
- Simulator setup for testing without the physical robot

This repository serves as a central resource for researchers, developers, and students working on projects involving the Toyota HSR, enabling them to quickly set up their development environment and start creating applications for the robot.

## Getting Started

1. Clone this repo and navigate to the directory:
   ```
   git clone https://github.com/your-username/hsr-demo-lab.git
   cd hsr-demo-lab
   ```

2. Install ROS Noetic Desktop Full

3. Add HSR repositories:
   ```bash
   sudo sh -c 'echo "deb [arch=amd64] https://hsr-user:jD3k4G2e@packages.hsr.io/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/tmc.list'
   sudo sh -c 'echo "deb [arch=amd64] https://hsr-user:jD3k4G2e@packages.hsr.io/tmc/ubuntu `lsb_release -cs` multiverse main" >> /etc/apt/sources.list.d/tmc.list'
   sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
   ```

4. Add keys:
   ```bash
   wget https://hsr-user:jD3k4G2e@packages.hsr.io/tmc.key -O - | sudo apt-key add -
   wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -O - | sudo apt-key add -
   wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
   ```

5. Set up authentication:
   ```bash
   sudo sh -c 'mkdir -p /etc/apt/auth.conf.d'
   sudo sh -c '/bin/echo -e "machine packages.hsr.io\nlogin hsr-user\npassword jD3k4G2e" >/etc/apt/auth.conf.d/auth.conf'
   ```

6. Add package preferences:
   ```bash
   sudo sh -c '/bin/echo -e "Package: ros-noetic-laser-ortho-projector\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-laser-scan-matcher\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-laser-scan-sparsifier\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-laser-scan-splitter\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-ncd-parser\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-polar-scan-matcher\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-scan-to-cloud-converter\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-scan-tools\nPin: version 0.3.3*\nPin-Priority: 1001" > /etc/apt/preferences'
   ```

7. Update and install:
   ```bash
   sudo apt-get update
   sudo apt-get install ros-noetic-tmc-desktop-full
   ```

## Environment Setup

1. Edit `.bashrc`:
   ```bash
   gedit ~/.bashrc
   ```

2. Add the following to the end of `.bashrc`:
   ```bash
   # please set network-interface
   network_if=eno1

   if [ -e /opt/ros/noetic/setup.bash ] ; then
       source /opt/ros/noetic/setup.bash
   else
       echo "ROS packages are not installed."
   fi

   export TARGET_IP=$(LANG=C /sbin/ip address show $network_if | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*')
   if [ -z "$TARGET_IP" ] ; then
       echo "ROS_IP is not set."
   else
       export ROS_IP=$TARGET_IP
   fi

   export ROS_HOME=~/.ros
   alias sim_mode='export ROS_MASTER_URI=http://localhost:11311 export PS1="\[\033[44;1;37m\]<local>\[\033[0m\]\w$ "'
   alias hsrb_mode='export ROS_MASTER_URI=http://hsrb.local:11311 export PS1="\[\033[41;1;37m\]<hsrb>\[\033[0m\]\w$ "'
   ```

## Running the Simulator

```bash
roslaunch hsrb_gazebo_launch hsrb_megaweb2015_world.launch
```

## Time Synchronization

### When a Time Synchronization Server is Available

#### Robot Configuration:
1. SSH into the robot:
   ```bash
   ssh administrator@hsrb.local
   ```
2. Change configuration:
   ```bash
   cd /etc/chrony
   sudo rm chrony.conf
   sudo ln -s chrony.conf.client chrony.conf
   ```
3. Manual time synchronization:
   ```bash
   sudo ntpdate {{ Time synchronization server }}
   ```
4. Reboot the robot

#### Client PC Configuration:
Configure appropriately to match the network, time synchronization server, and client.

### When a Time Synchronization Server is Unavailable

#### Robot Configuration:
1. SSH into the robot:
   ```bash
   ssh administrator@hsrb.local
   ```
2. Change configuration:
   ```bash
   cd /etc/chrony
   sudo rm chrony.conf
   sudo ln -s chrony.conf.isolate chrony.conf
   ```
3. Restart chrony:
   ```bash
   sudo service chrony restart
   ```

#### Client PC Configuration:
1. Install chrony:
   ```bash
   sudo apt-get install chrony
   ```
2. Configure chrony:
   ```bash
   sudo mv /etc/chrony/chrony.conf /etc/chrony/chrony.conf.orig
   sudo gedit /etc/chrony/chrony.conf
   ```
3. Add the following settings:
   ```
   server hsrb.local
   driftfile /var/lib/chrony/chrony.drift
   keyfile /etc/chrony/chrony.keys
   generatecommandkey
   log tracking measurements statistics
   logdir /var/log/chrony
   local stratum 10
   allow hsrb.local
   logchange 0.5
   initstepslew 20 hsrb.local
   ```
4. Restart chrony:
   ```bash
   sudo service chrony restart
   ```

## Installing Jupyter Notebook

1. Install Python and pip:
   ```bash
   sudo apt update
   sudo apt install python3-pip python3-dev
   ```

2. Create a Python Virtual Environment:
   ```bash
   sudo -H pip3 install virtualenv
   virtualenv --system-site-packages hsr_env
   source hsr_env/bin/activate
   ```

3. Install Jupyter:
   ```bash
   pip install jupyter
   ```

4. Run Jupyter Notebook:
   ```bash
   jupyter notebook
   ```
   Access it via `http://localhost:8888` in your web browser.

## Marker Recognition

Enable marker recognition:
```bash
rosservice call /marker/start_recognition "{}"
```

## RViz Visualization

Launch RViz with HSR configuration:
```bash
rosrun rviz rviz -d $(rospack find hsrb_common_launch)/config/hsrb_display_full_hsrb.rviz
```

## Uninstalling ROS HSR

To uninstall all HSR-related packages:
```bash
sudo apt-get purge ros-noetic-hsr* ros-noetic-tmc-*
sudo rm /etc/apt/sources.list.d/tmc.list*
sudo apt-get update
sudo apt-key del tmc.key
```