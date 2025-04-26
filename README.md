# HSR Demo Lab

This repository contains tools and instructions for developing programs that interface with the real Toyota Human Support Robot (HSR). It provides a development environment and examples for creating applications that can control and interact with the HSR in real-world scenarios.

## Repository Description

The HSR Demo Lab is designed to facilitate the development of software for the Toyota HSR. It includes:

- Setup instructions for the development environment
- Guidelines for connecting to and communicating with the physical HSR
- Examples and tools for common HSR operations
- Simulator setup for testing without the physical robot

This repository serves as a central resource for researchers, developers, and students working on projects involving the Toyota HSR, enabling them to quickly set up their development environment and start creating applications for the robot.

## Cloning This Repository

To clone this repository:

```bash
git clone https://github.com/anh0001/hsr-demo-lab.git
cd hsr-demo-lab
```

## Getting Started

You can set up the HSR Demo Lab environment either manually on Ubuntu 20.04 or using Docker (recommended for macOS users).

### Option 1: Manual Setup on Ubuntu 20.04

1. Clone this repo and navigate to the directory:
   ```bash
   git clone https://github.com/anh0001/hsr-demo-lab.git
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

8. Build HSR interfaces:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws
   source /opt/ros/noetic/setup.bash
   catkin_make
   ```

### Option 2: Docker Setup (Recommended for macOS)

This option is particularly useful for developers using macOS laptops.

#### Prerequisites:
- Install Docker Desktop on your macOS system
- Ensure Rosetta 2 is installed (required for M1/M2 Macs)
- In Docker Desktop settings, activate the "Use Rosetta for x86/amd64 emulation on Apple Silicon" option

#### Steps:
1. Clone the repository:
   ```bash
   git clone https://github.com/anh0001/hsr-demo-lab.git
   cd hsr-demo-lab
   ```

2. Build the Docker image:
   ```bash
   docker build --platform linux/amd64 -t hsr-demo-lab .
   ```

3. **Before running the container, get the HSR robot's IP address**:
   ```bash
   ping hsrb.local
   ```
   Note down the IP address that appears (e.g., 169.254.4.231)

4. Run the Docker container using the provided start script:
   ```bash
   chmod +x start_hsr_container.sh
   ./start_hsr_container.sh
   ```

   The script will automatically:
   - Use the default network interface (`enp1s0`) or detect an appropriate Ethernet interface
   - Set up proper network connectivity with the HSR robot
   - Mount the current directory to access your code inside the container

   You can also specify a different network interface or robot IP address:
   ```bash
   ./start_hsr_container.sh --interface eth0 --robot-ip 169.254.4.231
   ```

5. Access the development environment:
   - Open noVNC (for GUI access):
     ```bash
     open http://localhost:8081/
     ```
     Enter the password: 1234
     Use the tmux terminal within noVNC for command-line operations

   - Open Jupyter Notebook:
     ```bash
     open http://localhost:9113/
     ```

Now you can develop and test your HSR applications within this containerized environment.

## Setting Up Internet Sharing (for Ubuntu 22.04)

To allow your HSR robot to access the internet through your computer's WiFi connection, you can set up internet sharing from your WiFi to devices connected via LAN cable.

### Internet Sharing Using NetworkManager (Terminal Method)

1. First, identify your network interfaces:
   ```bash
   nmcli device status
   ```
   Note your WiFi interface name (likely "wlan0" or similar) and your Ethernet interface name (likely "eth0", "enp3s0" or similar).

2. Create a shared connection for your Ethernet interface (replace "eth0" with your actual Ethernet interface name):
   ```bash
   sudo nmcli connection add type ethernet ifname eth0 ipv4.method shared connection.autoconnect yes connection.id "Shared LAN"
   ```

3. Activate the connection:
   ```bash
   sudo nmcli connection up "Shared LAN"
   ```

This configuration will:
- Set up your Ethernet interface with a static IP (typically 10.42.0.1)
- Configure DHCP so connected devices (including the HSR) will automatically receive IP addresses
- Establish NAT (Network Address Translation) to route traffic between your WiFi and LAN devices

Your HSR robot should now be able to access the internet through your computer's WiFi connection. It will automatically receive an IP address in the 10.42.0.x range. If not reboot the HSR robot.

To disable sharing later:
```bash
sudo nmcli connection down "Shared LAN"
```

## Setting Up Local Link Connection to HSR (Terminal Method)

To establish a direct wired connection to the HSR without needing a DHCP server, you can set up a link-local connection using terminal commands:

### PC Side Configuration (Ubuntu 20.04):

1. Identify your Ethernet interface name:
   ```bash
   nmcli device status | grep ethernet
   ```
   Look for the interface in the left column (e.g., `enp1s0`).

2. Create a link-local connection using nmcli:
   ```bash
   sudo nmcli connection add \
     type ethernet \
     con-name link-local \
     ifname <your_interface> \
     autoconnect yes \
     ipv4.method link-local \
     ipv6.method ignore
   ```

3. Activate the link-local connection:
   ```bash
   sudo nmcli connection up link-local
   ```

4. Verify the configuration on your interface:
   ```bash
   ip addr show <your_interface>
   ```
   You should see an IPv4 address in the 169.254.x.x range.

5. Test connectivity to the HSR:
   ```
   ping hsrb.local
   ```

### Connection Verification and Credentials

- To check the connection to the HSR or find its IP address:
  ```
  ping hsrb.local
  ```

- HSR administrator login:
  ```
  Username: administrator
  Password: password
  ```

- HSR repository credentials:
  ```
  Username: hsr-user
  Password: jD3k4G2e
  ```

## Environment Setup (for Manual Installation)

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

## Time Synchronization

For applications that use tf, time synchronization between the robot and client PC is critical. Problems will occur if the time is not strictly synchronized.

### Robot Side Configuration

#### When a Time Synchronization Server is Available:

1. Log into the robot using administrator permissions:
   ```bash
   ssh administrator@hsrb.local
   ```
   When prompted, enter the administrator password.

2. Change the configuration file:
   ```bash
   cd /etc/chrony
   sudo rm chrony.conf
   sudo ln -s chrony.conf.client chrony.conf
   ```

3. Synchronization server configuration:
   The default configuration uses ntp.nict.jp. You can edit the `/etc/chrony/chrony.conf.client` file if you need to use a different time server.

4. Manual time synchronization:
   ```bash
   sudo ntpdate <time_synchronization_server>
   ```
   Replace `<time_synchronization_server>` with your server address.

5. Reboot the robot:
   Turn off the power by long-pressing the power button, then turn on the power by long-pressing again.

#### When a Time Synchronization Server is Unavailable:

In this scenario, the HSR robot itself acts as the time synchronization server.

1. Log into the robot using administrator permissions:
   ```bash
   ssh administrator@hsrb.local
   ```

2. Change the configuration file:
   ```bash
   cd /etc/chrony
   sudo rm chrony.conf
   sudo ln -s chrony.conf.isolate chrony.conf
   ```

3. Restart chrony:
   ```bash
   sudo service chrony restart
   ```

### Client PC Configuration

#### Docker Setup Users
For users using the provided Docker setup, time synchronization is already configured in the Dockerfile. No additional setup is required.

#### Manual Setup Users
If you're setting up your environment manually:

1. When a time synchronization server is available:
   - Configure your client PC to use the appropriate time synchronization server for your network.

2. When a time synchronization server is unavailable:
   - Install chrony: `sudo apt-get install chrony`
   - Configure chrony to use the HSR as the time source (see the Dockerfile in this repository for the exact configuration)
   - Restart chrony: `sudo service chrony restart`

#### Verifying Time Synchronization
For both Docker and manual setups, verify the synchronization with:
```bash
chronyc sources
```
You should see hsrb.local as a synchronized source (marked with "*"). This indicates your client PC is successfully synchronizing time with the HSR robot.

## Running the Simulator

```bash
roslaunch hsrb_gazebo_launch hsrb_megaweb2015_world.launch
```

## Installing Jupyter Notebook (for Manual Installation)

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

## Working with the HSR

### Marker Recognition

Enable marker recognition:
```bash
rosservice call /marker/start_recognition "{}"
```

### RViz Visualization

Launch RViz with HSR configuration:
```bash
rosrun rviz rviz -d $(rospack find hsrb_common_launch)/config/hsrb_display_full_hsrb.rviz
```

### Running Example Notebooks

The repository includes several Jupyter notebooks that demonstrate different capabilities:
- `demo_grab_bottle.ipynb`: Demonstrates how to use the HSR to grab a bottle
- `2024-08-19_grab_bottle_and_throw_demo.ipynb`: Shows a complete demo of grabbing a bottle and disposing of it
- `2025-04-24_minos_demo.ipynb`: Recent demo notebook with additional capabilities

To run a notebook:
1. Start the HSR container or set up your environment
2. Navigate to the Jupyter interface (http://localhost:9113/)
3. Open the notebooks directory
4. Select and run the desired notebook

## Uninstalling ROS HSR

To uninstall all HSR-related packages:
```bash
sudo apt-get purge ros-noetic-hsr* ros-noetic-tmc-*
sudo rm /etc/apt/sources.list.d/tmc.list*
sudo apt-get update
sudo apt-key del tmc.key
```