# HSR Demo Lab

This repository contains tools and instructions for developing programs that interface with the real Toyota Human Support Robot (HSR). It provides a development environment and examples for creating applications that can control and interact with the HSR in real-world scenarios.

## Hardware Configuration

The recommended hardware setup consists of:

1. **HSR Robot** - The Toyota Human Support Robot
2. **Ubuntu NUC** - An Intel NUC computer running the latest Ubuntu version, connected directly to the HSR via Ethernet cable
3. **Development Laptop** - Used to remotely access the NUC surface

This configuration allows the NUC to serve as an intermediate computing unit that:
- Connects to the HSR robot via wired Ethernet
- Connects to the internet via WiFi
- Can be remotely accessed from your development laptop

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

You can set up the HSR Demo Lab environment either manually on Ubuntu 20.04 or using Docker.

## Setting Up Internet Sharing on Ubuntu

To allow your HSR robot to access the internet through your computer's WiFi connection, follow these steps to set up internet sharing from your WiFi to devices connected via LAN cable.

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

Your HSR robot should now be able to access the internet through your computer's WiFi connection. It will automatically receive an IP address in the 10.42.0.x range. If not, reboot the HSR robot.

To disable sharing later:
```bash
sudo nmcli connection down "Shared LAN"
```

## Connecting to HSR Robot

### Finding the HSR Robot on the Network

Before proceeding, you need to find your HSR robot on the network:

```bash
ping hsrb.local
```

This command should return the IP address of your HSR robot. If it doesn't work, you may need to check your network configuration or connect directly to the HSR via Ethernet.

### SSH Connection to HSR Robot

To connect to the HSR robot via SSH:

```bash
ssh administrator@hsrb.local
```

When prompted, enter the administrator password: `password`

## Setting Up Time Synchronization

For applications that use tf, time synchronization between the robot and client PC is critical. Problems will occur if the time is not strictly synchronized.

### HSR Robot Configuration (Using Client Mode)

1. Log into the HSR robot using administrator permissions (if not already logged in):
   ```bash
   ssh administrator@hsrb.local
   ```

2. Change the configuration file to use the client mode:
   ```bash
   cd /etc/chrony
   sudo rm chrony.conf
   sudo ln -s chrony.conf.client chrony.conf
   ```

3. Edit the client configuration to add necessary settings:
   ```bash
   sudo nano /etc/chrony/chrony.conf.client
   ```
   
   Add these additional lines to the configuration:
   ```
   local stratum 10
   allow 10.42.0.0/16
   bindaddress 0.0.0.0
   ```
   
   Important notes about these settings:
   - `local stratum 10`: Enables the HSR robot to act as a time source when external sources are unavailable
   - `allow 10.42.0.0/16`: Permits time synchronization for devices in the specified subnet
     - This subnet mask (`10.42.0.0/16`) corresponds to all IP addresses from 10.42.0.0 to 10.42.255.255
     - If your network uses a different IP range, adjust this value accordingly
     - You can check your network's IP range by running `ip addr` on the NUC and looking at the Ethernet interface's IP
   - `bindaddress 0.0.0.0`: Binds the time server to all network interfaces

4. Restart chrony:
   ```bash
   sudo service chrony restart
   ```

### NUC Configuration

The Docker container for HSR development on the NUC already has chrony properly configured in the Dockerfile. When you run the container using the provided start script, time synchronization will be automatically set up.

If you need to check the time synchronization status inside the container:

```bash
chronyc sources
```

You should see hsrb.local listed as a synchronization source with an asterisk (*) indicating it's being used.

3. Restart chrony:
   ```bash
   sudo service chrony restart
   ```

4. Verify time synchronization with:
   ```bash
   chronyc sources
   ```
   You should see hsrb.local as a synchronized source (marked with "*"). This indicates your client PC is successfully synchronizing time with the HSR robot.

5. For more detailed synchronization, you can use the provided script:
   ```bash
   chmod +x sync_time.sh
   ./sync_time.sh
   ```

## Docker Setup for HSR Development

### Prerequisites:
- Install Docker on your Ubuntu system:
  ```bash
  sudo apt-get update
  sudo apt-get install docker.io
  sudo systemctl start docker
  sudo systemctl enable docker
  sudo usermod -aG docker $USER
  ```
  After adding yourself to the docker group, log out and back in for changes to take effect.

### Steps:
1. Clone the repository (if you haven't already):
   ```bash
   git clone https://github.com/anh0001/hsr-demo-lab.git
   cd hsr-demo-lab
   ```

2. Build the Docker image:
   ```bash
   docker build -t hsr-demo-lab .
   ```

3. **Before running the container, get the HSR robot's IP address**:
   ```bash
   ping hsrb.local
   ```
   Note down the IP address that appears (e.g., 10.42.0.129)

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
   ./start_hsr_container.sh --interface eth0 --robot-ip 10.42.0.129
   ```

5. Access the development environment:
   - From your laptop, SSH into the NUC with X11 forwarding:
     ```bash
     ssh -X username@nuc-ip-address
     ```
     
   - Open noVNC (for GUI access) from your laptop browser:
     ```bash
     firefox http://nuc-ip-address:8081/
     ```
     Enter the password: 1234
     Use the tmux terminal within noVNC for command-line operations

   - Open Jupyter Notebook from your laptop browser:
     ```bash
     firefox http://nuc-ip-address:9113/
     ```

Now you can develop and test your HSR applications on the NUC while controlling everything from your laptop.

## Manual Setup on Ubuntu 20.04

If you prefer not to use Docker, you can set up your environment manually:

1. Install ROS Noetic Desktop Full:
   ```bash
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   sudo apt update
   sudo apt install ros-noetic-desktop-full
   ```

2. Add HSR repositories:
   ```bash
   sudo sh -c 'echo "deb [arch=amd64] https://hsr-user:jD3k4G2e@packages.hsr.io/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/tmc.list'
   sudo sh -c 'echo "deb [arch=amd64] https://hsr-user:jD3k4G2e@packages.hsr.io/tmc/ubuntu `lsb_release -cs` multiverse main" >> /etc/apt/sources.list.d/tmc.list'
   sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
   ```

3. Add keys:
   ```bash
   wget https://hsr-user:jD3k4G2e@packages.hsr.io/tmc.key -O - | sudo apt-key add -
   wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -O - | sudo apt-key add -
   wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
   ```

4. Set up authentication:
   ```bash
   sudo sh -c 'mkdir -p /etc/apt/auth.conf.d'
   sudo sh -c '/bin/echo -e "machine packages.hsr.io\nlogin hsr-user\npassword jD3k4G2e" >/etc/apt/auth.conf.d/auth.conf'
   ```

5. Add package preferences:
   ```bash
   sudo sh -c '/bin/echo -e "Package: ros-noetic-laser-ortho-projector\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-laser-scan-matcher\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-laser-scan-sparsifier\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-laser-scan-splitter\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-ncd-parser\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-polar-scan-matcher\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-scan-to-cloud-converter\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-scan-tools\nPin: version 0.3.3*\nPin-Priority: 1001" > /etc/apt/preferences'
   ```

6. Update and install:
   ```bash
   sudo apt-get update
   sudo apt-get install ros-noetic-tmc-desktop-full
   ```

7. Build HSR interfaces:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws
   source /opt/ros/noetic/setup.bash
   catkin_make
   ```

## Environment Setup (for Manual Installation)

1. Edit `.bashrc`:
   ```bash
   gedit ~/.bashrc
   ```

2. Add the following to the end of `.bashrc` (replace "eno1" with your actual network interface):
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

### Running the Simulator

```bash
roslaunch hsrb_gazebo_launch hsrb_megaweb2015_world.launch
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

## Troubleshooting and Common Issues

### Network Connection Issues

If you can't connect to the HSR robot:
1. Check that the Ethernet cable between the NUC and HSR robot is properly connected
2. Verify that Internet Sharing is enabled on the NUC
3. Try restarting the HSR robot
4. Check if the HSR robot has received an IP address in the 10.42.0.x range
5. Verify you can ping the HSR from the NUC: `ping hsrb.local`

If you can't connect to the NUC from your laptop:
1. Make sure both devices are on the same WiFi network
2. Verify the NUC's IP address using `ip addr` on the NUC
3. Check firewall settings on the NUC (SSH port 22 should be open)
4. Try using the IP address directly instead of hostname

### Time Synchronization Issues

If you experience tf transformation errors or other timing issues:
1. Verify chrony is running on both your PC and the HSR robot
2. Check the synchronization status with `chronyc sources`
3. Try running the sync_time.sh script again
4. Restart the HSR robot if necessary

### Container Access Issues

If you can't access noVNC or Jupyter from the container:
1. Verify the container is running with `docker ps`
2. Check if the ports are properly forwarded
3. Try restarting the container with `docker restart hsr_container`

## Credentials and Important Information

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

## Uninstalling ROS HSR

To uninstall all HSR-related packages:
```bash
sudo apt-get purge ros-noetic-hsr* ros-noetic-tmc-*
sudo rm /etc/apt/sources.list.d/tmc.list*
sudo apt-get update
sudo apt-key del tmc.key
```