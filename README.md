# Waypoint Navigation of Quadcopter in PX4-ROS

This repository provides a streamlined devcontainer to perform waypoint nagivation of quadcopters in ROS2, Gazebo and PX4.

### Software Stack

* ROS2 Humble
* Gazebo Harmonic
* PX4
* QGroundControl

### Trajectory of Quadrotor Navigation

![Alt text](images/sim_results/QGroundControl.png "QGroundControl")

![Alt text](images/sim_results/Gazebo.png "Gazebo-Quadcopter")

## **Setup**

### **1. Clone the Repository**

First, clone the project:


```bash
git clone https://github.com/Srindot/assignment2_Introduction_To_UAV_Design.git
```

### **2. Setup Docker**

Ensure Docker is installed and configured on your device.

### **3. Open the Project in VS Code**

* Launch  **Visual Studio Code (VS Code)** .
* Open the cloned project folder.

### **4. Packages Dependencies**

#### ***1. Install xorg-xhost***

***For arch:***
```bash
 sudo pacman -S xorg-xhost
 ```

***For Ubuntu:***
 ```bash
 sudo apt update && sudo apt install x11-xserver-utils
```


### ***2.Install Nvidia Container toolkit:***

> **If your host system doesn't have a Nvidia Card, remove the instance of gpus in [devcontainer](.devcontainer/devcontainer.json) and you can ignore the below instruction to download Nvidia Container Toolkit. (Follow the comments in the [devcontainer](.devcontainer/devcontainer.json) file)**

For Arch:
 ```bash
 sudo pacman -S nvidia-container-toolkit
```
and restart the docker daemon 
```bash 
sudo systemctl restart docker
```

For Ubuntu:
Set up the Repository
```bash 
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```
Then install the nvidia-contianer-toolkit
```bash
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
```
After this restart the docker daemon:
```bash
sudo systemctl restart docker
```

### **5. Build the Container**

* Press `Ctrl + Shift + P` to open the  **Command Palette** .

  ![Alt text](images/guide/reopen.png)
* Search for **"Dev Conatiners: Reopen in Container"** and select it to start building the container.
> This will take a long time, it will pull the image from the github container registry and to open it.

* After this, make sure to choose the terminal profile in vscode to be `bash`. You can do this in the command pallete in github, search for terminal profile and select `bash`.

### **6. Run Tasks Inside the Container**

* Once inside the container, press ` Ctrl + Shift + P`  again.

  ![Alt text](images/guide/runtasks.png)t
* Search for  **"Run Tasks"** , then select and execute the  **following tasks** :

#### **Available Tasks**

* Below are the tasks avaliable

  ![Alt text](images/guide/tasks.png)

1. MicroXRCEAgent : Connection to ROS2 and PX4
2. PX4 SITL  : To start up PX4 with gazebo gui
3. QGroundControl : To launch QGroundControl
4. Sensor Combined Listener : To echo gyro readings and acceleration in all three axis
5. Offboard Control : To arm the vehicle and publish waypoint for quadcopter to track

### Manual Installation (If you choose to run on terminal)

First pull the image: Open a terminal and run the following command

```bash
docker pull ghcr.io/srindot/rosgzpx4
```

This is going to take some time.

 Create a container instance from the pulled image refer to this [repo](https://github.com/Srindot/docker-dev-for-ros.git) for instructions

To run the simulator follow below steps.

1. To launch **MicroXRCEAgent**: Navigate to a new terminal and run the below command to launch it

```bash
   MicroXRCEAgent udp4 -p 8888
```

3. To make **PX4_sitl**, open another terminal and navigate to PX4-Autopilot dir

```bash
   cd ~/PX4-Autopilot
```

  Make the PX4_sitl and launch the vehicle gz_x500

```bash
   make px4_sitl gz_x500
```

3. To launch QGroundControl, Open another terminal and run the following command

```bash
   QGroudControl
```

4. open a new terminal and source ros2 and local_setup.bash in sensor combined workspace

```bash
   cd ~/ws_sensor_combined && source /opt/ros/humble/setup.bash && source install/local_setup.bash
```

  and run the below command to run the sensor combined listener

```bash
    ros2 launch px4_ros_com sensor_combined_listener.launch.py
```

5. To arm the vehicle and switch it to offoard mode, open a new temrinal
   source ros2 and local setup by runnig below the command

```bash
   cd ~/ws_offboard_control/src && cd .. && source /opt/ros/humble/setup.bash && source install/local_setup.bash
```

  Now run the offboard_control.py file present in the dir

```bash
   cd && python3 ~/workspace/offboard_control.py
```

### To set Quadcopter Waypoints

To set the waypoint for the quadcopter to navigate, go to [here](offboard_control.py) and set the waypoints in the list.

## **Need Help? Raise an Issue!**

If you encounter any problems or have questions, feel free to **raise an issue.**
