# Autonomous Indoor Navigation for UGV
![Demo](demo.gif)

This project implements an autonomous indoor navigation system for an Unmanned Ground Vehicle using LiDAR and a Depth Camera, built on top of the ROS2 Navigation Stack. It enables reliable navigation in indoor environments through real-time mapping, localization, and path planning.

The system uses LiDAR for global mapping and navigation with an A*-based global planner, and a Depth Camera for local obstacle detection. For smooth and adaptive local motion, it employs the Model Predictive Path Integral controller, allowing the UGV to safely maneuver around dynamic obstacles and navigate tight indoor spaces.

For map building, the system uses SLAM Toolbox, which enables online 2D SLAM using LiDAR data. Once a map is created, AMCL  is used to localize the robot within that map during autonomous navigation.

## Installation and usage
You can install and run the project in two ways:
### 1. Using Docker
A Docker setup is available for easier and more reproducible deployment.  
**Note:** Make sure you have `docker` and `docker-compose` installed on your system.

To run the simulation using Docker Compose:
1. Clone the repository:
```bash
git clone https://github.com/dimianx/autonav
cd autonav/docker
```
2. Build the image:
```bash
docker-compose build
```
3. Run the container:
```bash
docker-compose up
```
4. Find the container ID of the running simulation container (look for the image dimianx/autonav:devel):
```bash
docker container ls
```
5. Access the container's shell (you will need two shell sessions for the container):
```bash
docker exec -it <container_id> bash
```
Replace `<container_id>` with the actual ID from the previous command.

6. Build the project inside the container:
```bash
colcon build
```

7. Source the environment:
```bash
source /entrypoint.sh
```

8. First, start the Gazebo simulation:
```bash
ros2 launch autonav_gz simulation.launch.py
```
> **Note:** Run either step 9 **or** step 10 below, depending on whether you're using navigation with an existing map or building a new map.

9. Navigation stack with a pre-built map:
    1. In a second shell session, access the container again:
    ```bash
    docker exec -it <container_id> bash
    ```
    2. Launch the navigation stack:
    ```bash
    ros2 launch autonav_navigation gz_autonav_navigation.launch.py
    ```
10. Mapping Mode (to build the map):
    1. In a second shell session, access the container again:
    ```bash
    docker exec -it <container_id> bash
    ```
    2. Launch the mapping stack:
    ```bash
    ros2 launch autonav_navigation gz_autonav_mapping.launch.py
    ```

### 2. Manual Installation
> This project requires **ROS Humble** and **Ubuntu 22.04**.  Make sure ROS is properly installed and sourced before proceeding.

1. Clone the repository:
```bash
git clone https://github.com/dimianx/autonav
cd autonav 
```
2. Create a new ROS workspace:
```
mkdir -p ./workspace/src
```
3. Move the `autonav_gz`, `autonav_navigation`, and `autonav_rviz` ROS packages into the created workspace
```bash
mv -v ./autonav_gz ./workspace/src
mv -v ./autonav_navigation ./workspace/src
mv -v ./autonav_rviz ./workspace/src
```
4. Install `rosdep` if it's not already installed:
```bash
sudo apt install python3-rosdep
```
5. Install Clearpath Robotics Dependencies:
    1. Add the Clearpath APT repository and update package lists:
    ```bash
    wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add -
    sh -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list'
    sudo apt update
    ```
    2. Add the Clearpath rosdep source and update rosdep:
    ```bash
    wget https://raw.githubusercontent.com/clearpathrobotics/public-rosdistro/master/rosdep/50-clearpath.list \
        -O /etc/ros/rosdep/sources.list.d/50-clearpath.list
    rosdep update

    ```
6. Install project's dependencies:
```bash
cd ./workspace
rosdep install --from-paths src --ignore-src -r -y
```
7. Build the project:
```bash
colcon build
```
8. Source the environment:
```bash
source ./install/setup.bash
```
9. First, start the Gazebo simulation:
```bash
ros2 launch autonav_gz simulation.launch.py
```
> **Note:** Run either step 10 **or** step 11 below, depending on whether you're using navigation with an existing map or building a new map.

10. Navigation stack with a pre-built map:
    1. In a second shell session, source the environment:
    ```bash
    source <PATH TO THE PROJECTS ROOT>/workspace/install/setup.bash
    ```
    >**Note**: Replace `<PATH TO THE PROJECTS ROOT>` with the actual path to your project's root directory.
    
    2. Launch the navigation stack:
    ```bash
    ros2 launch autonav_navigation gz_autonav_navigation.launch.py
    ```
11. Mapping Mode (to build the map):
    1. In a second shell session, source the environment:
    ```bash
    source <PATH TO THE PROJECTS ROOT>/workspace/install/setup.bash
    ```
    >**Note**: Replace `<PATH TO THE PROJECTS ROOT>` with the actual path to your project's root directory.

    2. Launch the mapping stack:
    ```bash
    ros2 launch autonav_navigation gz_autonav_mapping.launch.py