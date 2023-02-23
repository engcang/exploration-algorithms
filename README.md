# Exploration-algorithms
+ How to build, install and run open-source exploration algorithms

<br>

# List
## 1. Volumetric mapping purpose only

|     Name    |                                                               Papers                                                              |                                                            Videos                                                           |                            Code                           |          Main Group         |
|:-----------:|:---------------------------------------------------------------------------------------------------------------------------------:|:---------------------------------------------------------------------------------------------------------------------------:|:---------------------------------------------------------:|:---------------------------:|
|     NBVP    |                                     [2016 ICRA](https://ieeexplore.ieee.org/document/7487281)                                     |                                                                                                                             |       [git](https://github.com/ethz-asl/nbvplanner)       |           ETHZ-ASL          |
|     GBP     | [2019 IROS](https://ieeexplore.ieee.org/document/8968151), [2020 JFR](https://onlinelibrary.wiley.com/doi/full/10.1002/rob.21993) | [youtube](https://youtu.be/VWgEVTeABdE), [youtube2](https://youtu.be/mw0qy05Fo6Q), [youtube3](https://youtu.be/SNMsKAnCQkw) |                       Same with GBP2                      |   ETHZ-ASL+RSL / NTNU-ARL   |
|     GBP2    |                                     [2022 ICRA](https://ieeexplore.ieee.org/document/9812401)                                     |                                           [youtube](https://youtu.be/P3uT4gHEFHw)                                           |      [git](https://github.com/ntnu-arl/gbplanner_ros)     |   ETHZ-ASL+RSL / NTNU-ARL   |
|     MBP     |                                     [2020 ICRA](https://ieeexplore.ieee.org/document/9196964)                                     | [youtube](https://youtu.be/ZvUedi5mzN8), [youtube2](https://youtu.be/bKVRHaJO938), [youtube3](https://youtu.be/aT9F4gRjyJ0) |      [git](https://github.com/ntnu-arl/mbplanner_ros)     |     ETHZ-ASL / NTNU-ARL     |
|     AEP     |                                      [2019 RAL](https://ieeexplore.ieee.org/document/8633925)                                     |                                           [youtube](https://youtu.be/Mg93ojV5rC8)                                           |         [git](https://github.com/mseln/aeplanner)         |     LiU-AIICS / KTH-RPL     |
| UFOExplorer |                                      [2022 RAL](https://ieeexplore.ieee.org/document/9681328)                                     |                                           [youtube](https://youtu.be/MWrRXaaW-bg)                                           | [git](https://github.com/UnknownFreeOccupied/UFOExplorer) |           KTH-RPL           |
|     FUEL    |                                      [2021 RAL](https://ieeexplore.ieee.org/document/9324988)                                     |                                           [youtube](https://youtu.be/_dGgZUrWk-8)                                           |    [git](https://github.com/HKUST-Aerial-Robotics/FUEL)   | HKUST Aerial Robotics Group |
|     DSVP    |                                     [2021 IROS](https://ieeexplore.ieee.org/document/9636473)                                     |                                           [youtube](https://youtu.be/1yLLIZIIsDk)                                           |      [git](https://github.com/HongbiaoZ/dsv_planner)      |    CMU Robotics Institute   |
|     TARE    |      [2021 RSS](http://www.roboticsproceedings.org/rss17/p018.pdf), [2021 ICRA](https://ieeexplore.ieee.org/document/9561916)     |                                           [youtube](https://youtu.be/pIo64S-uOoI)                                           |      [git](https://github.com/caochao39/tare_planner)     |    CMU Robotics Institute   |

## 2. Volumetric mapping + Considering pose estimation

|  Name  |                                                                       Papers                                                                      |                                       Videos                                      |                           Code                          | Main Group |
|:------:|:-------------------------------------------------------------------------------------------------------------------------------------------------:|:---------------------------------------------------------------------------------:|:-------------------------------------------------------:|:----------:|
|  RHEM  | [2017 ICRA](https://ieeexplore.ieee.org/document/7989531), [2019 Autonomous Robots](https://link.springer.com/article/10.1007/s10514-019-09864-1) | [youtube](https://youtu.be/iveNtQyUut4), [youtube2](https://youtu.be/LtQ7sPbHqr0) | [git](https://github.com/ntnu-arl/rhem_planner) |  NTNU-ARL  |
| GLocal |                                         [2021 RAL](https://ieeexplore.ieee.org/abstract/document/9387110)                                         |                      [youtube](https://youtu.be/WInjZvoCDCM)                      |  [git](https://github.com/ethz-asl/glocal_exploration)  |  ETHZ-ASL  |

## 3. Volumetric mapping + inspection/coverage path planning

|    Name   |                             Papers                            |                  Videos                 |                            Code                           |          Main Group         |
|:---------:|:-------------------------------------------------------------:|:---------------------------------------:|:---------------------------------------------------------:|:---------------------------:|
|    OIPP   |    [2020 RAL](https://ieeexplore.ieee.org/document/8968434)   | [youtube](https://youtu.be/lEadqJ1_8Do) | [git](https://github.com/ethz-asl/mav_active_3d_planning) |           ETHZ-ASL          |
| PredRecon | [2023 ICRA (arXiv yet)](https://arxiv.org/pdf/2302.04488.pdf) | [youtube](https://youtu.be/ek7yY_FZYAc) | [git](https://github.com/HKUST-Aerial-Robotics/PredRecon) | HKUST Aerial Robotics Group |

<br>


# How to build and run
## Installation
### 0. Common dependencies
+ ROS: all
+ OctoMap: NBVP, GBP, MBP, AEP
+ Voxblox: MBP, GBP
### 1. Install simulator
<details><summary>Unfold to see</summary>
  
#### Note: When having PX4-SITL and RotorS Simulator at the same time
+ They both use `libmav_msgs.so` file with the same name but different source codes.
+ If you have both simulators, do not forget to change the name of either one temporally.
  + PX4-SITL: `PX4-Autopilot/build/px4_sitl_default/build_gazebo/libmav_msgs.so`
  + RotorS: `<workspace>/build/rotors_gazebo_plugins/libmav_msgs.so`

#### 1-1. Install PX4-SITL - for AEP
+ Follow [here](https://github.com/engcang/mavros-gazebo-application/blob/master/README.md#installation)

#### 1-2. Install RotorS Simulator - for NBVP, GBP, MBP
+ Because of the version issuse, I recommend to install as here
+ Get the code and build
  ```shell
    cd ~/catkin_ws/src
    git clone https://github.com/ethz-asl/rotors_simulator --recursive
    rm -r rotors_simulator/rotors_description
    rm -r rotors_simulator/rotors_gazebo

    cd ~/catkin_ws
    git clone https://github.com/engcang/exploration-algorithms
    mv exploration-algorithms/rotors_description src/rotors_simulator/
    mv exploration-algorithms/rotors_gazebo src/rotors_simulator/

    catkin build
  ```

</details>
  
### 2. Install algorithms
#### Note: When having NBVP, GBP, MBP at the same time
+ They use different versions of `volumetric_mapping`, `rotors_simulator`, `mav_comm`, `eigen_catkin`, `eigen_checks`

#### 2-1. NBVP
<details><summary>Unfold to see</summary>

+ Install dependencies and get the code
  ```shell
  cd ~/catkin_ws/src
  git clone https://github.com/ethz-asl/nbvplanner.git
  cd nbvplanner
  git submodule update --init --recursive
  
  rm -r rotors # install it as above Secetion 1-2.
  cd mav_comm && git pull origin master
  ```
+ Change CMakeLists.txt and build the code
  ```shell
  cd ~/catkin_ws/src/nbvplanner/nbvplanner
  wget -O CMakeLists.txt https://raw.githubusercontent.com/engcang/exploration-algorithms/main/nbvp/nbvplanner/CMakeLists.txt
  cd ~/catkin_ws/src/nbvplanner/interface_nbvp_rotors
  wget -O CMakeLists.txt https://raw.githubusercontent.com/engcang/exploration-algorithms/main/nbvp/interface_nbvp_rotors/CMakeLists.txt

  cd ~/catkin_ws
  catkin build
  ```

</details>

#### 2-2. MBP
<details><summary>Unfold to see</summary>

+ Get the code and build
  ```shell
  git clone https://github.com/unr-arl/mbplanner_ws.git
  cd mbplanner_ws
  git checkout melodic-devel
  wstool init
  wstool merge packages_https.rosinstall
  wstool update

  mv mbplanner_ws/src/* ~/catkin_ws/src/
  cd ~/catkin_ws
  rm -r src/sim/rotors_simulator # install it as above Secetion 1-2.

  catkin config -DCMAKE_BUILD_TYPE=Release
  catkin build
  ```
+ Trouble shooting for `planner_common`
  + When `opencv` path errors from `image_proc`,
    + Change the directory of `opencv` in `/opt/ros/melodic/share/image_proc/cmake/image_procConfig.cmake`
  + When `Eigen` error,
    + Change the path of `Eigen` in `exploration/mbplanner_ros/planner_common/include/planner_common/visualizer.h`
      + From `<eigen3/Eigen/Dense>` to `<Eigen/Dense>`
    + (Optional): Delete `eigen_catkin` and `eigen_checks`

</details>

#### 2-3. AEP
<details><summary>Unfold to see</summary>

+ Install dependencies and build the code
  ```shell
    sudo apt install ros-<distro>-octomap* 
    sudo apt-get install libspatialindex-dev
    python2 -m pip install rtree

    cd ~/catkin_ws/src/
    git clone https://github.com/catkin/catkin_simple.git
    git clone https://github.com/mseln/aeplanner.git
    cd ..
    catkin build

    ## If PCL errors in rpl_exploration,
    ## change compiler to newer than c++14
    ## in line 4 for CMakeLists.txt of rpl_exploration
  ```
  
</details>


<br>

## Run Demos
#### 1. NBVP
<details><summary>Unfold to see</summary>

+ **Important:** Put `<plugin name="ros_interface_plugin" filename="librotors_gazebo_ros_interface_plugin.so"/>` into Gazebo `.world` file
+ Run the demo
  ```shell
    roslaunch interface_nbvp_rotors flat_exploration.launch
  ```

</details>

#### 2. MBP
<details><summary>Unfold to see</summary>

+ Run the demo
  ```shell
  roslaunch mbplanner mbplanner_m100_sim.launch
  rosservice call /planner_control_interface/std_srvs/automatic_planning "{}" 
  ```

</details>

#### 3. AEP
<details><summary>Unfold to see</summary>

+ Get config files and Gazebo models and build
  ```shell
    git clone https://github.com/engcang/exploration-algorithms --recursive
    mv exploration-algorithms/aep/ouster_gazebo_plugins ~/catkin_ws/src/
    mv exploration-algorithms/aep/gazebo_env ~/catkin_ws/src/
    mv exploration-algorithms/aep/rpl_exploraiton ~/catkin_ws/src/aeplanner/

    cd ~/catkin_ws
    catkin build
  ```
+ Set Gazebo paths
  ```shell
    cd ~/catkin_ws/src/gazebo_env/gazebo_maps/reconstruction
    tar -xf recon3.tar.xz

    gedit ~/.bashrc

    !Then, edit GAZEBO_PLUGIN_PATH and GAZEBO_MODEL_PATH!

    export GAZEBO_PLUGIN_PATH=:/home/$(whoami)/PX4-Autopilot/build/px4_sitl_default/build_gazebo:$GAZEBO_PLUGIN_PATH
    export GAZEBO_MODEL_PATH=/home/$(whoami)/catkin_ws/src/gazebo_env/drone_models:/home/$(whoami)/catkin_ws/src/gazebo_env/gazebo_maps/reconstruction:/home/$(whoami)/PX4-Autopilot/Tools/sitl_gazebo/models:$GAZEBO_MODEL_PATH
  ```
+ Run the demo
  ```shell
    roslaunch rpl_exploration px4_sitl_gazebo.launch
    # choose sensor
    roslaunch rpl_exploration rpl_exploration.launch sensor:=lidar
    roslaunch rpl_exploration rpl_exploration.launch sensor:=rgbd
    # arming & offboarding
    rosservice call /mavros/cmd/arming "value: true"
    rosservice call /mavros/set_mode "base_mode: 0 custom_mode: 'OFFBOARD'"
  ```

</details>

<br>

