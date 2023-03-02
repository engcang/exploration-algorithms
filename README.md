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
+ OctoMap: NBVP, GBP, MBP, AEP, DSVP
+ Voxblox: MBP, GBP, OIPP
### 1. Install simulator (required)
<details><summary>Unfold to see</summary>
  
#### Note: When having PX4-SITL and RotorS Simulator at the same time
+ They both use `libmav_msgs.so` file with the same name but different source codes.
+ If you have both simulators, do not forget to change the name of either one temporally.
  + PX4-SITL: `PX4-Autopilot/build/px4_sitl_default/build_gazebo/libmav_msgs.so`
  + RotorS: `<workspace>/build/rotors_gazebo_plugins/libmav_msgs.so`

#### 1-1. Install PX4-SITL - for AEP
+ Follow [here](https://github.com/engcang/mavros-gazebo-application/blob/master/README.md#installation)

#### 1-2. Install RotorS Simulator - for NBVP, GBP, MBP, OIPP
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

    catkin build -DCMAKE_BUILD_TYPE=Release
  ```

#### 1-3. Install Autonomous Exploration Development Environment - for DSVP, TARE
+ Follow [here](https://www.cmu-exploration.com/)

</details>
  
### 2. Install algorithms
#### Note: When having NBVP, GBP, MBP at the same time
+ They use different versions of `volumetric_mapping`, `rotors_simulator`, `mav_comm`, `eigen_catkin`, `eigen_checks`, etc...

#### 2-1. NBVP
<details><summary>Unfold to see</summary>

+ Install dependencies and get the code
  ```shell
  cd ~/catkin_ws/src
  git clone https://github.com/ethz-asl/nbvplanner.git
  cd nbvplanner
  git submodule update --init --recursive
  
  rm -r rotors # install it as above Section 1-2.
  cd mav_comm && git pull origin master
  ```
+ Change CMakeLists.txt and build the code
  ```shell
  cd ~/catkin_ws/src/nbvplanner/nbvplanner
  wget -O CMakeLists.txt https://raw.githubusercontent.com/engcang/exploration-algorithms/main/nbvp/nbvplanner/CMakeLists.txt
  cd ~/catkin_ws/src/nbvplanner/interface_nbvp_rotors
  wget -O CMakeLists.txt https://raw.githubusercontent.com/engcang/exploration-algorithms/main/nbvp/interface_nbvp_rotors/CMakeLists.txt

  cd ~/catkin_ws
  catkin build -DCMAKE_BUILD_TYPE=Release
  ```

</details>

#### 2-2. GBP2
<details><summary>Unfold to see</summary>

+ Install dependencies and get the code
  ```shell
  sudo apt install python-catkin-tools \
  libgoogle-glog-dev \
  ros-melodic-joy \
  ros-melodic-twist-mux \
  ros-melodic-interactive-marker-twist-server \
  ros-melodic-octomap-ros

  cd catkin_ws
  git clone https://github.com/ntnu-arl/gbplanner_ros.git -b gbplanner2

  wstool init
  wstool merge ./gbplanner_ros/packages_ssh.rosinstall
  wstool update

  rm -r src/sim/rotors_simulator #install it as above Section 1-2.
  ```
+ Build  
  ```shell
  cd ~/catkin_ws
  catkin build -DCMAKE_BUILD_TYPE=Release
  ```

</details>

#### 2-3. GBP1
<details><summary>Unfold to see</summary>

+ Clone and build the code
  ```shell
    cd ~/catkin_ws
    git clone https://github.com/ntnu-arl/gbplanner_ws.git
    cd gbplanner_ws
    git checkout origin/melodic
    wstool init
    wstool merge packages_https.rosinstall
    wstool update

    mv gbplanner_ws/src/* ~/catkin_ws/src/
    cd ~/catkin_ws
    rm -r src/sim/rotors_simulator # install it as above Section 1-2.  
  ```
+ Change the path of `Eigen` in
  + `gbplanner_ros/gbplanner/include/gbplanner/params.h`
  + `gbplanner_ros/gbplanner/include/gbplanner/gbplanner_rviz.h`
  + `gbplanner_ros/gbplanner/include/gbplanner/geofence_manager.h`
  + `gbplanner_ros/gbplanner/include/gbplanner/graph_manager.h`
  + `gbplanner_ros/gbplanner/include/gbplanner/map_manager.h`
  + `gbplanner_ros/gbplanner/include/gbplanner/rrg.h`
    ```c++
      //#include <eigen3/Eigen/Dense>
      #include <Eigen/Dense>
    ```
+ Build
  ```shell
    cd ~/catkin_ws
    catkin build -DCMAKE_BUILD_TYPE=Release

    !Optionally, for use with OctoMap
    catkin build -DCMAKE_BUILD_TYPE=Release -DUSE_OCTOMAP=1
  ```

</details>

#### 2-4. MBP
<details><summary>Unfold to see</summary>

+ Get the code and build
  ```shell
    git clone https://github.com/ntnu-arl/mbplanner_ws.git
    cd mbplanner_ws
    git checkout melodic-devel
    wstool init
    wstool merge packages_https.rosinstall
    wstool update

    mv mbplanner_ws/src/* ~/catkin_ws/src/
    cd ~/catkin_ws
    rm -r src/sim/rotors_simulator # install it as above Section 1-2.
  ```
+ Fix the code error
  + `exploration/mbplanner/mbplanner_ros/planner_common/src/params.cpp`
  + Line 847 (in `MBParams::loadParams(std::string ns)`)
    ```c++
      // Add
      return true;
    ```
+ Change the path of `Eigen` in `exploration/mbplanner_ros/planner_common/include/planner_common/visualizer.h`
  ```c++
    //#include <eigen3/Eigen/Dense>
    #include <Eigen/Dense>
  ```
+ Build
  ```shell
    cd ~/catkin_ws
    catkin build -DCMAKE_BUILD_TYPE=Release

    !Optionally, for use with OctoMap
    catkin build -DCMAKE_BUILD_TYPE=Release -DUSE_OCTOMAP=1
  ```
+ Trouble shooting for `planner_common`
  + When `opencv` path errors from `image_proc`,
    + Change the directory of `opencv` in `/opt/ros/melodic/share/image_proc/cmake/image_procConfig.cmake`

</details>

#### 2-5. AEP
<details><summary>Unfold to see</summary>

+ Install dependencies and build the code
  ```shell
    sudo apt install ros-melodic-octomap* 
    sudo apt-get install libspatialindex-dev
    python2 -m pip install rtree

    cd ~/catkin_ws/src/
    git clone https://github.com/catkin/catkin_simple.git
    git clone https://github.com/mseln/aeplanner.git
    cd ..
    catkin build -DCMAKE_BUILD_TYPE=Release

    ## If PCL errors in rpl_exploration,
    ## change compiler to newer than c++14
    ## in line 4 for CMakeLists.txt of rpl_exploration
  ```
  
</details>

#### 2-6. FUEL
<details><summary>Unfold to see</summary>

+ Install dependencies 
  ```shell
    sudo apt-get install libarmadillo-dev
    sudo apt-get install libdw-dev
    git clone https://github.com/stevengj/nlopt.git
    cd nlopt
    mkdir build && cd build
    cmake ..
    make
    sudo make install
  ```
+ Get the code and change `CMakeLists.txt` of `bsline_opt`
  ```shell
    cd ~/catkin_ws/src
    git clone https://github.com/HKUST-Aerial-Robotics/FUEL.git
    cd FUEL/fuel_planner/bspline_opt
    wget -O CMakeLists.txt https://raw.githubusercontent.com/engcang/exploration-algorithms/main/fuel/CMakeLists.txt
  ```
+ Change compiler into `c++14` in all `CMakeLists.txt` files
  ```makefile
    #set(CMAKE_CXX_FLAGS "-std=c++11")
    set(CMAKE_CXX_FLAGS "-std=c++14")
  ```
+ Fix the code error
  + `FUEL/fuel_planner/path_searching/src/kinodynamic_astar.cpp`
  + Line 654 (in `int KinodynamicAstar::timeToIndex(double time)`)
    ```c++
      // Add
      return idx;
    ```
+ Build the code
  ```shell
    cd ~/catkin_ws
    catkin build -DCMAKE_BUILD_TYPE=Release
  ```

</details>

#### 2-7. DSVP
<details><summary>Unfold to see</summary>

+ Install dependencies and get the code
  ```shell
    sudo apt install ros-melodic-octomap-ros libgoogle-glog-dev libgflags-dev

    cd ~/catkin_ws/src
    git clone https://github.com/HongbiaoZ/dsv_planner.git
    cd dsv_planner
    git checkout origin/melodic
  ```
+ Fix `CMakeLists.txt` in `dsv_planner/src/volumetric_mapping/octomap_world`
  ```makefile
    #target_link_libraries(octomap_manager ${PROJECT_NAME} glog)
    target_link_libraries(octomap_manager ${PROJECT_NAME} glog gflags)
  ```
+ Fix `CMakeLists.txt` in `dsv_planner/src/dsvplanner/dsvplanner`
  ```makefile
    find_package(catkin REQUIRED COMPONENTS
      roscpp
      geometry_msgs
      visualization_msgs
      message_generation
      octomap_world
      tf
      kdtree
      std_msgs
      nav_msgs
      misc_utils
      graph_planner #ADDED
      graph_utils
    )
  ```
+ Change the path of `Eigen` in
  + `dsv_planner/src/dsvplanner/dsvplanner/include/dsvplanner/drrt_base.h`
  + `dsv_planner/src/dsvplanner/dsvplanner/src/drrtp.cpp`
  + `dsv_planner/src/dsvplanner/dsvplanner/src/drrtp_node.cpp`
    ```c++
      //#include <eigen3/Eigen/Dense>
      #include <Eigen/Dense>
    ```
+ Build the code
  ```shell
    cd ~/catkin_ws
    catkin build -DCMAKE_BUILD_TYPE=Release
  ```

</details>

#### 2-8. TARE
<details><summary>Unfold to see</summary>

+ Get and build the code
  ```shell
    cd ~/catkin_ws/src
    git clone https://github.com/caochao39/tare_planner.git
    cd ..

    sudo apt remove libflags* ### as it is using OR-Tools

    catkin build -DCMAKE_BUILD_TYPE=Release
  ```

</details>


#### 2-9. OIPP - without Unreal, Gazebo instead (thanks to [Dongkyu Lee](https://github.com/dklee98))
<details><summary>Unfold to see</summary>

+ Get dependencies
  ```shell
  sudo apt-get install python-wstool python-catkin-tools
  sudo apt-get install ros-melodic-cmake-modules ros-melodic-control-toolbox ros-melodic-joy ros-melodic-octomap-ros ros-melodic-mavlink ros-melodic-geographic-msgs autoconf libyaml-cpp-dev protobuf-compiler libgoogle-glog-dev liblapacke-dev libgeographic-dev

  cd ~/catkin_ws/src
  git clone https://github.com/ethz-asl/mav_active_3d_planning
  wstool init
  wstool merge mav_active_3d_planning/mav_active_3d_planning_https.rosinstall
  wstool update

  rm -r rotors_simulator # install it as above Section 1-2.
  rm -r yaml_cpp_catkin # prevent confliction

  cd ..
  catkin build -DCMAKE_BUILD_TYPE=Release
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

#### 2. GBP2 - it does not work for me yet
<details><summary>Unfold to see</summary>

+ Run the demo
  ```shell
    roslaunch gbplanner rmf_sim.launch
    or
    roslaunch gbplanner smb_sim.launch
  ```

</details>

#### 3. GBP1
<details><summary>Unfold to see</summary>

+ Check `map_config_file`, if it is `octomap` or `voxblox`
  ```xml
    <arg name="map_config_file" default="$(arg octomap_config_file)"/>
    <arg name="map_config_file" default="$(arg voxblox_config_file)"/>
  ```
+ Run the demo
  ```shell
    roslaunch gbplanner gbplanner_sim.launch
    rosservice call /planner_control_interface/std_srvs/automatic_planning "{}" 
  ```

</details>

#### 4. MBP
<details><summary>Unfold to see</summary>

+ Check `map_config_file`, if it is `octomap` or `voxblox`
  ```xml
    <arg name="map_config_file" default="$(arg octomap_config_file)"/>
    <arg name="map_config_file" default="$(arg voxblox_config_file)"/>
  ```
+ Run the demo
  ```shell
    roslaunch mbplanner mbplanner_m100_sim.launch
    rosservice call /planner_control_interface/std_srvs/automatic_planning "{}" 
  ```

</details>

#### 5. AEP (no official demo provided)
<details><summary>Unfold to see</summary>

+ Get config files and Gazebo models and build
  ```shell
    git clone https://github.com/engcang/exploration-algorithms --recursive
    mv exploration-algorithms/aep/ouster_gazebo_plugins ~/catkin_ws/src/
    mv exploration-algorithms/aep/gazebo_env ~/catkin_ws/src/
    mv exploration-algorithms/aep/rpl_exploraiton ~/catkin_ws/src/aeplanner/

    cd ~/catkin_ws
    catkin build -DCMAKE_BUILD_TYPE=Release
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


#### 6. FUEL
<details><summary>Unfold to see</summary>

+ Run the demo
  ```shell
    roslaunch exploration_manager rviz.launch
    roslaunch exploration_manager exploration.launch
    !Start with 2D Nav Goal in Rviz
  ```

</details>

#### 7. DSVP
<details><summary>Unfold to see</summary>

+ Run the demo
  ```shell
    roslaunch vehicle_simulator system_garage.launch
    roslaunch dsvp_launch explore_garage.launch
  ```

</details>


#### 8. TARE
<details><summary>Unfold to see</summary>

+ Run the demo
  ```shell
    roslaunch vehicle_simulator system_garage.launch
    roslaunch tare_planner explore_garage.launch
  ```
+ Trouble shooting
  + When `symbol lookup error: tare_planner/or-tools/lib/libortools.so: undefined symbol: _ZN6gflags14FlagRegistererC1IiEEPKcS3_S3_PT_S5_`
    + Check if `libortools.so` is referring the right libraries
    + If reffering wrong, delete or rename the wrong libraries temporarily
      ```shell
        ldd tare_planner/or-tools/lib/ldd libortool.so

        ! wrong output
        libgflags.so.2.2 => sota_ws/devel/lib/libgflags.so.2.2 (0x00007f036830d000)
        libglog.so.0 => sota_ws/devel/lib/libglog.so.0 (0x00007f03680dc000)

        ! after delete/rename wrong files
        sudo ldconfig
        ldd tare_planner/or-tools/lib/ldd libortool.so        
        
        ! correct output
        libgflags.so.2.2 => tare_planner/src/tare_planner/or-tools/lib/./libgflags.so.2.2 (0x00007fe6749e0000)
        libglog.so.0 => tare_planner/src/tare_planner/or-tools/lib/./libglog.so.0 (0x00007fe6747a3000)
      ```

</details>

#### 9. OIPP without Unreal, Gazebo instead (thanks to [Dongkyu Lee](https://github.com/dklee98))
<details><summary>Unfold to see</summary>



</details>



<br>