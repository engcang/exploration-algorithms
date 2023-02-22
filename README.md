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
+ Option1: Follow install instructions below and run demos
+ Option2: Get Docker file that already has SOTA packages installed

## Installation
### Option1 - Follow install instructions
#### 1. Install simulator
<details><summary>Unfold to see</summary>
  
##### Note: Having PX4-SITL and RotorS Simulator at the same time
+ They both use `libmav_msgs.so` file with the same name but different source codes.
+ If you have both simulators, do not forget to change the name of either one temporally.

##### 1-1. Install PX4-SITL - for AEP
+ Follow [here](https://github.com/engcang/mavros-gazebo-application/blob/master/README.md#installation)

##### 1-2. Install RotorS Simulator - for NBVP, GBP, MBP

</details>
  
#### 2. Install algorithms
##### 2-1. AEP
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

### Option2 - Get Docker file
+ AEP
+ NBVP, GBP, MBP
+ DSVP, TARE


<br>

## Run Demos
+ AEP
  + If installed manually,
  + If installed with docker,
    ```shell
      roslaunch rpl_exploration px4_sitl_gazebo.launch
      # choose sensor
      roslaunch rpl_exploration rpl_exploration.launch sensor:=lidar
      roslaunch rpl_exploration rpl_exploration.launch sensor:=rgbd
      # arming & offboarding
      rosservice call /mavros/cmd/arming "value: true"
      rosservice call /mavros/set_mode "base_mode: 0 custom_mode: 'OFFBOARD'"
    ```

<br>

