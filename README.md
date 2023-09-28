# ALAN ("A"utonomous "L"anding using "A" "N"on-Robocentric Framework)
## Towards Non-Robocentric Dynamic Landing of a Quadrotor: A New Framework

### Abstract
<div align="justify">
This research addresses the problem of a quadrotor UAV landing on a ground vehicle. Yet, unlike most existing literature, we transfer most sensing and computing tasks to the ground vehicle, designing the landing system in a non-robocentric fashion. Such a framework greatly alleviates the payload burden, allowing more resource allocation for the quadrotor UAV. To validate the proposed framework, the implementation starts with relative pose estimation through detection and tracking of LED markers on an aerial vehicle. The 6 DoF orientation and position information is then returned through a PnP-based algorithm. Successively, by considering the visibility and dynamic constraints, the motion planning module computes an optimized landing trajectory, such that the aerial vehicle stays within a safety corridor and performs the landing mission. Through experiments, we demonstrate the applicability of this research work, in which a quadrotor could be guided remotely and landed on a moving ground vehicle smoothly without the support from any airborne exteroceptive sensors and computers.
</div>

### Video
### Video
<a href="https://www.youtube.com/watch?v=7wiCh46MQmc&ab_channel=AIRO-LAB%40HKPolyU
" target="_blank"><img src="https://img.youtube.com/vi/7wiCh46MQmc/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="533" height="400" border="10" /></a>
### Software Installation
#### Pre-Requisite

```
# we tested on system with ubuntu 18.04 and 20.04 
# install Ubuntu 18.04 || 20.04
# install ROS
# install OpenCV (should come along with ROS)
```

This repo utilize some third-party libraries,
- [OSQP](https://github.com/osqp/osqp.git) to solve the convex optimization problem for trajectory generation. <br/>
- [OSQP-Eigen](https://github.com/robotology/osqp-eigen.git) for using OSQP with Eigen. Much much much convinient.
- [DecompUtil](https://github.com/sikang/DecompUtil.git) to pass and visualize safety corridor.

Therefore, under your workspace folder
```
mkdir -p {name alan}_ws/src
cd {name alan}_ws && mkdir alan_third_party

# install third party



sudo apt install ros-noetic-sophus

(if neccessary)
cd /usr/include
sudo ln -sf eigen3/Eigen Eigen

cd alan_third_party

git clone --recursive https://github.com/pattylo/osqp.git && \
git clone https://github.com/pattylo/osqp-eigen.git && \
git clone https://github.com/catkin/catkin_simple.git && \
git clone https://github.com/sikang/DecompUtil.git

#for the above package, please do

mkdir build && cd build && cmake .. && make -j8 && sudo make install
#then do compilation
cd ~/{name alan}_ws
catkin_make


# If you are using scount_ros, feel free to use our scount_ros package
mkdir -p {name scout}_ws/src
cd {name scout}_ws/src
git clone https://github.com/HKPolyU-UAV/scout_ros
catkin_make
```
For scout_ros, click [here](https://github.com/agilexrobotics/scout_ros.git) to know more. They have done a fantastic job on packaging their hardware platform.

The above packages salute the contribution of the following academic paper:

```
@article{stellato2020osqp,
  title={OSQP: An operator splitting solver for quadratic programs},
  author={Stellato, Bartolomeo and Banjac, Goran and Goulart, Paul and Bemporad, Alberto and Boyd, Stephen},
  journal={Mathematical Programming Computation},
  volume={12},
  number={4},
  pages={637--672},
  year={2020},
  publisher={Springer}
}

@article{liu2017planning,
  title={Planning dynamically feasible trajectories for quadrotors using safe flight corridors in 3-d complex environments},
  author={Liu, Sikang and Watterson, Michael and Mohta, Kartik and Sun, Ke and Bhattacharya, Subhrajit and Taylor, Camillo J and Kumar, Vijay},
  journal={IEEE Robotics and Automation Letters},
  volume={2},
  number={3},
  pages={1688--1695},
  year={2017},
  publisher={IEEE}
}
```
Further details will be released once the manuscript is accepted.

<!-- Now, below shows the architecture of the software platform: -->


<!-- ### Hardware Used in Literature -->
