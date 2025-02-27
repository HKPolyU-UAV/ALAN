# ALAN ("A"utonomous "L"anding using "A" "N"on-Robocentric Framework)
## Experimental Non-Robocentric Dynamic Landing of Quadrotor UAVs with On-Ground Sensor Suite

### Abstract
<div align="justify">
In this work, we propose a novel dynamic landing solution utilizing an on-ground sensor suite, eliminating the need for airborne exteroceptive sensors and expensive computational units. All localization and control modules operate in a noninertial frame. The system begins with a relative state estimator that tracks the unmanned aerial vehicle’s (UAV) state via onboard light-emitting diode (LED) markers and an on-ground camera. The state is geometrically expressed on a manifold and estimated using an iterated extended Kalman filter (IEKF) algorithm. A motion planning module is then developed to guide the landing process, leveraging the differential flatness property to formulate it as a minimum jerk trajectory. Considering visibility and dynamic constraints, the problem is solved using quadratic programming (QP), with the final motion primitive represented through piecewise polynomials. A series of experiments validate the applicability of the proposed approach, achieving successful landings of an 18×18 cm quadrotor on a 43×43 cm platform, demonstrating performance comparable to conventional methods. In addition, comprehensive hardware and software details are provided for future reference within the research community.
</div>
<br/>

Cite us!
```
@article{lo2024experimental,
  title={Experimental Non-Robocentric Dynamic Landing of Quadrotor UAVs with On-Ground Sensor Suite},
  author={Lo, Li-Yu and Li, Boyang and Wen, Chih-Yung and Chang, Ching-Wei},
  journal={IEEE Transactions on Instrumentation and Measurement},
  year={2024},
  publisher={IEEE}
}

@inproceedings{lo2023landing,
  title={Landing a Quadrotor on a Ground Vehicle without Exteroceptive Airborne Sensors: A Non-Robocentric Framework and Implementation},
  author={Lo, Li-Yu and Li, Boyang and Wen, Chih-Yung and Chang, Ching-Wei},
  booktitle={2023 IEEE 26th International Conference on Intelligent Transportation Systems (ITSC)},
  pages={6080--6087},
  year={2023},
  organization={IEEE}
}
```

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


<!-- Now, below shows the architecture of the software platform: -->


<!-- ### Hardware Used in Literature -->
