# ALAN ("A"utonomous "L"anding using "A" "N"on-Robocentric Framework)
## Towards Non-Robocentric Dynamic Landing of a Quadrotor: A New Framework

### Abstract
<div align="justify">
This paper addresses the problem of a quadrotor landing on a dynamic platform. Yet, different from
most existing literatures, we approach the problem from a non-robocentric perspective, in which
most sensing and computing tasks are transferred to ground, being performed in an offboard fashion.
Such framework is believed to greatly alleviate the payload burden of a quadrotor UAV, while
potentially increase the computation capability. In the proposed framework, modular modules are
designed, in which relative pose estimation is first conducted through detection and tracking of LED
markers of an aerial vehicles. The 6 DoF information is then returned through PnP algorithm family
and minimization of reprojection error. Successively, by considering the kinematic and dynamic
constraints, the motion planning module computes an optimized landing trajectory such that the
aerial vehicle stays within safety corridor and perform the landing mission. Through a series of
experiments, we demonstrate the applicability of this research work, in which a quadrotor could be
controlled remotely and land on a moving ground vehicle constantly without equipped with sensors
and computers. The hardware and software will be released to the research community for future
reference.
</div>

### Video

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

cd alan_third_party

git clone --recursive https://github.com/pattylo/osqp.git
git clone https://github.com/pattylo/osqp-eigen.git
git clone https://github.com/catkin/catkin_simple.git
git clone https://github.com/sikang/DecompUtil.git


#for the above package, please do

mkdir build && cd build && make && sudo make install
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

Now, below shows the architecture of the software platform:


### Hardware Used in Literature