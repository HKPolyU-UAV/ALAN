# Autonomous LANding
## Project under construction...

This repo DOES use third-party libraries, under your workspace folder
```
mkdir -p {name alan}_ws/src
cd {name alan}_ws && mkdir alan_third_party


#install third party

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


If you are using scount_ros, feel free to use our scount_ros package
mkdir -p {name scout}_ws/src
cd {name scout}_ws/src
git clone https://github.com/HKPolyU-UAV/scout_ros
catkin_make
```

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