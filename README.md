# HECTOR V1
This is the locomotion controller code for HECTOR V1. \
It comes with C++ implementation of Convex MPC controller and Unitree motor interface. \
We also support python binding of the controller, but the usage is limited in simulation because we do not have python bindings of Unitree motor interface.

# Dependencies install instruction
You need to install the following libraries to run this repository.
* Boost (1.5.4 or higher)
* Cmake (2.8.3 or higher)
* LCM (1.4.0 or higher)
* Eigen3
* realsense SDK 2.0 (v2.50.0 or lower)
* SDL2

## Simulation only 
* Eigen: `sudo apt install -y libeigen3-dev`

## Simulation + hardware interface
* Boost: `sudo apt-get install libboost-all-dev`
* Cmake: `sudo apt install cmake`
* LCM: Follow instructions from [https://lcm-proj.github.io/lcm/content/build-instructions.html](https://lcm-proj.github.io/lcm/content/build-instructions.html)
* Eigen3: `sudo apt install -y libeigen3-dev`
* realsense SDK 2.0: download source files [https://github.com/IntelRealSense/librealsense/releases/tag/v2.50.0](https://github.com/IntelRealSense/librealsense/releases/tag/v2.50.0)
  * Install using instructions from [https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)
* SDL2: install by typing `sudo apt install libsdl2-2.0-0 libsdl2-dev`

  
# Build instruction
```bash
# For hardware 
cd {/path/to/this/repo}
mkdir build && cd build
cmake -DHARDWARE=ON ..
make -j8


# For simulation (building python binding)
cd {/path/to/this/repo}
pip install -v -e .
```

> [!Note]
> Depending on ubuntu version, you may encouter the following error **libqpoases.so.3.2: cannot open shared object file: no such file or directory**. The quick workaround is add path to qpoases shared library in your .bashrc as:
```bash 
export LD_LIBRARY_PATH={/path/to/HECTOR_HW}/build/hector_system/third_party/qpOASES/libs:$LD_LIBRARY_PATH
```

# Operation instruction 
We use keyboard for operation. 

## Finited State Machine (FSM) transition

* Press 1: Transition to Passive mode
* Press 2: Transition to PDStand mode
* Press 3: Transition to Standing mode (Only activated in WalkingFSM)
* Press 4: Transition to Walking mode (Only activated in WalkingFSM)

## Velocity control
* Press w: Increase x velocity by 0.05
* Press s: Decrease x velocity by 0.05
* Press d: Increase y velocity by 0.05
* Press a: Decrease y velocity by 0.05
* Press q: Increase yaw angular velocity by 0.05
* Press e: Decrease yaw angular velocity by 0.05

To control the resolution, edit `sensitivityLeft` and `sensitivityRight` in include/interface/KeyBoard.h

# Citation
We thank University of Souther California for open-sourcing HECTOR's MPC controller. 
```bibtex
@article{li2023dynamic,
  title={Dynamic Loco-manipulation on HECTOR: Humanoid for Enhanced ConTrol and Open-source Research},
  author={Li, Junheng and Ma, Junchao and Kolt, Omar and Shah, Manas and Nguyen, Quan},
  journal={arXiv preprint arXiv:2312.11868},
  year={2023}
}
```

```bibtex
  @inproceedings{li2021force,
  title={Force-and-moment-based model predictive control for achieving highly dynamic locomotion on bipedal robots},
  author={Li, Junheng and Nguyen, Quan},
  booktitle={2021 60th IEEE Conference on Decision and Control (CDC)},
  pages={1024--1030},
  year={2021},
  organization={IEEE}
}
```