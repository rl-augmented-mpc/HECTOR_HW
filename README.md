# HECTOR_V1
Hardware code for HECTOR V1

# Dependencies
* Boost (1.5.4 or higher)
* Cmake (2.8.3 or higher)
* LCM (1.4.0 or higher)
* Eigen3
* realsense SDK 2.0 (v2.50.0 or lower)
* SDL2

# Dependencies install instruction

* Boost: `sudo apt-get install libboost-all-dev`
* Cmake: `sudo apt install cmake`
* LCM: Follow instructions from [https://lcm-proj.github.io/lcm/content/build-instructions.html](https://lcm-proj.github.io/lcm/content/build-instructions.html)
* Eigen3: download sources file from [https://eigen.tuxfamily.org/index.php?title=Main_Page#Download](https://eigen.tuxfamily.org/index.php?title=Main_Page#Download)
  * In the root directory of eigen3 source file, type
    ```bash
    mkdir build && cd build
    cmake ..
    sudo make install
    ```
* realsense SDK 2.0: download source files [https://github.com/IntelRealSense/librealsense/releases/tag/v2.50.0](https://github.com/IntelRealSense/librealsense/releases/tag/v2.50.0)
  * Install using instructions from [https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)
* SDL2: install by typing `sudo apt install libsdl2-2.0-0 libsdl2-dev`

  
# Build Instruction
```bash
cd {/path/to/root}
mkdir build && cd build
cmake ..
make -j8
```


# Operation instruction 
To operate the robot, you need keyboard.

* Press 1: Transition to Passive mode
* Press 2: Transition to PDStand mode
* Press 3: Transition to walking gait (Only activated in WalkingFSM)
* Press 4: Transition to standing gait (Only activated in WalkingFSM)

## Keyboard
* Press w: Increase x velocity by 0.05
* Press s: Decrease x velocity by 0.05
* Press d: Increase y velocity by 0.05
* Press a: Decrease y velocity by 0.05
* Press q: Increase yaw angular velocity by 0.05
* Press e: Decrease yaw angular velocity by 0.05

To control the resolution, edit `sensitivityLeft` and `sensitivityRight` in include/interface/KeyBoard.h


# Change default parameters
There are sets of parameters to tune for stable walking or testing with different conditions. 

* Gait parameters \
Modify gait durations in `ConvexMPC/ConvexMPCLocomotion.cpp`.
```cpp
ConvexMPCLocomotion::ConvexMPCLocomotion(double _dt, int _iterations_between_mpc) : iterationsBetweenMPC(_iterations_between_mpc),
                                                                                    horizonLength(10),
                                                                                    dt(_dt),
                                                                                    walking(horizonLength, Vec2<int>(200, 200), Vec2<int>(0, 0)),
                                                                                    standing(horizonLength, Vec2<int>(int(0.0/_dt), int(0.0/_dt)), Vec2<int>(int(0.2/_dt), int(0.2/_dt)))
```

* Swing parameters 
Modify `foot_height` in `include/common/Biped.h`. \
Default value is 0.12m

* Reference COM height 
Modify `ref_height` in `include/common/Biped.h`. \
Default value is 0.55m