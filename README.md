# HECTOR_V1_GT
Hardware code for HECTOR GT

# Dependencies
* Boost (1.5.4 or higher)
* Cmake (2.8.3 or higher)
* LCM (1.4.0 or higher)
* Eigen3
* realsense SDK 2.0 (v2.50.0 or lower)
* SDL2

# Dependencies install instruction
* Boost: '$ sudo apt-get install libboost-all-dev'
* Cmake: '$ sudo apt install cmake'
* LCM: Follow instructions from [https://lcm-proj.github.io/lcm/content/build-instructions.html](https://lcm-proj.github.io/lcm/content/build-instructions.html)
* Eigen3: download sources file from [https://eigen.tuxfamily.org/index.php?title=Main_Page#Download](https://eigen.tuxfamily.org/index.php?title=Main_Page#Download)
  * In the root directory of eigen3 source file, type
    ```
    $ mkdir build && cd build
    $ cmake ..
    $ sudo make install
    ```
* realsense SDK 2.0: download source files [https://github.com/IntelRealSense/librealsense/releases/tag/v2.50.0](https://github.com/IntelRealSense/librealsense/releases/tag/v2.50.0)
  * Install using instructions from [https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)
* SDL2: install by typing 'sudo apt install libsdl2-2.0-0 libsdl2-dev'

  
# Build Instruction
```
$ mkdir build && cd build
$ cmake ..
$ make  # use -j to speed up the process
```
