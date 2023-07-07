Meta-Vision-SolaisNG
===
=> Please make sure to visit [Project Wiki](https://github.com/Meta-Team/Meta-Vision-SolaisNG/wiki) :smiley:

Meta-Vision-SolaisNG is a complete rewrite of the original [Meta-Vision-Solais](https://github.com/Meta-Team/Meta-Vision-Solais) project. It is based on ROS2 (Robot Operating System) as it is designed to be modular. Currently only the core components are implemented, the `SolaisTerminal` is still under development.

> **Note**: _Solais_ means "light" in Irish. [Claíomh Solais](https://en.wikipedia.org/wiki/Cla%C3%ADomh_Solais), 
"Sword of Light" or "Shining Sword,"  is a weapon that 
appears in Irish and Scottish Gaelic folktales, reputedly as an Undefeatable Sword such that once unsheathed, 
no one could escape its blows.
> 
> Also, we Vision group mainly deal with lights in the images :)

# TOC
<!-- TOC start (generated with https://github.com/derlin/bitdowntoc) -->

- [Meta-Vision-SolaisNG](#meta-vision-solaisng)
- [TOC](#toc)
- [Dependencies](#dependencies)
- [Setup on Jetson Orin Nano (Ubuntu 20.04)](#setup-on-jetson-orin-nano-ubuntu-2004)
- [Setup on macOS](#setup-on-macos)
- [Setup on WSL (Ubuntu 22.04)](#setup-on-wsl-ubuntu-2204)
- [Colcon Build System](#colcon-build-system)
  - [Other CMake Options](#other-cmake-options)
- [Design Idea: Complete Modularity based on ROS2](#design-idea-complete-modularity-based-on-ros2)

<!-- TOC end -->

# Dependencies
* CMake >= 3.25
* OpenCV 4
* Boost >= 1.71
* ZBar (for ArmorSolverUnitTest)
* CUDA >= 12.0 or 11.4
* cuDNN >= 8.x
* TensorRT >= 8.5.2 (used for YOLOv5)
* ROS Humble

# Setup on Jetson Orin Nano (Ubuntu 20.04)

Please refer to this [page](https://github.com/Meta-Team/Meta-Vision-SolaisNG/wiki/Setup-on-Jetson-Orin-Nano-(Ubuntu-20.04)) for further information.

# Setup on macOS

**Note**: Since no one from our team uses macOS currently, we're unable to provide detailed instructions on how to install
CUDA, cuDNN and TensorRT on macOS. To our knowledge, it's **no longer possible** to install CUDA on macOS since CUDA 10.2.

# Setup on WSL (Ubuntu 22.04)

Please refer to this [page](https://github.com/Meta-Team/Meta-Vision-SolaisNG/wiki/Setup-on-WSL-(Ubuntu-22.04)) for further information.

# Colcon Build System
This project uses `colcon` build system. It is the build system for ROS2.

Therefore, we don't need to install Qt (required by SolaisTerminal) on Jetson Nano, neither
OpenCV (required by Solais Core) on the PC that only needs to run SolaisTerminal.

The two main targets are Solais Core (Solais) and Terminal (SolaisTerminal). The others are shared components, 
tools and unit tests.

## Other CMake Options

`PARAM_SET_ROOT` and `DATA_SET_ROOT` define the directory paths of parameter sets and data sets. By default, they
are both `<Project Directory>/data`, and therefore:

```
<Project Directory>/data/params: parameter files
<Project Directory>/data/params_backup: backup of parameter files on every save
<Project Directory>/data/videos: video files
<Project Directory>/data/images/<Image Set Folder>: image files
```


# Design Idea: Complete Modularity based on ROS2

