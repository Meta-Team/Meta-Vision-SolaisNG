Meta-Vision-SolaisNG
===
=> Please make sure to visit [Project Wiki](https://github.com/Meta-Team/Meta-Vision-SolaisNG/wiki) :smiley:

Meta-Vision-SolaisNG is a complete rewrite of the original [Meta-Vision-Solais](https://github.com/Meta-Team/Meta-Vision-Solais) project. It is based on ROS2 (Robot Operating System) as it is designed to be modular.

> _Solais_ means "light" in Irish. [Claíomh Solais](https://en.wikipedia.org/wiki/Cla%C3%ADomh_Solais), 
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
- [Clone](#clone)
- [Setup on Jetson Orin Nano (Ubuntu 22.04 JetPack 6)](#setup-on-jetson-orin-nano-ubuntu-2204-jetpack-6)
- [Setup on Jetson Orin Nano (Ubuntu 20.04 JetPack 5)](#setup-on-jetson-orin-nano-ubuntu-2004-jetpack-5)
- [Setup on WSL (Ubuntu 22.04)](#setup-on-wsl-ubuntu-2204)
- [Colcon Build System](#colcon-build-system)
- [Design Idea: Complete Modularity based on ROS2](#design-idea-complete-modularity-based-on-ros2)

<!-- TOC end -->

# Dependencies
* ROS 2 Humble

# Clone

This project contains multiple submodules. They are all necessary for compilation.

To clone this project and all the submodules, run the following:

```shell
git clone --recurse-submodules https://github.com/Meta-Team/Meta-Vision-SolaisNG
```

# Setup on Jetson Orin Nano (Ubuntu 22.04 JetPack 6)

# Setup on Jetson Orin Nano (Ubuntu 20.04 JetPack 5)

Please refer to this [page](https://github.com/Meta-Team/Meta-Vision-SolaisNG/wiki/Setup-on-Jetson-Orin-Nano-(Ubuntu-20.04)) for further information.

# Setup on WSL (Ubuntu 22.04)

Please refer to this [page](https://github.com/Meta-Team/Meta-Vision-SolaisNG/wiki/Setup-on-WSL-(Ubuntu-22.04)) for further information.

# Colcon Build System
This project uses `colcon` build system. It is the build system for ROS2.

# Design Idea: Complete Modularity based on ROS2
