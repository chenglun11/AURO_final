<!-- PROJECT SHIELDS -->

<!-- ![GitHub Repo stars](https://img.shields.io/github/stars/chenglun11/AURO_final)
![GitHub Issues or Pull Requests](https://img.shields.io/github/issues/chenglun11/AURO_final)
![GitHub Release](https://img.shields.io/github/v/release/chenglun11/AURO_final)
![GitHub License](https://img.shields.io/github/license/chenglun11/AURO_final) -->

<!-- PROJECT LOGO -->

<br />

<p align="center">
  <a href="https://github.com/chenglun11/AURO_final/">
    <img src="https://www.york.ac.uk/static/stable/img/logo.svg" alt="Logo" width="80" height="80">
  </a>

<h3 align="center">Nav2Robot </h3>
  <p align="center">Nav2Robot is a solution to the finial Assessment of AURO at University of York 2024-2025-M</p>
  <p align="center">
    <br />
    <a href="https://github.com/chenglun11/AURO_final/blob/main/README.md"><strong>Explore this document »</strong></a>
    <br />
    <br />
    <a href="#demo">Demo</a>
    ·
    <a href="https://github.com/chenglun11/AURO_final/issues">简体中文[ZH-CN]</a>
    ·
    <a href="https://github.com/chenglun11/AURO_final/issues">Report Bug</a>
    ·
    <a href="https://github.com/chenglun11/AURO_final/issues">Commit a Feature</a>

</p>

</p>

TOC

- [Demo](#demo)
- [Requirements](#requirements)
- [Getting Started](#getting-started)
- [Catalog description](#catalog-description)
- [Features](#features)
- [Version Control](#version-control)
- [Author](#author)
- [License](#license)
- [Contributors](#contributors)

### Demo

1. Initialize the robot and acting
   ![initgif](./imgs/init_and_pick.gif)
2. Find another target
   ![repick](./imgs/repick.gif)

### Requirements

1. [ROS2 Humble](https://docs.ros.org/en/humble/index.html)
2. [ROS2 Navigation2](https://docs.nav2.org/#)
3. [Gazebo Classic Version 11](https://gazebosim.org/home)
4. [RViz](http://wiki.ros.org/rviz)
5. Ubuntu 22.04 `forced`
6. Tf_transforms

### Getting Started

1. install Requirements

```bash
sudo apt install ros-humble-tf-transformations
```

1. Clone the repo `Will available soom`

```bash
git clone https://github.com/chenglun11/AURO_final.git
```

3. Build

```bash
colcon build
```

4. Source Code

```bash
source /path/install/local_setup.bash
```

### Catalog description

```
filetree 
├── /solution/
│  ├── /config/
│  │  ├── custom_rviz_windows.yaml #rviz config
│  │  └── initial_poses.yaml #pose config
│  ├── /solution/
│  │  ├── robot_controller.py #main controller
│  │  ├── data_logger.py  #goal value logger
│  │  └── __init__.py
│  ├── /launch/
│  │  ├── solution_launch.py  #launch without nav2
│  │  └── solution_nav2_launch.py #launch within nav2
│  ├── /params/
│  │  └── custom_nav2_params_namespaced.yaml #Nav2 config
│  ├── /test/
│  │  ├── test_copyright.py
│  │  ├── test_flake8.py
│  │  └── test_pep257.py
│  ├── package.xml  #package detail
│  ├── setup.cfg
│  └── setup.py #setup
├── LICENSE
└── README.md 

```

### Features

### Version Control

The project uses Git for version control. You can see the currently available versions in the repository.

### Author

The Author detail is currectly unavailable duo to the policy of exam.

*You can also see all the developers involved in the project in the contributors list.*

### License

1. Package solution：Copyright (c) 2024 chenglun11 with [MIT License](https://github.com/chenglun11/AURO_final/blob/main/LICENSE)
2. Package assessment: Copyright (c) 2024 University of York and others with ELP2.0

### Contributors

**Thanks for every contributor and related package author**

- Alan Millard - initial contributor
- [Pedro Ribeiro](https://github.com/pefribeiro) - revised implementation

