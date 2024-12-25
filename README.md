<!-- PROJECT SHIELDS -->

[![Contributors][contributors-shield]][contributors-url]
![GitHub Repo stars](https://img.shields.io/github/stars/chenglun11/AURO_final)
![GitHub Issues or Pull Requests](https://img.shields.io/github/issues/chenglun11/AURO_final)
![GitHub Release](https://img.shields.io/github/v/release/chenglun11/AURO_final)
![GitHub License](https://img.shields.io/github/license/chenglun11/AURO_final)

<!-- PROJECT LOGO -->

<br />

<p align="center">
  <a href="https://github.com/chenglun11/AURO_final/">
    <img src="https://www.york.ac.uk/static/stable/img/logo.svg" alt="Logo" width="80" height="80">
  </a>

<h3 align="center">AURO - Nav2Robot </h3>
  <p align="center">Nav2Robot is a solution to the finial Assessment of AURO at University of York 2024-2025-M</p>
  <p align="center">
    <br />
    <a href="https://github.com/chenglun11/AURO_final/blob/main/README.md"><strong>Explore this document »</strong></a>
    <br />
    <br />
    <a href="https://github.com/chenglun11/AURO_final">Demo</a>
    ·
    <a href="https://github.com/chenglun11/AURO_final/issues">Report Bug</a>
    ·
    <a href="https://github.com/chenglun11/AURO_final/issues">Commit a Feature</a>
  </p>

</p>

## 目录

- [目录](#目录)
  - [Requirements](#requirements)
  - [Installation](#installation)
  - [文件目录说明](#文件目录说明)
  - [How it works](#how-it-works)
  - [部署](#部署)
  - [Framework](#framework)
  - [Contributing](#contributing)
    - [如何参与开源项目](#如何参与开源项目)
  - [Version Control](#version-control)
  - [Author](#author)
  - [版权说明](#版权说明)
  - [鸣谢](#鸣谢)


### Requirements 

1. [ROS2 Humble](https://docs.ros.org/en/humble/index.html)
2. [ROS2 Navigation2](https://docs.nav2.org/#)
3. [Gazebo Classic Version 11](https://gazebosim.org/home)
4. [RViz](http://wiki.ros.org/rviz)
5. Ubuntu 22.04 `forced`

### Installation

1. Clone the repo `Will available soom`

```bash
git clone https://github.com/chenglun11/AURO_final.git
```

2. Build

```bash
colcon build
```

3. Source Code

```bash
source /path/install/local_setup.bash
```

### 文件目录说明

```
filetree 
├── /solution/
│  ├── /config/
│  │  ├── custom_rviz_windows.yaml
│  │  └── initial_poses.yaml
│  ├── /solution/
│  │  ├── robot_controller.py
│  │  ├── data_logger.py
│  │  └── __init__.py
│  ├── /launch/
│  │  ├── solution_launch.py
│  │  └── solution_nav2_launch.py
│  ├── /params/
│  │  └── custom_nav2_params_namespaced.yaml
│  ├── /test/
│  │  ├── test_copyright.py
│  │  ├── test_flake8.py
│  │  └── test_pep257.py
│  ├── package.xml
│  ├── setup.cfg
│  └── setup.py
├── LICENSE
└── README.md 

```

### How it works

请阅读[ARCHITECTURE.md](https://github.com/shaojintian/Best_README_template/blob/master/ARCHITECTURE.md) 查阅为该项目的架构。

### 部署

暂无

### Framework

- [xxxxxxx](https://getbootstrap.com)
- [xxxxxxx](https://jquery.com)
- [xxxxxxx](https://laravel.com)

### Contributing

请阅读**CONTRIBUTING.md** 查阅为该项目做出贡献的开发者。

#### 如何参与开源项目

贡献使开源社区成为一个学习、激励和创造的绝佳场所。你所作的任何贡献都是**非常感谢**的。

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

### Version Control

该项目使用Git进行版本管理。您可以在repository参看当前可用版本。
The project uses Git for version control. You can see the currently available versions in the repository.

### Author
The Author detail is currectly unavailable duo to the policy of exam.

*You can also see all the developers involved in the project in the contributors list.*

### 版权说明

该项目签署了MIT 授权许可，详情请参阅 [LICENSE.txt](https://github.com/shaojintian/Best_README_template/blob/master/LICENSE.txt)

### 鸣谢

- [GitHub Emoji Cheat Sheet](https://www.webpagefx.com/tools/emoji-cheat-sheet)
- [Img Shields](https://shields.io)
- [Choose an Open Source License](https://choosealicense.com)
- [GitHub Pages](https://pages.github.com)
- [Animate.css](https://daneden.github.io/animate.css)
- [xxxxxxxxxxxxxx](https://connoratherton.com/loaders)

<!-- links -->

[your-project-path]: shaojintian/Best_README_template
[contributors-shield]: https://img.shields.io/github/contributors/shaojintian/Best_README_template.svg?style=flat-square
[contributors-url]: https://github.com/shaojintian/Best_README_template/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/shaojintian/Best_README_template.svg?style=flat-square
[forks-url]: https://github.com/shaojintian/Best_README_template/network/members
[stars-shield]: https://img.shields.io/github/stars/shaojintian/Best_README_template.svg?style=flat-square
[stars-url]: https://github.com/shaojintian/Best_README_template/stargazers
[issues-shield]: https://img.shields.io/github/issues/shaojintian/Best_README_template.svg?style=flat-square
[issues-url]: https://img.shields.io/github/issues/shaojintian/Best_README_template.svg
[license-shield]: https://img.shields.io/github/license/shaojintian/Best_README_template.svg?style=flat-square
[license-url]: https://github.com/shaojintian/Best_README_template/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=flat-square&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/shaojintian
