#!/bin/bash

# 参数说明
# $1：是否执行 git pull（yes/no）
# $2：colcon build 的目标路径（默认为当前路径）
# $3：local_setup.bash 的路径（默认为 ~/AURO_final/install/local_setup.bash）

# 设置默认值
GIT_PULL="no"
BUILD_PATH="."
SETUP_PATH="$HOME/AURO_final/install/local_setup.bash"
LAUNCH="no"

# 使用 getopts 解析参数
while getopts ":g:b:s:h:l:" opt; do
  case $opt in
    g)
      GIT_PULL=$OPTARG
      ;;
    b)
      BUILD_PATH=$OPTARG
      ;;
    s)
      SETUP_PATH=$OPTARG
      ;;

    h)
      echo "Usage: $0 [-g yes/no] [-b build_path] [-s setup_path]"
      echo "  -g    Run 'git pull' (yes or no, default: yes)"
      echo "  -b    Path for 'colcon build' (default: current directory)"
      echo "  -s    Path to 'local_setup.bash' (default: ~/AURO_final/install/local_setup.bash)"
      echo "  -l    Run 'ros2 launch solution solution_nav2_launch.py'"
      exit 0
      ;;
    l)
        LAUNCH=$OPTARG
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      exit 1
      ;;
  esac
done
# 判断是否执行 git pull
# 执行 git pull
if [[ $GIT_PULL == "yes" ]]; then
    echo "Running git pull..."
    git pull
else
    echo "Skipping git pull."
fi

# 构建项目
echo "Building project with colcon in path: $BUILD_PATH"
colcon build --base-path "$BUILD_PATH"

# 设置环境变量
if [[ -f $SETUP_PATH ]]; then
    echo "Sourcing local setup from: $SETUP_PATH"
    source "$SETUP_PATH"
else
    echo "Error: Setup file not found at $SETUP_PATH"
    exit 1
fi
# 该项被废弃
if [[ $LAUNCH == "yes" ]]; then
    echo "Running ros2 launch.."
    ros2 launch solution solution_nav2_launch.py
else
    echo "Skipping ros2 launch."
fi
echo "All steps completed successfully!"