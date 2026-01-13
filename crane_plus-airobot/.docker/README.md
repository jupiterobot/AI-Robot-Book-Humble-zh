# 使用 Docker

## 使用 Docker 镜像

我们已在以下地址发布 Docker 镜像：  
https://github.com/rt-net/crane_plus/pkgs/container/crane_plus  
镜像名称为 `ghcr.io/rt-net/crane_plus:$ROS_DISTRO`。  
请在 tag 中指定您使用的 ROS 发行版（如 humble、foxy）。

若要下载 Humble 发行版的镜像，请运行以下命令：

docker pull ghcr.io/rt-net/crane_plus:humble

### 启动节点

为了在容器中运行 GUI 应用程序，我们使用  
[osrf/rocker](https://github.com/osrf/rocker) 工具。

rocker 的选项包括：
- `--net=host`：使用主机的网络环境；
- `--privileged`：用于规避 [GUI 与主机网络共用时可能出现的错误](https://github.com/osrf/rocker/issues/13)。

此外，若您计划控制 CRANE+ V2 实体机器人，请参考  
[crane_plus_control/README.md](../crane_plus_control/README.md)，  
确保已正确配置 USB 通信端口（如 /dev/ttyUSB0）的访问权限。

# 安装 rocker
sudo apt install python3-rocker

# 启动带 GUI 的节点示例
rocker --x11 --net=host --privileged \
    --volume /dev/shm:/dev/shm \
    -- ghcr.io/rt-net/crane_plus:$ROS_DISTRO \
    ros2 launch crane_plus_examples demo.launch.py

### 重新构建软件包

通过将主机上创建的 ROS 2 工作空间挂载到 Docker 容器中，  
您可以即时在容器内反映并测试主机上的代码修改。

# 创建工作空间
mkdir -p ～/crane_ws/src
# 克隆软件包
git clone https://github.com/rt-net/crane_plus.git ～/crane_ws/src/crane_plus

# 在容器内构建软件包
rocker --x11 --net=host --privileged \
    --volume ～/crane_ws:/root/overlay_ws \
    -- ghcr.io/rt-net/crane_plus:$ROS_DISTRO \
    colcon build --symlink-install

# 使用已构建的工作空间启动节点
rocker --x11 --net=host --privileged \
    --volume /dev/shm:/dev/shm \
    --volume ～/crane_ws:/root/overlay_ws \
    -- ghcr.io/rt-net/crane_plus:$ROS_DISTRO \
    ros2 launch crane_plus_examples demo.launch.py

## 构建 Docker 镜像

您可以运行脚本 `./build_source.sh $ROS_DISTRO` 在本地构建镜像。

# 构建 Humble 发行版的镜像
cd crane_plus/.docker
./build_source.sh humble
...
Successfully tagged crane_plus:humble