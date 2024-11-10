# ros2

## install m-ros agent
- navigate to ros2 workspace src folder `cd ros2_ws/src`
- `git clone https://github.com/micro-ROS/micro-ROS-Agent.git -b <distro>` <br>
    and change distro to required distribution
- `source /opt/ros/<distro>/setup.bash` <br>
adjust distro such as jazzy or fuzzy
- `sudo apt install python3-rosdep`
- `colcon build --merge-install --symlink-install`

## run m-ros agent
- `source /opt/ros/<distro>/setup.bash` <br>
adjust distro such as jazzy or fuzzy
- `cd ros2_ws/src`
- `source install/setup.bash`
- `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0` <br>
adjust serial port or use UDP protocols for network

## clone with submodule
- `git clone https://github.com/01604987/ros2.git`
- `git submodule update --init --recursive`
- modify submodule with:
    - `git pull --recurse-submodules`
    - `git submodule update --recursive --remote`
- or alternatively:
    - `cd <submodule folder>`
    - `git fetch/pull/commit/push`
    - `cd <root folder>`
    - `git commit`