# Outdoor Manipulator URDFs and Xacro description package

## Installation
```
cd bunker_ws
rosdep install --from-paths src/bunker_description --ignore-src -r -y
colcon build --symlink-install --packages-select bunker_description
```

## Usage
```
source ~/bunker_ws/install/setup.bash
ros2 launch bunker_description display_urdf.launch.py

# or

ros2 launch bunker_description display_xacro.launch.py
```