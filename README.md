# depth_anything_ros

ROS1 wrapper for [Depth Anything V2](https://github.com/DepthAnything/Depth-Anything-V2.git).

## Creating Pointcloud without depth
![Alt text](assets/demo.gif)

## Setup

### Prerequisite
This package is build upon
- ROS1 (Noetic)
- cuda (11.8+)
- TensorRT(8+)
- (Optional) docker and nvidia-container-toolkit (for environment safety)

### Build package

#### on your workspace
```bash
mkdir -p ~/ros/catkin_ws/src && cd ~/ros/catkin_ws/src
git clone https://github.com/ojh6404/depth_anything_ros.git
cd ~/ros/catkin_ws/src/depth_anything_ros && git submodule --init --recursive
cd ~/ros/catkin_ws && TENSORRT_DIR='/path/to/your/tensorrt_dir' catkin build
cd ~/ros/catkin_ws/src/depth_anything_ros && pip install -r requierments.txt
cd ~/ros/catkin_ws/src/depth_anything_ros && ./scripts/onnx2tensorrt.sh trained_data
```

#### using docker (Recommended)
```bash
git clone https://github.com/ojh6404/depth_anything_ros.git
cd depth_anything_ros && docker build -t depth_anything_ros .
```

## How to use
```bash
roslaunch depth_anything_ros.launch depth_estimation.launch input_image:=/your/image/topic camera_info:=/your/camera_info/topic
```
Please refer depth_estimation.launch to see args.


In case you are using docker,
```bash
docker run --rm --net=host -it --gpus 1  depth_anything_ros:latest /bin/bash
```
then inside docker,
```bash
roscd depth_anything_ros
./scripts/onnx2tensorrt trained_data # convert onnx to tensorrt
roslaunch depth_anything_ros.launch depth_estimation.launch input_image:=/your/image/topic camera_info:=/your/camera_info/topic
```

For just test with provided rosbag, run
```bash
roslaunch depth_anything_ros test.launch
```

### Acknowledgement
- [Depth Anything V2](https://github.com/DepthAnything/Depth-Anything-V2.git).
- [depth-anything-tensorrt](https://github.com/spacewalk01/depth-anything-tensorrt.git).
