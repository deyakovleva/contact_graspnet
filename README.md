# RISE Contact-GraspNet Test Repository
> NOTE: This section is a not part of the official document. If your looking for orignal document, go down to find the official contents or go to the [orignal repository](https://github.com/NVlabs/contact_graspnet).

## 1. Prerequisite

### 1.1 Using conda env
Before using conda download the dev_ros branch only in your ros ws/src and catkin build.

Create the conda env
```
conda env create -f contact_graspnet_conda_env.yml

conda install tensorflow-base=2.4.1

conda install pyyaml=5.4.1
```

### 1.2 Troubleshooting
* Recompile pointnet2 tf_ops:
```shell
sh compile_pointnet_tfops.sh
```

## 2. Download Models and Data
Model

Download trained models from https://drive.google.com/drive/folders/1tBHKf60K8DLM5arm-Chyf7jxkzOr5zGl and copy them into the checkpoints/ folder.
Test data

Download the test data from https://drive.google.com/drive/folders/1v0_QMTUIEOcu09Int5V6N2Nuq7UCtuAA and copy them them into the test_data/ folder.

## 3. ROS Server Interface
### 3.1 grasp_planner ([ContactGraspNetPlanner](./srv/ContactGraspNetPlanner.srv))
#### 3.1.1 Service Request Messages
* color_image ([sensor_msgs/Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html))
    * Color image. It is not used for generating grasps.
* depth_image ([sensor_msgs/Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html))
    * Detph image
* camera_info ([sensor_msgs/Camerainfo](http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html))
    * Depth camera intrinsic for deprojecting depth image to point clouds.
* segmask ([sensor_msgs/Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html))
    * Object instance segmentation for grasp filltering.

#### 3.1.2 Service Reponse Messages
* grasps ([ContactGrasp[]](./msg/ContactGrasp.msg))
    * List of generated grasps.

#### 3.1.3 Arguments
* ckpt_dir (str)
    * Contact-GraspNet checkpoint directory.
    * Default: checkpoints/scene_test_2048_bs3_hor_sigma_001
* z_min (double)
    * Z min value threshold to crop the input point cloud.
    * Default: 0.2
* z_max (double)
    * Z max value threshold to crop the input point cloud.
    * Default: 1.8
* local_regions (bool)
    * Crop 3D local regions around given segments.
    * Default: False
* filter_grasps (bool)
    * Filter grasp contacts according to segmap.
    * Default: False
* skip_border_objects (bool)
    * When extracting local_regions, ignore segments at depth map boundary.
    * Default: False
* forward_passes (int)
    * Run multiple parallel forward passes to mesh_utils more potential contact points.
    * Default: 1
* segmap_id (int)
    * Only return grasps of the given object id
    * Default: 0

## 4. Launch ROS node
### 4.1 Grasp Planner Server Node
```bash
roslaunch contact_graspnet_planner grasp_planner.launch
```
### 4.2 Example Client Node
```bash
python contact_graspnet/ros_client_exmaple.py --ckpt_dir checkpoints/scene_test_2048_bs3_hor_sigma_001 --np_path test_data/7.npy
```

### 4.3 How to use my_node.py

Create conda environment with GPU tensorflow like here https://docs.anaconda.com/anaconda/user-guide/tasks/tensorflow/

Start Grasp Planner Server Node
```
roslaunch contact_graspnet_planner grasp_planner.launch
```
Start my_node.py
```
cd contact_graspnet/
python3 my_node.py
```
Start rviz with aligned_depth_to_color
```
roslaunch realsense2_camera rs_aligned_depth.launch
```
Call service for grasps generation 
```
rosservice call /get_grasps
```
Topic /answer prints 5 grasps position, orientation, scores (ascending), contact_points, id. The amount of grasps can be changed in line 334.
## Citation

```
@article{sundermeyer2021contact,
  title={Contact-GraspNet: Efficient 6-DoF Grasp Generation in Cluttered Scenes},
  author={Sundermeyer, Martin and Mousavian, Arsalan and Triebel, Rudolph and Fox, Dieter},
  booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2021}
}
```
