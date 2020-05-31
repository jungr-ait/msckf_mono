# msckf_mono - Profiling
Monocular MSCKF with ROS Support

# Requirements
- ROS Kinetic with Boost, OpenCV and Eigen
- https://github.com/uzh-rpg/fast build and install according to their instructions

# PROFILING

`main_demo_ros` allows a synchronous message by message processing of bag files and the parametrization of the
`ros_interface` via YAML file. This standalone executable supports various input arguments:
```
Usage: ./msckf_demo_ros [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  -b,--bagfile TEXT           bag file name to be processed
  --topic_imu TEXT            topic name
  --topic_camera TEXT         topic name
  --topic_gt_position TEXT    topic name
  --topic_gt_pose TEXT        topic name
  --start_sec FLOAT           start time
  --stop_sec FLOAT            stop time
  --num_loops INT             number of loops processed
  --rate_Hz INT               processing rate
  -c,--config_filename TEXT   configuration file
  --rate_reduction_imu INT    reduction factor for imu
  --rate_reduction_cam INT    reduction factor for cam
```     
to simplify the profiling.

In the following a instruction to profile the algorithm using the synthetic dataset is given.


## AAU VIO Unity Dataset -- ROS Bag

To profile the algorithm with different camera resolution and different sensor rates the `AAU VIO dataset`
can be used, which can be downloaded [here](TODO)

In the folder `msckf_mono/profiling` a script is provided to profile the `MSCKF_mono` with [valgrind](valgrind.org) tool set using the first 500 camera images at a rate of 10 Hz and the IMU at a rate of 10 Hz (refer to the reduction rates).

Run the scripts with the binary and the root directory of the dataset.
```
msckf_mono/profiling$ profile_msckf_demo_ros.sh <main_demo_ros binary> <root of dataset>
```

**HINT:** the MSCKF_mono might fail at some runs and completely diverge. In general, the algorithm does not provide reproducible results, e.g. due to RANSAC-based optimization. Therefore be patient and simply rerun the evaluation ;)  


## Publications

The profiling results are used in
-  [To Offload or Not to Offload: Edge Computing in 5G for Drone Navigation](TODO) by `Hayat et al.`


## Different resolutions

Small: `320x240`
![AAU_VIO_small](./doc/AAU_VIO_small.png)
Medium: `640x480`
![AAU_VIO_medium](./doc/AAU_VIO_medium.png)
Large: `1280x960`
![AAU_VIO_large](./doc/AAU_VIO_large.png)



# Euroc Dataset -- ROS Bag
Download MH_03_medium.bag from into the euroc folder in this repository.

```
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_03_medium/MH_03_medium.bag
mv MH_03_medium.bag <path_to_msckf_mono>/euroc/.
```

Now run the MSCKF on this sequence
```
roslaunch msckf_mono euroc.launch
```

RViz will come up by default showing the odometry and image with tracks.


# Euroc Dataset -- ASL Format
Download one (or more) of the datasets from https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets in the ASL dataset format
Place
```
%YAML:1.0
```
At the top of each YAML file. Currently OpenCV is being used to load these YAMLs and expects this header to exist.

The stand_still_end is going to be the time just before the quad takes off for the actual sequence--take care to find this before starting the MSCKF.

Now you can run the MSCKF on the individual sequence
```
roslaunch msckf_mono asl_msckf.launch data_set_path:=<directory of mav0 inside of sequence> stand_still_end:=<time to start at with dot at the end>
```

RViz will come up by default and display the image with tracks on the left and the generated path and map on the right.

![Machine Hall 03 Medium](https://github.com/daniilidis-group/msckf_mono/raw/master/euroc/MH03.png)

The two paths shown, green is ground truth and red is from the MSCKF.

# MSCKF

The actual MSCKF is fully templated based on the floating point type that you want. It should be easy to compile for applications that see real speedups from smaller floating point sizes.

We have run this on platforms ranging from the odroid to a modern laptop, so hopefully it should work on whatever device you want.

# Used in
- The Euroc dataset was evaluated in http://rpg.ifi.uzh.ch/docs/ICRA18_Delmerico.pdf
- The core MSCKF was used in http://openaccess.thecvf.com/content_cvpr_2017/papers/Zhu_Event-Based_Visual_Inertial_CVPR_2017_paper.pdf

# TODO
- ROS Nodelet
- Remove OpenCV from opening YAML files
- PennCOSYVIO Dataset support
