#! /bin/bash

# specify the executable:

profile_msckf_demo_ros() {
  EXEC=$1
  TYPE=$2
  NUM_IMAGES=$3
  
  CALLGRIND_OUTPUT="callgrind.main_demo_ros."${TYPE}".N_"${NUM_IMAGES}".%p"
  
 valgrind --tool=callgrind --callgrind-out-file=${CALLGRIND_OUTPUT} $EXEC --topic_imu /mus/imu --topic_camera /mus/image --topic_gt_pose /mus/ground_truth_pose_imu --rate_reduction_imu 5 --rate_reduction_cam 10 -b /home/jungr/workspace/datasets/MCS_Run_15_Resolutions/${TYPE}.bag -c /home/jungr/workspace/NAV/development/catkin_workspaces/msckf_mono_cws/src/msckf_mono/unitysim/unitysim_config_${TYPE}.yaml --process_N_cam_imgs $NUM_IMAGES 
}

USED_EXEC=../../../build-msckf_mono-Reldbg/devel/lib/msckf_mono/msckf_demo_ros

echo "profile SMALL..."
#profile_msckf_demo_ros $USED_EXEC small 500
echo "profile MEDIUM..."
profile_msckf_demo_ros $USED_EXEC medium 500
echo "profile LARGE..."
#profile_msckf_demo_ros $USED_EXEC large 500
