#! /bin/bash
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#% created: March 2020
#% version: 1.0.0
#% authors: 
#% * Roland Jung (roland.jung@aau.at)
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

# specify the executable:

profile_msckf_demo_ros() {
  EXEC=$1
  TYPE=$2
  NUM_IMAGES=$3
  DATASET_PATH=$4
  RESOLUTION="unknown"
  
  if [ "$TYPE" = "small"]; then
    RESOLUTION="320_240"
  elif [ "$TYPE" = "medium"]; then
    RESOLUTION="640_480"
  elif [ "$TYPE" = "small"]; then
    RESOLUTION="1280_960"
  else
    echo "ERROR: unknown type! either: small, medium, large supported!"
  fi
  
  CALLGRIND_OUTPUT="callgrind.main_demo_ros."${TYPE}".N_"${NUM_IMAGES}".%p"
  
 valgrind --tool=callgrind --callgrind-out-file=${CALLGRIND_OUTPUT} --main-stacksize=10000000000 $EXEC --topic_imu /mus/imu --topic_camera /mus/image --topic_gt_pose /mus/ground_truth_pose_imu --rate_reduction_imu 5 --rate_reduction_cam 10 -b /DATASET_PATH/${RESOLUTION}/record.bag -c /home/jungr/workspace/NAV/development/catkin_workspaces/msckf_mono_cws/src/msckf_mono/unitysim/unitysim_config_${TYPE}.yaml --process_N_cam_imgs $NUM_IMAGES 
}


if [[ -z $1 ]]; then
  echo "first argument for USED_EXEC empty..."
  USED_EXEC=../../../build-msckf_mono-Reldbg/devel/lib/msckf_mono/msckf_demo_ros
else
  USED_EXEC=$1
fi


if [[ -z $2 ]]; then
  echo "second argument for DATASET_PATH empty..."
  DATASET_PATH=/home/jungr/workspace/datasets/MCS_Run_15_Resolutions/
else
  DATASET_PATH=$2
fi


echo "profile SMALL..."
profile_msckf_demo_ros $USED_EXEC small 500 $DATASET_PATH
echo "profile MEDIUM..."
profile_msckf_demo_ros $USED_EXEC medium 500 $DATASET_PATH
echo "profile LARGE..."
profile_msckf_demo_ros $USED_EXEC large 500 $DATASET_PATH
