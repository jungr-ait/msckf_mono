/******************************************************************************
* FILENAME:     main_demo_ros.cpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     24.03.2020
*
*  Copyright (C) 2020
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <eigen3/Eigen/Core>
#include <opencv/cxeigen.hpp>
#include <CLI11/CLI11.hpp>
#include <boost/filesystem.hpp>
#include <msckf_mono/ros_interface.h>
#include <utilities/IO.hpp>
#include <utilities/RTVerification.hpp>
#include <vision_core/config_helper.hpp>
//using namespace boost::filesystem;

int main(int argc, char **argv)
{
  std::string app_name = "MSCKF_demo_ros";
  ros::init(argc, argv, app_name);
  ros::start();

  CLI::App app{app_name};

  std::string bag_filename = "./euroc/MH_03_medium.bag";
  app.add_option("-b,--bagfile", bag_filename, "bag file name to be processed");
  std::string topic_imu = "/imu0";
  app.add_option("--topic_imu", topic_imu, "topic name");
  std::string topic_camera = "/cam0/image_raw";
  app.add_option("--topic_camera", topic_camera, "topic name");

  std::string topic_gt_position = "/gt_pos";
  app.add_option("--topic_gt_position", topic_gt_position, "topic name");
  std::string topic_gt_pose = "/gt_pose";
  app.add_option("--topic_gt_pose", topic_gt_pose, "topic name");

  int start = 0;
  app.add_option("--start", start, "start time");
  int stop = 0;
  app.add_option("--stop", stop, "stop time");

  int num_loops = 1;
  app.add_option("--num_loops", num_loops, "number of loops processed");
  int rate_Hz = 1;
  app.add_option("--rate_Hz", rate_Hz, "processing rate");

  std::string config_filename = "./euroc/euroc_config.yaml";
  app.add_option("-c,--config_filename", config_filename, "configuration file");

  CLI11_PARSE(app, argc, argv);


  if(boost::filesystem::exists(boost::filesystem::path(bag_filename)))
  {
    std::cout << "bagfile exists...; " << std::endl;
    std::cout << "Opening bag-file: " << bag_filename;

    rosbag::Bag bag;
    bag.open(bag_filename, rosbag::bagmode::Read);

    // setup topics
    std::vector<std::string> topics{topic_imu, topic_camera, topic_gt_position, topic_gt_pose};

    rosbag::TopicQuery query(topics);
    rosbag::View       view(bag, query);
    float processing_stop_at  = stop * view.size();
    float processing_start_at = start * view.size();



    if(num_loops == 0)
    {
      num_loops = std::numeric_limits<int>::max();
    }

    std::cout << "run bag-file " << num_loops << " times";

    ros::Rate rate(rate_Hz);

    bool useSingleStep = true;
    int  num_run       = 0;


    msckf_mono::RosInterface ros_node;
    RTV_EXPECT_TRUE(ros_node.init_YAML(config_filename), "could not load parameters!");
    std::cout << "node configured...." << std::endl;
    do
    {
      size_t         seq_len       = view.size();
      unsigned const slice_len     = seq_len / 100;
      unsigned       percent       = 0;
      int            processed_cnt = 0;
      bool           shutdown      = false;


      for(auto const& m : view)
      {
        sensor_msgs::ImageConstPtr               image  = m.instantiate<sensor_msgs::Image>();
        sensor_msgs::ImuConstPtr                 imu    = m.instantiate<sensor_msgs::Imu>();
        geometry_msgs::PointStamped::ConstPtr   gt_position = m.instantiate<geometry_msgs::PointStamped>();
        geometry_msgs::PoseStamped::ConstPtr       gt_pose = m.instantiate<geometry_msgs::PoseStamped>();

        if(processed_cnt >= (int) ((percent + 1) * slice_len))
        {
          percent++;
          std::cout << "bag processed [" << percent << "%]" << std::endl;
        }

        //if((processing_stop_at == 0 || processed_cnt < processing_stop_at) && processed_cnt > processing_start_at)
        {


          if(!ros::ok() || shutdown)
          {
            std::cout << "abort loop" << std::endl;
            break;
          }
          if(image != nullptr &&  (m.getTopic() == topic_camera) )
          {
            std::cout << "image: " << std::endl;
            ros_node.imageCallback(image);
            //node.imgMonoCallback(image);
          }

          if(imu != nullptr &&  (m.getTopic() == topic_imu))
          {
            std::cout << "imu" << std::endl;
            ros_node.imuCallback(imu);
            //node.imuCallback(imu);
          }

          if(gt_pose != nullptr)
          {
            //node.orientCallback(orient);
          }

          if(gt_position != nullptr)
          {
            //node.groundTruthCallback(groundTruth);
          }

          ros::spinOnce();
        }

        processed_cnt++;
      }

      num_run++;
    }
    while(num_run < num_loops);



  }
  return 0;
}
