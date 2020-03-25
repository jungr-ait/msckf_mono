/******************************************************************************
* FILENAME:     ros_utils.hpp
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr
* MAIL:         <your mail address>
* VERSION:      v0.0.0
* CREATION:     25.03.2020
*
*  Copyright (C) 2020
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef ROS_UTILS_HPP
#define ROS_UTILS_HPP
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>

namespace msckf_mono
{

  class PathPublisher
  {
    public:
      PathPublisher(ros::NodeHandle & nh, std::string const& topic_name) :
        topic_name_(topic_name), parent_frame_id_("world"), child_frame_id_("body")
      {
        pub_path_ = nh.advertise<nav_msgs::Path>(topic_name, 4);
      }

      void publish(const geometry_msgs::PoseStamped & pose){
        path_msg_.header      = pose.header;
        path_msg_.poses.push_back(pose);
        pub_path_.publish(path_msg_);
      }
      void publish(const geometry_msgs::Pose & pose, const ros::Time& time){
        path_msg_.header.stamp = time;
        path_msg_.header.seq += 1;
        path_msg_.header.frame_id = parent_frame_id_;

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = path_msg_.header;
        pose_stamped.pose = pose;
        path_msg_.poses.push_back(pose_stamped);
        pub_path_.publish(path_msg_);
      }

      void publish(const geometry_msgs::Point & point, const ros::Time& time){
        path_msg_.header.stamp = time;
        path_msg_.header.seq += 1;
        path_msg_.header.frame_id = parent_frame_id_;

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = path_msg_.header;
        pose_stamped.pose.position = point;
        path_msg_.poses.push_back(pose_stamped);
        pub_path_.publish(path_msg_);
      }

    private:
      std::string parent_frame_id_;
      std::string child_frame_id_;
      std::string topic_name_;
      geometry_msgs::PoseStamped  pose_;
      nav_msgs::Path path_msg_;
      ros::Publisher pub_path_;
  };


  class TFPublisher {
    public:
    TFPublisher() {

    }

    void publish(geometry_msgs::PoseStamped const& msg,
                 std::string const parent_frame_id = "world",
                 std::string const child_frame_id = "body")
    {
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z) );
      tf::Quaternion q;
      q.setW(msg.pose.orientation.w);
      q.setX(msg.pose.orientation.x);
      q.setY(msg.pose.orientation.y);
      q.setZ(msg.pose.orientation.z);
      transform.setRotation(q);
      br_.sendTransform(tf::StampedTransform(transform, msg.header.stamp, parent_frame_id, child_frame_id));
    }

    void publish(geometry_msgs::Pose const& msg,
                 std::string const parent_frame_id = "world",
                 std::string const child_frame_id = "body")
    {
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(msg.position.x, msg.position.y, msg.position.z) );
      tf::Quaternion q;
      q.setW(msg.orientation.w);
      q.setX(msg.orientation.x);
      q.setY(msg.orientation.y);
      q.setZ(msg.orientation.z);
      transform.setRotation(q);
      br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame_id, child_frame_id));
    }

    void publish(geometry_msgs::PointStamped const& msg,
                 std::string const parent_frame_id = "world",
                 std::string const child_frame_id = "body")
    {
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(msg.point.x, msg.point.y, msg.point.z) );
      br_.sendTransform(tf::StampedTransform(transform, msg.header.stamp, parent_frame_id, child_frame_id));
    }

    void publish(geometry_msgs::Point const& msg,
                 std::string const parent_frame_id = "world",
                 std::string const child_frame_id = "body")
    {
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(msg.x, msg.y, msg.z) );
      br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame_id, child_frame_id));
    }



  private:
    tf::TransformBroadcaster br_;
  };
}



#endif // ROS_UTILS_HPP
