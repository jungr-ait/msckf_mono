#ifndef VSLAMFW_DEMO_ROS_HELPER_HPP
#define VSLAMFW_DEMO_ROS_HELPER_HPP
#include <iostream>
#include <iomanip>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/se3.hpp>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

namespace msckf
{
  /**
   *
   * @return current time formated as string HH-mm-ss
   */

  /**
   *
   * @param p ROS stamped pose to print
   * @return string representation of the pose
   */
  static std::string toString(geometry_msgs::PoseStamped const& p)
  {
    std::stringstream ss;
    ss << std::setprecision(std::numeric_limits < long
    double > ::digits10 + 1)
    << p.header.stamp.toSec() << " " << std::setprecision(8) << "[sec] - " << static_cast<float>(p.pose.position.x) << ","
        << static_cast<float>(p.pose.position.y) << "," << static_cast<float>(p.pose.position.z) << "(xyz)/"
        << static_cast<float>(p.pose.orientation.x) << "," << static_cast<float>(p.pose.orientation.y) << ","
        << static_cast<float>(p.pose.orientation.z) << "," << static_cast<float>(p.pose.orientation.w) << "(xyzw)";
    return ss.str();
  }

  static std::string toString(geometry_msgs::Pose const& p)
  {
    std::stringstream ss;
    ss << std::setprecision(std::numeric_limits < long
    double > ::digits10 + 1)
        << static_cast<float>(p.position.x) << ","
        << static_cast<float>(p.position.y) << "," << static_cast<float>(p.position.z) << "(xyz)/"
        << static_cast<float>(p.orientation.x) << "," << static_cast<float>(p.orientation.y) << ","
        << static_cast<float>(p.orientation.z) << "," << static_cast<float>(p.orientation.w) << "(xyxz)";
    return ss.str();
  }

  template<typename T>
  static geometry_msgs::Point toRos(Eigen::Matrix<T, 3, 1> const& p, float const s = 1.0f)
  {
    geometry_msgs::Point r;
    r.x = p.x() * s;
    r.y = p.y() * s;
    r.z = p.z() * s;
    return r;
  }

  template<typename T>
  static geometry_msgs::Quaternion toRos(Eigen::Quaternion<T> const &q)
  {
    geometry_msgs::Quaternion r;
    r.x = q.x();
    r.y = q.y();
    r.z = q.z();
    r.w = q.w();
    return r;
  }

  template<typename T>
  static geometry_msgs::Pose toRos(Sophus::SE3<T> const & pose)
  {
    geometry_msgs::Pose pose_;
    pose_.orientation = toRos(Eigen::Quaternion<T>(pose.rotationMatrix()));
    pose_.position = toRos(pose.translation());
    return pose_;
  }

  template<typename T>
  static geometry_msgs::Pose toRos(Eigen::Matrix<T, 3, 1> const& p, Eigen::Quaternion<T> const &q)
  {
    geometry_msgs::Pose pose_;
    pose_.orientation = toRos(q);
    pose_.position = toRos(p, 1);
    return pose_;
  }


  static void toRos(geometry_msgs::PoseStamped& msg,
                    std::string const& frame, int frame_number, double stamp,
                     Sophus::SE3d const& pose)
  {
    msg.header.stamp       = ros::Time(stamp);
    msg.header.seq         = frame_number;
    msg.header.frame_id    = frame;
    msg.pose.position = toRos(pose.translation());
    msg.pose.orientation = toRos(Eigen::Quaterniond(pose.rotationMatrix()));
  }

  static void toRos(geometry_msgs::PoseStamped& msg,
                    std::string const& frame, int frame_number, double stamp,
                    Eigen::Quaterniond q, Eigen::Vector3d p)
  {
    msg.header.stamp       = ros::Time(stamp);
    msg.header.seq         = frame_number;
    msg.header.frame_id    = frame;
    msg.pose.position = toRos(p);
    msg.pose.orientation = toRos(q);
  }


  static Eigen::Quaterniond toEigen(geometry_msgs::Quaternion const &q)
  {
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  }

  static Eigen::Vector3d toEigen(geometry_msgs::Point const &p)
  {
    return Eigen::Vector3d( p.x, p.y, p.z);
  }

  static Sophus::SE3d toSophus(geometry_msgs::Pose const& pose)
  {
    Eigen::Quaterniond q = toEigen(pose.orientation);
  //  (msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    Eigen::Vector3d    p = toEigen(pose.position);
  //  (msg.position.x, msg.position.y, msg.position.z);
    return Sophus::SE3d(q, p);
  }

  static geometry_msgs::Pose invert(geometry_msgs::Pose const& pose)
  {
    Sophus::SE3d pose_ = toSophus(pose);
    pose_ = pose_.inverse();
    return toRos(pose_);
  }

}
#endif

