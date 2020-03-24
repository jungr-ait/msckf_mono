/******************************************************************************
* FILENAME:     utils
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr - Roland Jung
* MAIL:         Roland.Jung@ait.ac.at
* VERSION:      v1.0.0
* CREATION:     29.9.2017
*
*  Copyright (C) 2017 AIT Austrian Institute of Technology GmbH
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
#ifndef VISION_CORE_UTILS_HPP
#define VISION_CORE_UTILS_HPP
#include <iostream>
#include <sstream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <sophus/se3.hpp>
#include <sophus/sim3.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

// TODO: move print and toString into a dedicated header!
namespace vision_core
{
  /**
  * @brief collection of convertion and print functions
  */
  namespace utils
  {
    // 2x1 CV -> 2x1 EIGEN
    template<typename T = double>
    inline Eigen::Matrix<T, 2, 1> cv2Eigen(const cv::Point2d& pt)
    {
      return Eigen::Matrix<T, 2, 1>(pt.x, pt.y);
    }

    template<typename T = float>
    inline Eigen::Matrix<T, 2, 1> cv2Eigen(const cv::Point2f& pt)
    {
      return Eigen::Matrix<T, 2, 1>(pt.x, pt.y);
    }

    template<typename T = double>
    inline Eigen::Matrix<T, 2, 1> cv2EigenFloat2Double(const cv::Point2f& pt)
    {
      return Eigen::Matrix<T, 2, 1>(pt.x, pt.y);
    }

    // 3x1 CV -> 3x1 EIGEN
    template<typename T = float>
    inline Eigen::Matrix<T, 3, 1> cv2Eigen(const cv::Point3f& pt)
    {
      return Eigen::Matrix<T, 3, 1>(pt.x, pt.y, pt.z);
    }

    template<typename T = double>
    inline Eigen::Matrix<T, 3, 1> cv2Eigen(const cv::Point3d& pt)
    {
      return Eigen::Matrix<T, 3, 1>(pt.x, pt.y, pt.z);
    }

    // 2x1 EIGEN -> 2x1 CV
    static inline cv::Point2i eigen2CV(const Eigen::Vector2i& pt)
    {
      return cv::Point2i(pt.x(), pt.y());
    }

    static inline cv::Point2f eigen2CV(const Eigen::Vector2f& pt)
    {
      return cv::Point2f(pt.x(), pt.y());
    }

    static inline cv::Point2d eigen2CV(const Eigen::Vector2d& pt)
    {
      return cv::Point2d(pt.x(), pt.y());
    }

    // 3x1 EIGEN -> 3x1 CV
    static inline cv::Point3f eigen2CV(const Eigen::Vector3f& pt)
    {
      return cv::Point3f(pt.x(), pt.y(), pt.z());
    }

    static inline cv::Point3d eigen2CV(const Eigen::Vector3d& pt)
    {
      return cv::Point3d(pt.x(), pt.y(), pt.z());
    }

    static inline cv::Point3i eigen2CV(const Eigen::Vector3i& pt)
    {
      return cv::Point3i(pt.x(), pt.y(), pt.z());
    }

    // 3x1 EIGEN -> 2x1 EIGEN
    template<typename T>
    static inline Eigen::Matrix<T, 2, 1> vec2(Eigen::Matrix<T, 3, 1> const& vec)
    {
      return Eigen::Matrix<T, 2, 1>(vec.x(), vec.y());
    }

    static inline Eigen::Matrix<double, 2, 1> vec2(Eigen::Vector3d const& vec)
    {
      return Eigen::Matrix<double, 2, 1>(vec.x(), vec.y());
    }

    // 3x1 CV -> 2x1 CV
    static inline cv::Point2i vec2(const cv::Point3i& pt)
    {
      return cv::Point2i(pt.x, pt.y);
    }

    static inline cv::Point2f vec2(const cv::Point3f& pt)
    {
      return cv::Point2f(pt.x, pt.y);
    }

    static inline cv::Point2d vec2(const cv::Point3d& pt)
    {
      return cv::Point2d(pt.x, pt.y);
    }


    static inline std::string print(Eigen::Quaterniond const& q)
    {
      std::stringstream str;
      str << " (x,y,z,w) " << q.x() << " " << q.y() << " " << q.z() << " " << q.w();
      return str.str();
    }

    static inline std::string print(Eigen::Vector3d const& p)
    {
      std::stringstream str;
      str << " (x,y,z) " << p.x() << " " << p.y() << " " << p.z();
      return str.str();
    }

    template<typename T>
    static inline std::string toString(Eigen::Quaternion<T> const& q)
    {
      std::stringstream str;
      str << "(wxyz)[" << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << "]";
      return str.str();
    }

    template<typename T>
    static inline std::string toString(Eigen::Matrix<T, 3, 1> const& t)
    {
      std::stringstream str;
      str << "(xyz)[" << t(0) << "," << t(1) << "," << t(2) << "]";
      return str.str();
    }

    template<typename T>
    static inline std::string toStringRot(Eigen::Matrix<T, 3, 3> const& R)
    {
      Eigen::Matrix<T, 3, 1> euler = R.eulerAngles(2, 1, 0);
      std::stringstream      str;
      str << "(YPR)[" << euler[0] << "," << euler[1] << "," << euler[2] << "]";
      return str.str();
    }

    template<typename T>
    static inline std::string toString(Sophus::SE3Group<T> const& s)
    {
      std::stringstream str;
      str << "S3E: " << toString(s.unit_quaternion()) << "; " << toString(s.translation());
      return str.str();
    }

    // TODO rename to toSE3f
    static inline Sophus::SE3f toSophus(cv::Mat const& pose)
    {
      assert(pose.cols == 4 && pose.rows == 4 && pose.type() == CV_32F);
      cv::Mat tcw = pose.rowRange(0, 3).col(3).clone();
      cv::Mat Rcw = pose.rowRange(0, 3).colRange(0, 3).clone();

      Eigen::Matrix3f eRcw;
      Eigen::Vector3f eTcw;
      cv::cv2eigen(tcw, eTcw);
      cv::cv2eigen(Rcw, eRcw);
      Eigen::Quaternionf q_Rcw(eRcw);
      Sophus::SE3f       res(q_Rcw, eTcw);

      return res;
    }
    // TODO add  toSE3d(cv::Mat)

    static inline cv::Mat toCVMat(Sophus::SE3f const& s_pose)
    {
      cv::Mat cv_Pose = cv::Mat::eye(4, 4, CV_32F);
      cv::Mat cv_T;
      cv::Mat cv_R    = cv::Mat(3, 3, CV_32F);

      cv::eigen2cv(s_pose.rotationMatrix(), cv_R);
      cv::eigen2cv(s_pose.translation(), cv_T);

      cv_R.copyTo(cv_Pose.rowRange(0, 3).colRange(0, 3));
      cv_T.copyTo(cv_Pose.rowRange(0, 3).col(3));

      return cv_Pose;
    }

    static inline cv::Mat toCVMat(Eigen::Quaternionf const& q_R, Eigen::Vector3f const& T)
    {
      return toCVMat(Sophus::SE3f(q_R, T));
    }

    // TODO: add toCVMat(Quaterniond, Vector3d)

    static inline void toEigen(cv::Mat const& cvPose, Eigen::Matrix3d& R, Eigen::Vector3d& t)
    {
      assert(cvPose.cols == 4 && cvPose.rows == 4 && cvPose.type() == CV_64F);
      cv::cv2eigen(cvPose.rowRange(0, 3).col(3), t);
      cv::cv2eigen(cvPose.rowRange(0, 3).colRange(0, 3), R);
    }

    static inline void toEigen(cv::Mat const& cvPose, Eigen::Quaterniond& q, Eigen::Vector3d& t)
    {
      //assert(cvPose.cols == 4 && cvPose.rows == 4 && cvPose.type() == CV_64F);
      cv::cv2eigen(cvPose.rowRange(0, 3).col(3), t);
      Eigen::Matrix3d R;
      cv::cv2eigen(cvPose.rowRange(0, 3).colRange(0, 3), R);
      q = Eigen::Quaterniond(R);
    }

    static inline void toEigen(cv::Mat const& cvPose, Eigen::Quaternionf& q, Eigen::Vector3f& t)
    {
      assert(cvPose.cols == 4 && cvPose.rows == 4 && cvPose.type() == CV_32F);
      cv::cv2eigen(cvPose.rowRange(0, 3).col(3), t);
      Eigen::Matrix3f R;
      cv::cv2eigen(cvPose.rowRange(0, 3).colRange(0, 3), R);
      q = Eigen::Quaternionf(R);
    }

    template<typename T>
    inline Sophus::Sim3Group<T> toSim3(Sophus::SE3Group<T> const& se3, T const scale)
    {
      Sophus::Sim3Group<T> sim3(se3.unit_quaternion(), se3.translation());
      sim3.setScale(scale);
      return sim3;
    }

    template<typename T>
    inline Sophus::SE3Group<T> toSE3(Sophus::Sim3Group<T> const& sim3)
    {
      Sophus::SE3Group<T> se3(sim3.quaternion(), sim3.translation()/sim3.scale());


      return se3;
    }

  } // namespace utils
} // namespace vision_core

#endif // VISION_CORE_UTILS_HPP
