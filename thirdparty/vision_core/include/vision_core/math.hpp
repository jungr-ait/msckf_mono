/******************************************************************************
* FILENAME:     math
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr - Roland Jung
* VERSION:      v1.0.0
* CREATION:     29.9.2017

******************************************************************************/
#ifndef VISION_CORE_MATH_HPP
#define VISION_CORE_MATH_HPP
#include <set>
#include <algorithm>
#include <iterator>
#include <vision_core/utils.hpp>

// TODO: split up into vision-based math(project), geo(transform) and algorithms (median, diff)

namespace vision_core
{
  /**
  * @brief collection of functions to manipulate transforms, geometry, rotation matrices, quaternions, etc.
  */
  namespace math
  {

    template<typename T>
    static inline T toDeg(T const rad)
    {
      return (rad * 180.0) / M_PI;
    }

    template<typename T>
    static inline T toRad(T const deg)
    {
      return (deg * M_PI) / 180.0;
    }

    /**
     * @brief wrap to ]-pi,pi]
     *
     * @param x
     * @return
     */
    template<typename T>
    static inline T normalizeRad(T x)
    {
      x = static_cast<T>(fmod(x + M_PI, M_PI + M_PI));
      if(x <= 0.0)
      {
        x += (M_PI + M_PI);
      }

      return x - M_PI;
    }

    /**
     * @brief wrap to ]-180, 180]
     *
     * @param x
     * @return
     */
    template<typename T>
    static inline T normalizeDeg(T x)
    {
      x = static_cast<T>(fmod(x + 180.0, 360.0));
      if(x <= 0.0)
      {
        x += 360.0;
      }

      return x - 180.0;
    }

    /**
     * @brief apply rotation and translation on pose
     * @param cvPose input pose
     * @param q_R quaternion (rotation)
     * @param T translation
     * @return transformed pose
    */
    static inline cv::Mat applyTransformation(cv::Mat const& cvPose,
                                              Eigen::Quaternionf const& q_R,
                                              Eigen::Vector3f const& T)
    {
      Sophus::SE3f s_trans(q_R, T);

      Sophus::SE3f s_pose = utils::toSophus(cvPose);
      s_pose *= s_trans;

      return utils::toCVMat(s_pose);
    }

    /**
     * @brief apply inline rotation and translation on pose (q_, p_)
     * @param q_ input/output rotation
     * @param p_ input/output position
     * @param R quaternion (rotation)
     * @param t translation
    */
    static void applyTransformation(Eigen::Quaterniond& q_,
                                    Eigen::Vector3d& p_,
                                    Eigen::Quaterniond const& R,
                                    Eigen::Vector3d const& t)
    {
      Sophus::SE3d s_trans(R, t);
      Sophus::SE3d s_pose = Sophus::SE3d(q_, p_);
      s_pose *= s_trans;

      q_ = s_pose.unit_quaternion();
      p_ = s_pose.translation();
    }

    static inline float euclidianDistance(cv::Mat const& pose_a, cv::Mat const& pose_b)
    {
      return cv::norm(pose_a, pose_b, cv::NORM_L2);
    }

    static inline Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)
    {
      Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

      Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
      q.normalize();
      return q;
    }

    static inline Eigen::Quaterniond euler_deg2Quaternion(const double roll_deg,
                                                          const double pitch_deg,
                                                          const double yaw_deg)
    {
      Eigen::AngleAxisd rollAngle(toRad(normalizeDeg(roll_deg)), Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(toRad(normalizeDeg(pitch_deg)), Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yawAngle(toRad(normalizeDeg(yaw_deg)), Eigen::Vector3d::UnitZ());

      Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
      q.normalize();
      return q;
    }

    template<typename T>
    static inline void quaternion2EulerRad(Eigen::Quaternion<T> const& q, T& roll, T& pitch, T& yaw)
    {
      Eigen::Matrix<T, 3, 1> euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
      yaw   = euler[0];
      pitch = euler[1];
      roll  = euler[2];
    }

    /**
     * @brief return angles between -180° and 180°
     *
     * @tparam T
     * @param q
     * @param roll_deg
     * @param pitch_deg
     * @param yaw_deg
     */
    template<typename T>
    static inline void quaternion2EulerDeg(Eigen::Quaternion<T> const& q, T& roll_deg, T& pitch_deg, T& yaw_deg)
    {
      Eigen::Matrix<T, 3, 1> euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
      yaw_deg   = normalizeDeg(toDeg(euler[0]));
      pitch_deg = normalizeDeg(toDeg(euler[1]));
      roll_deg  = normalizeDeg(toDeg(euler[2]));
    }

    /**
     * @brief convert euler angles to rotation matrix
     * @param roll
     * @param pitch
     * @param yaw
     * @return rotation matrix
     */
    static inline Eigen::Matrix3d euler2Rxyz(const double roll, const double pitch, const double yaw)
    {
      Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

      Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
      return q.matrix();
    }

    /**
     * calculate relative transformation between T1 and T2 \n
     * T2 = T12 * T1
     * @param T1
     * @param T2
     * @return T12
     */
    static inline Sophus::SE3d T12(Sophus::SE3d const& T1, Sophus::SE3d const& T2)
    {
      return T2 * T1.inverse();
    }

    /**
     * calculate relative transformation between T2 and T1 \n
     * T1 = T21 * T2
     * @param T1
     * @param T2
     * @return T21
     */
    static inline Sophus::SE3d T21(Sophus::SE3d const& T1, Sophus::SE3d const& T2)
    {
      return T12(T2, T1);
    }

    static inline Eigen::Matrix3d toSkewSymmetric(Eigen::Vector3d const& t)
    {
      Eigen::Matrix3d t12x;
      t12x << 0, -t.z(), t.y(), t.z(), 0, -t.x(), -t.y(), t.x(), 0;
      return t12x;
    }

    /**
     * @brief compute the fundamental matrix between f1 and f2
     * @param f1 frame1 pose Tcw
     * @param K1 camera intrinsics of frame1
     * @param f2 frame2 pose Tcw
     * @param K2 camera intrinsics of frame2
     * @return fundamental matrix
     */
    static inline Eigen::Matrix3d F12(Sophus::SE3d const& f1,
                                      Eigen::Matrix3d const& K1,
                                      Sophus::SE3d const& f2,
                                      Eigen::Matrix3d const& K2)
    {
      Sophus::SE3d    T    = T12(f1.inverse(), f2.inverse());
      Eigen::Matrix3d t12x = toSkewSymmetric(T.translation());
      return K1.transpose().inverse() * t12x * T.rotationMatrix() * K2.inverse();
    }

    /***
     * @brief compute the squared distance [px] from p2 to the epipolar line of p1
     * @param p1 point in frame 1
     * @param p2 point in frame 2
     * @param F12 transposed fundamental matrix
     * @return squared distance [px] to epipolar line from p1
     */
    template<typename TPrecision>
    static inline TPrecision distanceToEpipolarLine(Eigen::Matrix<TPrecision, 2, 1> const& p1,
                                                    Eigen::Matrix<TPrecision, 2, 1> const& p2,
                                                    Eigen::Matrix<TPrecision, 3, 3> const& F12t)
    {
      //   Epipolar line in second image l = x1'F12 = [a b c]
//      Eigen::Matrix<TPrecision, 3, 3> const F12t = F12.transpose();
      Eigen::Matrix<TPrecision, 3, 1> const e = F12t * p1.homogeneous();

      //d(e,x2) = (ax2_x + bx2_y + c) / sqrt(a^2 + b^2)
      TPrecision const d_nom = e.dot(p2.homogeneous());
      TPrecision const d_den = e.x() * e.x() + e.y() * e.y();

      if(d_den == 0)
      {
        return std::numeric_limits<double>::max();
      }

      TPrecision const d = d_nom * d_nom / d_den;
      return d;
    }

    static inline Eigen::Vector2d project2D(Eigen::Vector3d const& pt)
    {
      return pt.head<2>() / pt.z();
    }

    static inline Eigen::Vector2f project2D(Eigen::Vector3f const& pt)
    {
      return pt.head<2>() / pt.z();
    }


    template<typename T>
    inline T radAngleDiff(T a, T b)
    {
      double const M_2PI = M_PI + M_PI;
      double       d     = fmod(b - a + M_PI, M_2PI);

      if(d < 0.0)
      {
        d += M_2PI;
      }

      return d - M_PI;
    }

    template<typename T>
    inline T degAngleDiff(T a, T b)
    {
      double d = fmod(b - a + 180.0, 360.0);

      if(d < 0.0)
      {
        d += 360.0;
      }

      return d - 180.0;
    }


    // unwrap to -inf, inf
    template<typename T>
    static inline T degUnwarp(T const prev_x, T const x)
    {
      return prev_x - degAngleDiff(x, normalizeDeg(prev_x));
    }

    // unwrap to -inf, inf
    template<typename T>
    static inline T radUnwarp(T const prev_x, T const x)
    {
      return prev_x - radAngleDiff(x, normalizeRad(prev_x));
    }

    template<typename T>
    class UnwrapAngle
    {
      public:


      T unwrapDeg(T const x)
      {
        if(!mbInit)
        {
          set(x);
        }

        mPrev = degUnwarp(mPrev, x);
        return mPrev;
      }

      T unwrapRad(T const x)
      {
        if(!mbInit)
        {
          set(x);
        }

        mPrev = radUnwarp(mPrev, x);
        return mPrev;
      }

      void set(T const x)
      {
        mPrev  = x;
        mbInit = true;
      }

      private:
      bool mbInit = false;
      T    mPrev  = 0.0;
    }; // UnwrapAngle_deg

  } // math
}  // vision_core

#endif // VISION_CORE_MATH_HPP
