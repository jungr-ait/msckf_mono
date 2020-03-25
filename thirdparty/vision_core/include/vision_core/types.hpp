/******************************************************************************
* FILENAME:     types
* PURPOSE:      %{Cpp:License:ClassName}
* AUTHOR:       jungr - Roland Jung
* VERSION:      v1.0.0
* CREATION:     29.9.2017
*
******************************************************************************/
#ifndef VISION_CORE_TYPES_HPP
#define VISION_CORE_TYPES_HPP
#include <eigen3/Eigen/Core>
#include <vector>

namespace vision_core
{
  typedef double TPrecision;
} // namespace vision_core

namespace Eigen
{
  typedef Eigen::Matrix<vision_core::TPrecision, 2, 1> Vector2;
  typedef Eigen::Matrix<vision_core::TPrecision, 3, 1> Vector3;
  typedef Eigen::Quaternion<vision_core::TPrecision>   QuaternionT;
  typedef Eigen::Matrix<vision_core::TPrecision, 5, 1> Vector5;
  typedef Eigen::Matrix<vision_core::TPrecision, 3, 3> Matrix3;
  typedef Eigen::Matrix<vision_core::TPrecision, 4, 4> Matrix4;
  typedef Eigen::Matrix<vision_core::TPrecision, 3, 4> Matrix3x4;
} // namespace Eigen

namespace vision_core
{
  typedef Eigen::Vector2d                                           TPoint2D;
  typedef Eigen::Vector3d                                           TPoint3D;
  typedef std::vector<TPoint2D, Eigen::aligned_allocator<TPoint2D>> TPoints2D;
  typedef std::vector<TPoint3D, Eigen::aligned_allocator<TPoint3D>> TPoints3D;
} // namespace vision_core
#endif // VISION_CORE_TYPES_HPP
