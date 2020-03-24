#ifndef MSCKF_MONO_ROS_INTERFACE_H_
#define MSCKF_MONO_ROS_INTERFACE_H_

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <msckf_mono/types.h>
#include <msckf_mono/msckf.h>
#include <msckf_mono/corner_detector.h>
#include <atomic>

namespace msckf_mono
{
  class RosInterface {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      RosInterface();

      void imuCallback(const sensor_msgs::ImuConstPtr& imu);

      void imageCallback(const sensor_msgs::ImageConstPtr& msg);

      void publish_core(const ros::Time& publish_time);

      void publish_extra(const ros::Time& publish_time);

      void load_ROS_parameters();

      bool load_YAML_parameters(std::string const& filename);


  private:
      enum CalibrationMethod { TimedStandStill };

      //(cx, cy) is a principal point that is usually at the image center
      //fx, fy are the focal lengths expressed in pixel units.
      void set_camera_intrinsics(float const fx, float const fy, float const cx, float const cy);

      // k_1, k_2, k_3, k_4, k_5, and k_6 are radial distortion coefficients. p_1 and p_2 are tangential distortion coefficients.
      void set_distortion_coef(float const k1, float const k2, float const p1, float const p2);
      void set_cam_imu_extrinsics(Matrix3f const& R_C_I, Vector3f const& p_C_I);
      void set_imu_noise_params(float const w_var, float const dbg_var, float const a_var, float const dba_var);
      void set_P(float const q_var_init, float const bg_var_init, float const v_var_init, float const ba_var_init, float const p_var_init);

      void set_CAMERA_params(float const fx,
                        float const fy,
                        float const cx,
                        float const cy,
                        float const k1,
                        float const k2,
                        float const p1,
                        float const p2,
                        std::string const& distortion_model,
                        Matrix3f const& R_C_I,
                        Vector3f const& p_C_I);

      void set_NOISE_params(float const fx,
                            float const fy,
                            float const feature_covariance = 7,
                            float const w_var = 1e-5,
                            float const dbg_var = 3.6733e-5,
                            float const a_var = 1e-3,
                            float const dba_var = 7e-4,
                            float const q_var_init = 1e-5,
                            float const bg_var_init = 1e-2,
                            float const v_var_init = 1e-2,
                            float const ba_var_init = 1e-2,
                            float const p_var_init = 1e-12);
;

      void set_MSCKF_params(const float fx,
                            float const max_gn_cost_norm = 11,
                            float const translation_threshold = 0.05,
                            float const min_rcond = 3e-12,
                            float const keyframe_transl_dist = 0.005,
                            float const keyframe_rot_dist = 0.05,
                            float const max_track_length = 1000,
                            float const min_track_length = 3,
                            float const max_cam_states = 20);

      void set_TRACKER_params(float const n_grid_rows = 8,
                              float const n_grid_cols = 8,
                              float const ransac_threshold = 0.000002);

      void set_INITIALIZATON_params(float const stand_still_time,
                                     CalibrationMethod const method);

      void dump_info();

      ros::NodeHandle nh_;
      image_transport::ImageTransport it_;

      image_transport::Subscriber image_sub_;
      image_transport::Publisher track_image_pub_;
      ros::Publisher odom_pub_;

      ros::Subscriber imu_sub_;





      bool debug_;

      std::vector<std::tuple<double, imuReading<float>>> imu_queue_;
      double prev_imu_time_;

      void setup_track_handler();
      std::shared_ptr<corner_detector::TrackHandler> track_handler_;

      Matrix3<float> R_imu_cam_;
      Vector3<float> p_imu_cam_;

      Matrix3<float> R_cam_imu_;
      Vector3<float> p_cam_imu_;

      std::string camera_model_;
      cv::Mat K_;
      std::string distortion_model_;
      cv::Mat dist_coeffs_;

      int n_grid_cols_;
      int n_grid_rows_;
      float ransac_threshold_;


      CalibrationMethod imu_calibration_method_;

      double stand_still_time_;
      double done_stand_still_time_;

      std::atomic<bool> imu_calibrated_;
      bool can_initialize_imu();
      void initialize_imu();

      int state_k_;
      void setup_msckf();
      MSCKF<float> msckf_;
      Camera<float> camera_;
      noiseParams<float> noise_params_;
      MSCKFParams<float> msckf_params_;
      imuState<float> init_imu_state_;
  };
}

#endif
