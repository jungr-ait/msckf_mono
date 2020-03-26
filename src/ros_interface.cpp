#include <msckf_mono/ros_interface.h>
#include <iomanip>      // std::setprecision
#include <opencv/cxeigen.hpp>
#include <utilities/IO.hpp>
#include <utilities/RTVerification.hpp>
#include <vision_core/config_helper.hpp>
#include <msckf_mono/helper.hpp>

namespace msckf_mono
{
  RosInterface::RosInterface() :
    nh_("~"),
    it_(nh_),
    imu_calibrated_(false),
    prev_imu_time_(0.0),
    path_pub_est_(nh_, "path_est"),
    path_pub_gt_(nh_, "path_gt"),
    map_pub_(nh_, "map")
  {

  }

  void RosInterface::init_ROS()
  {
    load_ROS_parameters();
    setup_track_handler();

    init_topics();
  }

  bool RosInterface::init_YAML(const std::string &filename)
  {
    bool res = load_YAML_parameters(filename);
    setup_track_handler();

    init_topics();
    dump_info();
    return res;
  }

  void RosInterface::imuCallback(const sensor_msgs::ImuConstPtr& imu)
  {
    double cur_imu_time = imu->header.stamp.toSec();
    if(prev_imu_time_ == 0.0){
      prev_imu_time_ = cur_imu_time;
      done_stand_still_time_ = cur_imu_time + stand_still_time_;
      ros::Time time_over(done_stand_still_time_);

      std::cout << "* Stand still is expected to be over at: "
                << " " << time_over.sec << "." << time_over.nsec << " [sec]" << std::endl;
      return;
    }

    imuReading<float> current_imu;

    current_imu.a[0] = imu->linear_acceleration.x;
    current_imu.a[1] = imu->linear_acceleration.y;
    current_imu.a[2] = imu->linear_acceleration.z;

    current_imu.omega[0] = imu->angular_velocity.x;
    current_imu.omega[1] = imu->angular_velocity.y;
    current_imu.omega[2] = imu->angular_velocity.z;

    current_imu.dT = cur_imu_time - prev_imu_time_;

    std::tuple<double, imuReading<float>> val(cur_imu_time, current_imu);
    imu_queue_.emplace_back(val);

    prev_imu_time_ = cur_imu_time;
//    if(imu_calibrated_)
//    {
//      msckf_.propagate(current_imu);
//      publish_est();
//    }
  }

  void RosInterface::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    double cur_image_time = msg->header.stamp.toSec();
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if(!imu_calibrated_){
      if(imu_queue_.size() % 100 == 0){
        ROS_INFO_STREAM("Has " << imu_queue_.size() << " readings");
      }

      if(can_initialize_imu()){

        std::cout << "can initialize imu now at: " << std::setprecision(std::numeric_limits < long double > ::digits10 + 1) << prev_imu_time_ << std::endl;
        initialize_imu();

        imu_calibrated_ = true;
        imu_queue_.clear();

        setup_msckf();
        std::cout << "IMU initalized and MSCKF setup..." << std::endl;
      }

      return;
    }

    std::vector<imuReading<float>> imu_since_prev_img;
    imu_since_prev_img.reserve(50);

    // get the first imu reading that belongs to the next image
    auto frame_end = std::find_if(imu_queue_.begin(), imu_queue_.end(),
        [&](const auto& x){return std::get<0>(x) > cur_image_time;});

    std::transform(imu_queue_.begin(), frame_end,
        std::back_inserter(imu_since_prev_img),
        [](auto& x){return std::get<1>(x);});

    imu_queue_.erase(imu_queue_.begin(), frame_end);

    for(imuReading<float>& reading : imu_since_prev_img){
      msckf_.propagate(reading);
      publish_est();

      Vector3<float> omega_in_C = camera_.q_CI.inverse().matrix() * (reading.omega - init_imu_state_.b_g);
      track_handler_->add_gyro_reading(omega_in_C);
    }

    track_handler_->set_current_image( cv_ptr->image, cur_image_time );

    std::vector<Vector2<float>,
      Eigen::aligned_allocator<Vector2<float>>> cur_features;
    corner_detector::IdVector cur_ids;
    track_handler_->tracked_features(cur_features, cur_ids);

    std::vector<Vector2<float>,
      Eigen::aligned_allocator<Vector2<float>>> new_features;
    corner_detector::IdVector new_ids;
    track_handler_->new_features(new_features, new_ids);

    msckf_.augmentState(state_k_, (float)cur_image_time);
    msckf_.update(cur_features, cur_ids);
    msckf_.addFeatures(new_features, new_ids);
    msckf_.marginalize();
    // msckf_.pruneRedundantStates();
    msckf_.pruneEmptyStates();

    publish_core(msg->header.stamp);
    publish_extra(msg->header.stamp);
    publish_est();
  }

  void RosInterface::point_GT_Callback(const geometry_msgs::PointStampedConstPtr &msg)
  {
    last_pose_stamped_gt_.header = msg->header;
    last_pose_stamped_gt_.pose.orientation.w = 1;
    last_pose_stamped_gt_.pose.orientation.x = 0;
    last_pose_stamped_gt_.pose.orientation.y = 0;
    last_pose_stamped_gt_.pose.orientation.z = 0;
    last_pose_stamped_gt_.pose.position = msg->point;

    publish_gt();
  }

  void RosInterface::pose_GT_Callback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    last_pose_stamped_gt_ = *msg;
    publish_gt();
  }

  void RosInterface::publish_core(const ros::Time& publish_time)
  {
    auto imu_state = msckf_.getImuState();

    nav_msgs::Odometry odom;
    odom.header.stamp = publish_time;
    odom.header.frame_id = parent_frame_id_;
    odom.twist.twist.linear.x = imu_state.v_I_G[0];
    odom.twist.twist.linear.y = imu_state.v_I_G[1];
    odom.twist.twist.linear.z = imu_state.v_I_G[2];

    odom.pose.pose.position.x = imu_state.p_I_G[0];
    odom.pose.pose.position.y = imu_state.p_I_G[1];
    odom.pose.pose.position.z = imu_state.p_I_G[2];
    Quaternion<float> q_out = imu_state.q_IG.inverse();
    odom.pose.pose.orientation.w = q_out.w();
    odom.pose.pose.orientation.x = q_out.x();
    odom.pose.pose.orientation.y = q_out.y();
    odom.pose.pose.orientation.z = q_out.z();

    odom_pub_.publish(odom);
  }

  void RosInterface::publish_extra(const ros::Time& publish_time)
  {
    if(track_image_pub_.getNumSubscribers() > 0){
      cv_bridge::CvImage out_img;
      out_img.header.frame_id = "cam0";
      out_img.header.stamp = publish_time;
      out_img.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
      out_img.image = track_handler_->get_track_image();
      track_image_pub_.publish(out_img.toImageMsg());
    }
    if(map_pub_.pub_.getNumSubscribers()>0)
    {
      map_pub_.publish(msckf_.getMap());
    }
  }

  void RosInterface::init_topics()
  {
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 100);
    track_image_pub_ = it_.advertise("track_overlay_image", 1);

    imu_sub_ = nh_.subscribe("imu", 200, &RosInterface::imuCallback, this);
    image_sub_ = it_.subscribe("image_mono", 20,
                               &RosInterface::imageCallback, this);

    gt_pose_sub_ = nh_.subscribe("pose_gt", 10, &RosInterface::pose_GT_Callback, this);
    gt_point_sub_ = nh_.subscribe("point_gt", 10, &RosInterface::point_GT_Callback, this);
  }

  void RosInterface::publish_est()
  {
    imuState<float> state = msckf_.getImuState();
    state.q_IG.normalize();
    geometry_msgs::Pose pose_I_G_ros = msckf::toRos(state.p_I_G, state.q_IG);
    tf_pub_imu_.publish(pose_I_G_ros, parent_frame_id_, child_frame_id_);
    path_pub_est_.publish(pose_I_G_ros, ros::Time::now());
  }

  void RosInterface::publish_gt()
  {
    tf_pub_imu_.publish(last_pose_stamped_gt_.pose, parent_frame_id_, "GT");
    path_pub_gt_.publish(last_pose_stamped_gt_.pose, ros::Time::now());
  }

  bool RosInterface::can_initialize_imu()
  {
    if(imu_calibration_method_ == TimedStandStill){
      return prev_imu_time_ > done_stand_still_time_;
    }

    return false;
  }

  void RosInterface::initialize_imu()
  {
    Eigen::Vector3f accel_accum;
    Eigen::Vector3f gyro_accum;
    int num_readings = 0;

    accel_accum.setZero();
    gyro_accum.setZero();

    for(const auto& entry : imu_queue_){
      auto imu_time = std::get<0>(entry);
      auto imu_reading = std::get<1>(entry);

      accel_accum += imu_reading.a;
      gyro_accum += imu_reading.omega;
      num_readings++;
    }
    std::cout << "number IMU readings for init: " << num_readings << std::endl;

    Eigen::Vector3f accel_mean = accel_accum / num_readings;
    Eigen::Vector3f gyro_mean = gyro_accum / num_readings;

    init_imu_state_.b_g = gyro_mean;
    init_imu_state_.g << 0.0, 0.0, -9.81;

    Eigen::Vector3f diff = accel_mean + init_imu_state_.g;
    if (diff.norm() < 0.1) {
      init_imu_state_.q_IG = Quaternionf(1,0,0,0);
      init_imu_state_.b_a << 0.0, 0.0, 0.0;
      init_imu_state_.b_g << 0.0, 0.0, 0.0;
      std::cout << "imu_init: imu is gravity aligned..." << std::endl;
      std::cout << "simulation detected, setting ground-truth values..." << std::endl;
    }
    else
    {
      init_imu_state_.q_IG = Quaternion<float>::FromTwoVectors(
          -init_imu_state_.g, accel_mean);
      init_imu_state_.b_a = init_imu_state_.q_IG*init_imu_state_.g + accel_mean;
    }



    init_imu_state_.v_I_G.setZero();
    if(!last_pose_stamped_gt_.header.stamp.isZero())
    {
      std::cout << "* using last pose_I_G (from G to I) for  init:" << std::endl;
      std::cout << msckf::toString(last_pose_stamped_gt_) << std::endl;

      geometry_msgs::Pose pose_I_G = last_pose_stamped_gt_.pose;
      init_imu_state_.p_I_G = msckf::toEigen(pose_I_G.position).cast <float> ();
      init_imu_state_.q_IG = msckf::toEigen(pose_I_G.orientation).cast <float> ();
    }
    else
    {
      std::cout << "* using default init:" << std::endl;
      init_imu_state_.p_I_G.setZero();
    }

    init_imu_state_.p_I_G_null = init_imu_state_.p_I_G;
    init_imu_state_.v_I_G_null = init_imu_state_.v_I_G;
    init_imu_state_.q_IG_null = init_imu_state_.q_IG;

    const auto q = init_imu_state_.q_IG;
    ROS_INFO_STREAM("\nInitial IMU State" <<
      "\n--p_I_G " << init_imu_state_.p_I_G.transpose() <<
      "\n--q_IG " << q.w() << "," << q.x() << "," << q.y() << "," << q.x() <<
      "\n--v_I_G " << init_imu_state_.v_I_G.transpose() <<
      "\n--b_a " << init_imu_state_.b_a.transpose() <<
      "\n--b_g " << init_imu_state_.b_g.transpose() <<
      "\n--g " << init_imu_state_.g.transpose());

    std::cout << std::endl;

  }

  void RosInterface::setup_track_handler()
  {
    track_handler_.reset( new corner_detector::TrackHandler(K_, dist_coeffs_, distortion_model_) );
    track_handler_->set_grid_size(n_grid_rows_, n_grid_cols_);
    track_handler_->set_ransac_threshold(ransac_threshold_);
    track_handler_->set_detection_threshold(corner_treshold_);
    track_handler_->set_tracker_max_dist(tracker_max_pixel_dist_);
  }

  void RosInterface::setup_msckf()
  {
    state_k_ = 0;
    msckf_.initialize(camera_, noise_params_, msckf_params_, init_imu_state_);
  }

  //////////////////////////////////////////////////////////////////////
  // HELPER CODE:
  //////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////
  /// \brief RosInterface::load_ROS_parameters
  ///
  void RosInterface::load_ROS_parameters()
  {
    std::string kalibr_camera;
    nh_.getParam("kalibr_camera_name", kalibr_camera);

    nh_.getParam(kalibr_camera+"/camera_model", camera_model_);

    K_ = cv::Mat::eye(3,3,CV_32F);
    std::vector<float> intrinsics(4);
    nh_.getParam(kalibr_camera+"/intrinsics", intrinsics);
    K_.at<float>(0,0) = intrinsics[0];
    K_.at<float>(1,1) = intrinsics[1];
    K_.at<float>(0,2) = intrinsics[2];
    K_.at<float>(1,2) = intrinsics[3];

    nh_.getParam(kalibr_camera+"/distortion_model", distortion_model_);

    std::vector<float> distortion_coeffs(4);
    nh_.getParam(kalibr_camera+"/distortion_coeffs", distortion_coeffs);
    dist_coeffs_ = cv::Mat::zeros(distortion_coeffs.size(),1,CV_32F);
    dist_coeffs_.at<float>(0) = distortion_coeffs[0];
    dist_coeffs_.at<float>(1) = distortion_coeffs[1];
    dist_coeffs_.at<float>(2) = distortion_coeffs[2];
    dist_coeffs_.at<float>(3) = distortion_coeffs[3];

    XmlRpc::XmlRpcValue ros_param_list;
    nh_.getParam(kalibr_camera+"/T_cam_imu", ros_param_list);
    ROS_ASSERT(ros_param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    
    Matrix4<float> T_cam_imu;
    for (int32_t i = 0; i < ros_param_list.size(); ++i) 
    {
      ROS_ASSERT(ros_param_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
      for(int32_t j=0; j<ros_param_list[i].size(); ++j){
        ROS_ASSERT(ros_param_list[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        T_cam_imu(i,j) = static_cast<double>(ros_param_list[i][j]);
      }
    }

    // setup camera parameters
    camera_.f_u = intrinsics[0];
    camera_.f_v = intrinsics[1];
    camera_.c_u = intrinsics[2];
    camera_.c_v = intrinsics[3];

    auto R_CI =  T_cam_imu.block<3,3>(0,0);
    auto p_CI =  T_cam_imu.block<3,1>(0,3);
    camera_.q_CI = Quaternion<float>(R_CI); // q_CI reads as CAMERA in IMU (from IMU to CAMERA)
    camera_.p_C_I = p_CI;  // p_C_I reads as CAMERA in IMU (from IMU to CAMERA in IMU)

    // Feature tracking parameteres
    nh_.param<int>("n_grid_rows", n_grid_rows_, 8);
    nh_.param<int>("n_grid_cols", n_grid_cols_, 8);

    float ransac_threshold_;
    nh_.param<float>("ransac_threshold_", ransac_threshold_, 0.000002);
    nh_.param<float>("corner_treshold", corner_treshold_, 40.0);
    // MSCKF Parameters
    float feature_cov;
    nh_.param<float>("feature_covariance", feature_cov, 7);

    Eigen::Matrix<float,12,1> Q_imu_vars;
    float w_var, dbg_var, a_var, dba_var;
    nh_.param<float>("imu_vars/w_var", w_var, 1e-5);
    nh_.param<float>("imu_vars/dbg_var", dbg_var, 3.6733e-5);
    nh_.param<float>("imu_vars/a_var", a_var, 1e-3);
    nh_.param<float>("imu_vars/dba_var", dba_var, 7e-4);
    Q_imu_vars << w_var, 	w_var, 	w_var,
                  dbg_var,dbg_var,dbg_var,
                  a_var,	a_var,	a_var,
                  dba_var,dba_var,dba_var;

    Eigen::Matrix<float,15,1> IMUCovar_vars;
    float q_var_init, bg_var_init, v_var_init, ba_var_init, p_var_init;
    nh_.param<float>("imu_covars/q_var_init", q_var_init, 1e-5);
    nh_.param<float>("imu_covars/bg_var_init", bg_var_init, 1e-2);
    nh_.param<float>("imu_covars/v_var_init", v_var_init, 1e-2);
    nh_.param<float>("imu_covars/ba_var_init", ba_var_init, 1e-2);
    nh_.param<float>("imu_covars/p_var_init", p_var_init, 1e-12);
    IMUCovar_vars << q_var_init, q_var_init, q_var_init,
                     bg_var_init,bg_var_init,bg_var_init,
                     v_var_init, v_var_init, v_var_init,
                     ba_var_init,ba_var_init,ba_var_init,
                     p_var_init, p_var_init, p_var_init;

    // Setup noise parameters
    noise_params_.initial_imu_covar = IMUCovar_vars.asDiagonal();
    noise_params_.Q_imu = Q_imu_vars.asDiagonal();
    noise_params_.u_var_prime = pow(feature_cov/camera_.f_u,2);
    noise_params_.v_var_prime = pow(feature_cov/camera_.f_v,2);

    nh_.param<float>("max_gn_cost_norm", msckf_params_.max_gn_cost_norm, 11);
    msckf_params_.max_gn_cost_norm = pow(msckf_params_.max_gn_cost_norm/camera_.f_u, 2);
    nh_.param<float>("translation_threshold", msckf_params_.translation_threshold, 0.05);
    nh_.param<float>("min_rcond", msckf_params_.min_rcond, 3e-12);
    nh_.param<float>("keyframe_transl_dist", msckf_params_.redundancy_angle_thresh, 0.005);
    nh_.param<float>("keyframe_rot_dist", msckf_params_.redundancy_distance_thresh, 0.05);
    nh_.param<int>("max_track_length", msckf_params_.max_track_length, 1000);
    nh_.param<int>("min_track_length", msckf_params_.min_track_length, 3);
    nh_.param<int>("max_cam_states", msckf_params_.max_cam_states, 20);

    // Load calibration time
    int method;
    nh_.param<int>("imu_initialization_method", method, 0);
    if(method == 0){
      imu_calibration_method_ = TimedStandStill;
    }
    nh_.param<double>("stand_still_time", stand_still_time_, 8.0);

    dump_info();
  }

  bool RosInterface::load_YAML_parameters(const std::string &filename)
  {
    std::cout << "RosInterface.load_YAML_parameters() " << filename << std::endl;
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if(!fs.isOpened())
    {
      std::cout << "Could not load " << filename << std::endl;
      return false;
    }

    float w_var, dbg_var, a_var, dba_var, q_var_init, bg_var_init, v_var_init, ba_var_init, p_var_init;
    float feature_cov, n_grid_rows, n_grid_cols, corner_threshold, max_pixel_dist;
    float max_gn_cost_norm, translation_threshold, min_rcond, keyframe_transl_dist,
        keyframe_rot_dist, min_track_length, max_track_length, max_cam_states, ransac_threshold;
    float fx, fy, cx,cy, k1, k2, p1,p2;
    std::string distortion_model = "radtan";
    float stand_still_time;
    Matrix3f R_CI;
    Vector3f p_C_I;
    cv::FileNode fn_;
    RTV_EXPECT_TRUE_(vision_core::config_helper::loadNode(fn_, fs, "NOISE"));
    {
            // noise
      RTV_EXPECT_TRUE_(fn_["w_var"].isNamed());
      RTV_EXPECT_TRUE_(fn_["dbg_var"].isNamed());
      RTV_EXPECT_TRUE_(fn_["a_var"].isNamed());
      RTV_EXPECT_TRUE_(fn_["dba_var"].isNamed());

      vision_core::config_helper::get(fn_, "w_var", w_var);
      vision_core::config_helper::get(fn_, "dbg_var", dbg_var);
      vision_core::config_helper::get(fn_, "a_var", a_var);
      vision_core::config_helper::get(fn_, "dba_var", dba_var);
      // init
      RTV_EXPECT_TRUE_(fn_["q_var_init"].isNamed());
      RTV_EXPECT_TRUE_(fn_["bg_var_init"].isNamed());
      RTV_EXPECT_TRUE_(fn_["v_var_init"].isNamed());
      RTV_EXPECT_TRUE_(fn_["ba_var_init"].isNamed());
      RTV_EXPECT_TRUE_(fn_["p_var_init"].isNamed());
      vision_core::config_helper::get(fn_, "q_var_init", q_var_init);
      vision_core::config_helper::get(fn_, "bg_var_init", bg_var_init);
      vision_core::config_helper::get(fn_, "v_var_init", v_var_init);
      vision_core::config_helper::get(fn_, "ba_var_init", ba_var_init);
      vision_core::config_helper::get(fn_, "p_var_init", p_var_init);
    }
    RTV_EXPECT_TRUE_(vision_core::config_helper::loadNode(fn_, fs, "TRACKER"));
    {

      RTV_EXPECT_TRUE_(fn_["feature_cov"].isNamed());
      RTV_EXPECT_TRUE_(fn_["n_grid_rows"].isNamed());
      RTV_EXPECT_TRUE_(fn_["n_grid_cols"].isNamed());
      RTV_EXPECT_TRUE_(fn_["corner_threshold"].isNamed());
      RTV_EXPECT_TRUE_(fn_["max_pixel_dist"].isNamed());
      vision_core::config_helper::get(fn_, "feature_cov", feature_cov);
      vision_core::config_helper::get(fn_, "n_grid_rows", n_grid_rows);
      vision_core::config_helper::get(fn_, "n_grid_cols", n_grid_cols);
      vision_core::config_helper::get(fn_, "corner_threshold", corner_threshold);
      vision_core::config_helper::get(fn_, "max_pixel_dist", max_pixel_dist);
    }
    RTV_EXPECT_TRUE_(vision_core::config_helper::loadNode(fn_, fs, "INIT"));
    {
      RTV_EXPECT_TRUE_(fn_["stand_still_time"].isNamed());
      vision_core::config_helper::get_if(fn_, "stand_still_time", stand_still_time);
    }
    RTV_EXPECT_TRUE_(vision_core::config_helper::loadNode(fn_, fs, "MSCKF"));
    {

      RTV_EXPECT_TRUE_(fn_["max_gn_cost_norm"].isNamed());
      RTV_EXPECT_TRUE_(fn_["translation_threshold"].isNamed());
      RTV_EXPECT_TRUE_(fn_["min_rcond"].isNamed());
      RTV_EXPECT_TRUE_(fn_["keyframe_transl_dist"].isNamed());
      RTV_EXPECT_TRUE_(fn_["keyframe_rot_dist"].isNamed());
      RTV_EXPECT_TRUE_(fn_["min_track_length"].isNamed());
      RTV_EXPECT_TRUE_(fn_["max_track_length"].isNamed());
      RTV_EXPECT_TRUE_(fn_["max_cam_states"].isNamed());
      RTV_EXPECT_TRUE_(fn_["ransac_threshold"].isNamed());

      vision_core::config_helper::get(fn_, "max_gn_cost_norm", max_gn_cost_norm);
      vision_core::config_helper::get(fn_, "translation_threshold", translation_threshold);
      vision_core::config_helper::get(fn_, "min_rcond", min_rcond);
      vision_core::config_helper::get(fn_, "keyframe_transl_dist", keyframe_transl_dist);
      vision_core::config_helper::get(fn_, "keyframe_rot_dist", keyframe_rot_dist);
      vision_core::config_helper::get(fn_, "min_track_length", min_track_length);
      vision_core::config_helper::get(fn_, "max_track_length", max_track_length);
      vision_core::config_helper::get(fn_, "max_cam_states", max_cam_states);
      vision_core::config_helper::get(fn_, "ransac_threshold", ransac_threshold);

    }
    RTV_EXPECT_TRUE_(vision_core::config_helper::loadNode(fn_, fs, "Camera"));
    {
      RTV_EXPECT_TRUE_(fn_.type() == cv::FileNode::MAP);
      RTV_EXPECT_TRUE_(fn_.size() >= 2);

      cv::Mat CM, DM;
      RTV_EXPECT_TRUE_(fn_["CM"].isNamed());
      RTV_EXPECT_TRUE_(fn_["DM"].isNamed());
      fn_["CM"] >> CM;
      RTV_EXPECT_TRUE_(CM.rows == 4 || (CM.rows == 3 && CM.cols == 3));
      if(CM.rows == 4)
      {
        fx = CM.at<double>(0);
        fy = CM.at<double>(1);
        cx = CM.at<double>(2);
        cy = CM.at<double>(3);

      }
      else if(CM.rows == 3 && CM.cols == 3)
      {
        fx = CM.at<double>(0,0);
        fy = CM.at<double>(1,1);
        cx = CM.at<double>(0,2);
        cy = CM.at<double>(1,2);
      }
      fn_["DM"] >> DM;
      k1 = DM.at<double>(0);
      k2 = DM.at<double>(1);
      p1 = DM.at<double>(2);
      p2 = DM.at<double>(3);
    }
    if(vision_core::config_helper::loadNode(fn_, fs, "Camera-IMU"))
    {
      cv::Mat R_ci, p_c_i;  // reads as: from CAMERA to IMU
      RTV_EXPECT_TRUE_(fn_["R_ci"].isNamed());
      RTV_EXPECT_TRUE_(fn_["p_ci"].isNamed());
      fn_["R_ci"] >> R_ci;
      fn_["p_ci"] >> p_c_i;
      cv::cv2eigen(R_ci, R_CI);
      cv::cv2eigen(p_c_i, p_C_I);
    }

    set_CAMERA_params(fx, fy, cx,cy, k1, k2, p1, p2, distortion_model, R_CI, p_C_I);
    set_NOISE_params(fx, fy, feature_cov, w_var, dbg_var, a_var, dba_var, q_var_init, bg_var_init, v_var_init, ba_var_init, p_var_init);
    set_MSCKF_params(fx, max_gn_cost_norm, translation_threshold, min_rcond, keyframe_transl_dist, keyframe_rot_dist, max_track_length, min_track_length, max_cam_states);
    set_TRACKER_params(n_grid_rows, n_grid_cols, ransac_threshold, corner_threshold, max_pixel_dist);
    set_INITIALIZATON_params(stand_still_time, CalibrationMethod::TimedStandStill);
    return true;
  }

  void RosInterface::set_camera_intrinsics(const float fx, const float fy, const float cx, const float cy)
  {
    K_ = cv::Mat::eye(3,3,CV_32F);
    K_.at<float>(0,0) = fx;
    K_.at<float>(1,1) = fy;
    K_.at<float>(0,2) = cx;
    K_.at<float>(1,2) = cy;

    // setup camera parameters
    camera_.f_u = fx;
    camera_.f_v = fy;
    camera_.c_u = cx;
    camera_.c_v = cy;
  }

  void RosInterface::set_distortion_coef(const float k1, const float k2, const float p1, const float p2)
  {
    dist_coeffs_ = cv::Mat::zeros(4,1,CV_32F);
    dist_coeffs_.at<float>(0) = k1;
    dist_coeffs_.at<float>(1) = k2;
    dist_coeffs_.at<float>(2) = p1;
    dist_coeffs_.at<float>(3) = p2;
  }

  void RosInterface::set_cam_imu_extrinsics(const Matrix3f &R_CI, const Vector3f &p_CI)
  {
    camera_.q_CI = R_CI; // q_CI reads as CAMERA in IMU (from IMU to CAMERA)
    camera_.p_C_I = p_CI;  // p_C_I reads as CAMERA in IMU (from IMU to CAMERA in IMU)
  }

  void RosInterface::set_imu_noise_params(const float w_var, const float dbg_var, const float a_var, const float dba_var)
  {
    Eigen::Matrix<float,12,1> Q_imu_vars;
    Q_imu_vars << w_var, 	w_var, 	w_var,
                  dbg_var,dbg_var,dbg_var,
                  a_var,	a_var,	a_var,
                  dba_var,dba_var,dba_var;

    noise_params_.Q_imu = Q_imu_vars.asDiagonal();

  }

  void RosInterface::set_P(const float q_var_init, const float bg_var_init, const float v_var_init, const float ba_var_init, const float p_var_init)
  {
    Eigen::Matrix<float,15,1> IMUCovar_vars;
    IMUCovar_vars << q_var_init, q_var_init, q_var_init,
                     bg_var_init,bg_var_init,bg_var_init,
                     v_var_init, v_var_init, v_var_init,
                     ba_var_init,ba_var_init,ba_var_init,
                     p_var_init, p_var_init, p_var_init;

    noise_params_.initial_imu_covar = IMUCovar_vars.asDiagonal();
  }

  void RosInterface::set_CAMERA_params(const float fx, const float fy, const float cx, const float cy, const float k1, const float k2, const float p1, const float p2, const std::string &distortion_model, const Matrix3f &R_CI, const Vector3f &p_CI)
  {
    this->set_camera_intrinsics(fx, fy, cx, cy);
    this->set_distortion_coef(k1, k2, p1, p2);
    distortion_model_ = distortion_model;
    this->set_cam_imu_extrinsics(R_CI, p_CI);
  }

  void RosInterface::set_NOISE_params(const float fx,
                                      const float fy,
                                      const float feature_covariance,
                                      const float w_var,
                                      const float dbg_var,
                                      const float a_var,
                                      const float dba_var,
                                      const float q_var_init,
                                      const float bg_var_init,
                                      const float v_var_init,
                                      const float ba_var_init,
                                      const float p_var_init)
  {
    noise_params_.u_var_prime = pow(feature_covariance/fx,2);
    noise_params_.v_var_prime = pow(feature_covariance/fy,2);
    this->set_imu_noise_params(w_var, dbg_var, a_var, dba_var);
    this->set_P(q_var_init, bg_var_init, v_var_init, ba_var_init, p_var_init);
  }

  void RosInterface::set_MSCKF_params(const float fx,
                                      const float max_gn_cost_norm,
                                      const float translation_threshold,
                                      const float min_rcond,
                                      const float keyframe_transl_dist,
                                      const float keyframe_rot_dist,
                                      const float max_track_length,
                                      const float min_track_length,
                                      const float max_cam_states)
  {



    msckf_params_.max_gn_cost_norm = max_gn_cost_norm;
    msckf_params_.max_gn_cost_norm = pow(msckf_params_.max_gn_cost_norm/fx, 2);
    msckf_params_.translation_threshold = translation_threshold;
    msckf_params_.min_rcond = min_rcond;
    msckf_params_.redundancy_angle_thresh = keyframe_transl_dist;
    msckf_params_.redundancy_distance_thresh = keyframe_rot_dist;
    msckf_params_.max_track_length = max_track_length;
    msckf_params_.min_track_length = min_track_length;
    msckf_params_.max_cam_states = max_cam_states;
  }

  void RosInterface::set_TRACKER_params(const float n_grid_rows, const float n_grid_cols, const float ransac_threshold, const float corner_treshold, const float max_pixel_dist)
  {
    n_grid_rows_ = n_grid_rows;
    n_grid_cols_ = n_grid_cols;
    ransac_threshold_ = ransac_threshold;
    corner_treshold_ = corner_treshold;
    tracker_max_pixel_dist_ = max_pixel_dist;
  }

  void RosInterface::set_INITIALIZATON_params(const float stand_still_time, const RosInterface::CalibrationMethod method)
  {
    imu_calibration_method_ = method;
    stand_still_time_ = stand_still_time;
  }

  void RosInterface::dump_info()
  {
    ROS_INFO_STREAM("-  Intrinsics: " << camera_.f_u << ", "
                                   << camera_.f_v << ", "
                                   << camera_.c_u << ", "
                                   << camera_.c_v );
    ROS_INFO_STREAM("-  Distortion: " << dist_coeffs_.at<float>(0) << ", "
                                   << dist_coeffs_.at<float>(1) << ", "
                                   << dist_coeffs_.at<float>(2) << ", "
                                   << dist_coeffs_.at<float>(3) );

    ROS_INFO_STREAM("-  q_CI: " << camera_.q_CI.x() << "," << camera_.q_CI.y() << "," << camera_.q_CI.z() << "," << camera_.q_CI.w() << " (xyzw)");
    ROS_INFO_STREAM("-  p_C_I: " << camera_.p_C_I.transpose());
    ROS_INFO_STREAM("-  stand still duration: " << stand_still_time_);
    ROS_INFO_STREAM("-  Tracker config: " << n_grid_cols_ << "," << n_grid_rows_ << "," << ransac_threshold_ << "," <<  corner_treshold_);
  }

}
