%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% created: March 2020
% version: 1.0.0
% authors: 
% * Roland Jung (roland.jung@aau.at)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function profile_msckf_demo_ros_evaluation()
  close all
  
  % profiling results of: msckf_mono/profiling/profile_msckf_demo_ros.sh
  
  feature_size_bytes = 12;
  x_feat = [8*6, 12*9, 16*12]';
  x_res = [320*240, 640*480, 1280*960]';
  y_msckf_marginalize = [8960451, 18355433, 31793300]'; 
  y_msckf_augment = [3759840, 3761001, 3763214]';
  y_msckf_prune_empty = [121544, 121476, 121301]';
  y_msckf_prunce_redunant = [5710, 7229, 9610]';
  y_msckf_propagate = [146618, 146623, 146615]'; % needs to be scaled by 10 due to the IMU/CAM rate!
  y_msckf_update = [137089, 425671, 1007316]';
  y_tracker_trackfeat = [2827448, 8914060, 29230102]';  
  y_tracker_newfeat = [2790823, 17108755, 83104246 ]';
  y_tacker_setimg = [4776, 14463, 36600]'; 
  y_cv_copy_img = [22680, 53221, 148737]';
  
  
  
  y_msckf = y_msckf_marginalize + y_msckf_augment + y_msckf_prune_empty + y_msckf_prunce_redunant +  y_msckf_update
  y_tracker = y_tracker_trackfeat + y_tracker_newfeat + y_tacker_setimg + y_cv_copy_img
  
  y_img = y_msckf + y_tracker
  
  %% EXTRAPOLATE: features of resolution
  figure('Name', 'features over resolution');
  f_feat = fit(x_res,x_feat, 'linearinterp');
  new_res =  1280*960*4
  new_feat = 320; % too few datapoint for: f_feat(new_res)
  plot(f_feat,x_res,x_feat); grid on;
  hold on;
  stem([x_res; new_res], [x_feat; new_feat], 'g')
  xlabel('num pixels');
  ylabel('num features');
  legend('Location','northwest');
  
  %% BACK-END
  figure('Name', 'MSCKF - back-end')
  subplot(2,1,1);
  f_msckf = fit(x_feat,y_msckf,'poly2');
  new_backend = f_msckf(new_feat)
  plot(f_msckf,x_feat,y_msckf); grid on;
  hold on;
  stem([x_feat; new_feat], [y_msckf; new_backend], 'g')
  xlabel('');
  ylabel('cycle count');
  legend('Location','northwest');
  
  subplot(2,1,2);
  f_msckf = fit(x_res,y_msckf,'poly2');
  plot(f_msckf,x_res,y_msckf); grid on;
  xlabel('num pixels');
  ylabel('cycle count');
  legend('Location','northwest');
  
  %% FRONT-END
  figure('Name', 'Tracker - front-end');
  subplot(2,1,1);
  f_tracker = fit(x_feat,y_tracker,'poly2');
  plot(f_tracker,x_feat,y_tracker); grid on;
  xlabel('num features');
  ylabel('cycle count')
  legend('Location','northwest');
  
  subplot(2,1,2);
  f_tracker = fit(x_res,y_tracker,'poly2');
  new_frontend = f_tracker(new_res)
  plot(f_tracker,x_res,y_tracker); grid on;
  hold on;
  stem([x_res; new_res], [y_tracker; new_frontend], 'g')
  xlabel('num pixels');
  ylabel('cycle count')
  legend('Location','northwest');
  
  figure('Name', 'front- and back-end');
  subplot(2,1,1);
  f = fit(x_feat,y_img,'poly2');
  plot(f,x_feat,y_img); grid on;
  xlabel('num features');
  ylabel('cycle count')
  legend('Location','northwest');
  
  subplot(2,1,2);
  f = fit(x_res,y_img,'poly2');
  plot(f,x_res,y_img); grid on;
  xlabel('num pixels');
  ylabel('cycle count');
  legend('Location','northwest');
  
  new_total = new_frontend + new_backend
  
end
