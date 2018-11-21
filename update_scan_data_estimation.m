% this program use the prior information to estimate the errors generated from moving and update the
% lidar scan data to reduce the errors.
function [Lidar_expect_Table,Lidar_expect_xy]=update_Lidar_scan_xy(ret_R,ret_T,measurement_data,scan_data,Lidar_x,Lidar_y)

[l,w] = size(pose_hist);
%theta_delta = 
t=1/20;
pose_his
x_offset1 = pose_hist(l,1)-pose_hist(l-1,1);
y_offset1 = pose_hist(l,2)-pose_hist(l-1,2);
angle_offset1 = pose_hist(l,3)-pose_hist(l-1,3);
x_offset2 = pose_hist(l-1,1)-pose_hist(l-2,1);
y_offset2 = pose_hist(l-1,2)-pose_hist(l-2,2);
angle_offset2 = pose_hist(l-1,3)-pose_hist(l-2,3);
x_vel=(x_offset1-x_offset2)/t;  % x velocity
y_vel=(y_offset1-y_offset2)/t;  % y velocity
pose_expect(1,3) = pose_hist(l,3)+angle_offset1; % assume the angle is not changing during the moving
pose_expect(1,1) = pose_hist(l,1)+x_offset1;
pose_expect(1,2) = pose_hist(l,2)+y_offset1;