% this program handle the moving mode of AGV with a kine
function [moving_status,Lidar_trace,rotation_trace] = moving_mode(moving_thres,rot_angle_thres,amp_thres,reflector_diameter,distance_delta,Reflector_map,Reflector_ID,calibration_data,scan_data,match_reflect_pool,match_reflect_ID,reflector_index,pose_hist,thres_dist_match,thres_dist_large)

Lidar_trace=pose_hist(:,1:2);
rotation_trace=pose_hist(:,3);

last_reflector(1,1)=match_reflect_pool(match_reflect_ID(1,end),1); % load the last reflector x
last_reflector(1,2)=match_reflect_pool(match_reflect_ID(1,end),2); % load the last reflector y
last_reflector(1,3)=scan_data(3,reflector_index(1,end));
[pose_expect]=pose_expectation(pose_hist);
x_increment=pose_expect(1,1)-pose_hist(end,1);  % x increment to calculate new location of last reflector
y_increment=pose_expect(1,2)-pose_hist(end,2);
angle_increment=pose_expect(1,3)-pose_hist(end,3);
expect_reflector(1,1)=last_reflector(1,1)-x_increment; % calculate the last reflector x and y and compare with measured last reflector
expect_reflector(1,2)=last_reflector(1,2)-y_increment;
expect_reflector(1,3)=last_reflector(1,3)-angle_increment;
% find the last reflector in scan and calculate 
[detected_ID,detected_reflector,reflector_index]=identify_reflector(amp_thres,reflector_diameter,distance_delta,calibration_data,scan_data);
zz=length(detected_ID);
b=1; % flag to search for match reflector
%-- search for matched reflector from the last one
while b==1
    detected_ID(zz);
last_scan_reflector=detected_reflector(detected_ID(1,zz),:);
size(last_scan_reflector);
size(detected_reflector);
size(detected_ID);
size(scan_data);
x_delta=expect_reflector(1,1)-last_scan_reflector(1,1);
y_delta=expect_reflector(1,2)-last_scan_reflector(1,2);
zz
reflector_index(1,zz)
reflector_index
angle_delta=expect_reflector(1,3)-scan_data(3,reflector_index(1,zz));
if x_delta<moving_thres && y_delta<moving_thres && angle_delta<rot_angle_thres
    moving_status=0
    disp('last reflector MATCH with expected position!')
    Lidar_update_xy=[last_scan_reflector(1,1) last_scan_reflector(1,2)];
    Lidar_trace=[Lidar_trace;Lidar_update_xy];
    rotation_trace=[rotation_trace;rotation_trace(end,1)+angle_increment];
    break;
else
    moving_status=1;
    disp('last reflector NOT match with expected position!')
    zz=zz-1;
end
if moving_status==1 && zz==0
    break
end
end