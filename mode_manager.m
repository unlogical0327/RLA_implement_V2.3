%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RLA flow design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function is the top level code to implement matlab-to-C++
% verification platform
% RLA has options to generate test vectors to verify the algorithm.
% This program is developed and copyright owned by Soleilware LLC
% The code is writen to build the blocks for the localization
% algorithm process and efficiency.
% --------------------------------
% Created by Qi Song on 9/18/2018
%function [status]=RLA_toplevel(list_source_flag)% RLA top level function to convert Matlab code to C++ package and run C++ test code
function [mode,status,update_match_pool] = mode_manager(interrupt,scan_freq,reflector_source_flag,req_update_match_pool,num_ref_pool,num_detect_pool,scan_data,amp_thres,dist_thres,reflector_diameter,dist_delta,thres_dist_match,thres_dist_large,thres_angle_match)
%% -interrupt:              interrupt from GUI console to control the Lidar computing engine
%% -reflector_source_flag:  flag to define the reflector source from GUI
%% -data_source_flag:       flag to define the data source from GUI
%% -req_update_match_pool:  request to ask match pool to update to include more reflectors
%% -Reflector_map:          load Reflector map from GUI console
%% -scan_data:              load 3D Lidar data to module
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
data_source_flag=1;
moving_mode_simu=1;
scan_flag=1;
%% Load Reflector map
if scan_flag==0
    [Reflector_map, Reflector_ID, load_ref_map_status] = load_reflector_map();
 elseif scan_flag==1 && moving_mode_simu==0
    %%-- Read scan data as the reference reflector map
    fname_2 = ['Lidar_data_example2'];
    scan_data= dlmread(fname_2, '\t', 3, 0)';
    read_file=0;
    mode='cali';
    [Reflector_map,Reflector_map_polar, Reflector_ID, load_ref_map_status]=reflector_map_cali_scan(amp_thres,dist_thres,reflector_diameter,dist_delta,scan_data,data_source_flag,read_file);
    %%-- Only for test data
    fname_2= ['Lidar_data_example2'];
    Lidar_data = dlmread(fname_2, '\t', 3, 0)';
    size(Lidar_data)
    size(scan_data)
elseif scan_flag==1 && moving_mode_simu==1
    fname_moving = ['Data/1/Lidar_data.txt']; % Load moving data to test moving compensation algorithm
    mode='cali';
    [Lidar_data,data_length,data_round]=load_continous_scan_data(fname_moving,mode);
    scan_data=Lidar_data;
    size(Lidar_data)
    read_file=0;
    [Reflector_map, Reflector_map_polar, Reflector_ID, load_ref_map_status]=reflector_map_cali_scan(amp_thres,dist_thres,reflector_diameter,dist_delta,scan_data,data_source_flag,read_file);
Reflector_map;
Reflector_map_polar;
end
b=1;
tic;
while(b==1)
    tstart_cali=tic;
    %% convert polar data to rectangle data
    [calibration_data,scan_data]=PolarToRect(Reflector_map,Lidar_data,data_source_flag);
    %%-- Run calibration mode
    [cali_status,Lidar_trace,Lidar_trace_polar,rotation_trace] = calibration_mode(amp_thres,dist_thres,reflector_diameter,dist_delta,Reflector_map,Reflector_map_polar,Reflector_ID,calibration_data,scan_data,thres_dist_match,thres_dist_large,thres_angle_match);
    
    if cali_status==0
        disp('Calibration successful! Proceed to measurement mode....')
        break
    elseif cali_status==3
        disp('Data is bad, wait for new Lidar data for new Cali!!')
    else
        disp('Calibration failed, please check Lidar data!!')
        break
    end
end
tlapsed_cali=toc(tstart_cali);
%% Measurement mode
%-- need to read the scan data and process the data at each scan
%measurement
Lidar_trace_p=0;
rotation_trace_p=0;
Lidar_update_Table_p=0;
detected_ID_p=0;
detected_reflector_p=0;
match_reflect_pool_p=0;
match_reflect_ID_p=0;
Loop_num=scan_freq;
c=1;
%-- read the meas data
 mode='meas';
 if moving_mode_simu==1
[Lidar_data,data_length,data_round]=load_continous_scan_data(fname_moving,mode);
scan_data=Lidar_data;
 end
 % start the stopwatch
 tic;
for ll=1:Loop_num     % simulation loop start from here!!!
    while c==1
        %% scan data is 2D data
        %% measurement_data only need angle and distance;
        % -- Could be replace by 2D scan data directly
        tstart=tic;
        size(Lidar_data);
        [measurement_data3,scan_data]=PolarToRect(Reflector_map,Lidar_data,data_source_flag);
        
        %%-- Plot raw data
        plot_Lidar_data(measurement_data3)
        %%-- Run measurement mode to find Robot location in the world coordinate
        %[mea_status,Lidar_trace,rotation_trace,Lidar_update_Table,match_reflect_pool,match_reflect_ID,reflector_index,detected_reflector,detected_ID] = measurement_mode(num_ref_pool,num_detect_pool,Reflector_map,Reflector_ID,measurement_data3,scan_data,amp_thres,dist_thres,reflector_diameter,dist_delta,Lidar_trace,rotation_trace,thres_dist_match,thres_dist_large,thres_angle_match);
       
        %%-- Run measurement in polar system
        [mea_status,Lidar_trace,rotation_trace,Lidar_update_Table,match_reflect_pool,match_reflect_ID,reflector_index,detected_reflector,detected_ID] = measurement_mode(num_ref_pool,num_detect_pool,Reflector_map,Reflector_map_polar,Reflector_ID,measurement_data3,scan_data,amp_thres,dist_thres,reflector_diameter,dist_delta,Lidar_trace,rotation_trace,thres_dist_match,thres_dist_large,thres_angle_match);
        mea_status
        %%
        if mea_status==3
            Lidar_trace=Lidar_trace_p;
            rotation_trace=rotation_trace_p;
            Lidar_update_Table=Lidar_update_Table_p;
            detected_ID=detected_ID_p;
            detected_reflector=detected_reflector_p;
            match_reflect_pool=match_reflect_pool_p;
            match_reflect_ID=match_reflect_ID_p;
        else
            Lidar_trace_p=Lidar_trace;
            rotation_trace_p=rotation_trace;
            Lidar_update_Table_p=Lidar_update_Table;
            detected_ID_p=detected_ID;
            detected_reflector_p=detected_reflector;
            match_reflect_pool_p=match_reflect_pool;
            match_reflect_ID_p=match_reflect_ID;
        end
        if mea_status==0
            disp('Measurement successful! continuing.....')
            status='good';
            break
        elseif mea_status==1
            disp('Measurement error found! Please check Lidar data!!')
            status='minor error';
            break
        elseif mea_status==2
            disp('Measurement large error found! Please stop test and check Lidar data!!')
            status='major error';
            break
        elseif mea_status==3
            disp('Measurement failed!')
            status='broken';
        else
            disp('Measurement Status 4')
            break
        end
    end
    %% --Plot final result in the world coordinate
    Plot_world_map(Lidar_update_Table,match_reflect_pool,match_reflect_ID,detected_reflector,detected_ID,Lidar_trace)
    Plot_world_map(Lidar_update_Table,match_reflect_pool,match_reflect_ID,detected_reflector,detected_ID,Lidar_trace)
    %pause(1)
    tlapse_mea(ll)=toc(tstart)
end
update_match_pool='true';
%% Moving mode
%--Use moving mode to calculate the AGV in moving. 
% This mode will use moving estimation to reduce the errors generated
% during AGV is moving at the fast speed.
%-- read the rest of scan data
    mode='movi';
    moving_thres=200;   
    rot_angle_thres=0.25;
    if moving_mode_simu==1
    [Lidar_data_total,data_length,data_round]=load_continous_scan_data(fname_moving,mode);
    end
    minTime=Inf;
    tic;
 for ii=3:122  % 87 is the bad point?
 %for ii=3:150
 tstart=tic;
    pose_hist=[Lidar_trace rotation_trace];
    %--Call moving mode to calculate expected pose and location
    Lidar_data = Lidar_data_total(:,(ii-1)*data_length:ii*data_length);
   [measurement_data4,scan_data]=PolarToRect(Reflector_map,Lidar_data,data_source_flag);
   %%-- Plot raw data
   %plot_Lidar_data(measurement_data4)
   %[moving_status,Lidar_trace,rotation_trace] = moving_mode(moving_thres,rot_angle_thres,amp_thres,reflector_diameter,dist_delta,Reflector_map,Reflector_ID,measurement_data4,scan_data,match_reflect_pool,match_reflect_ID,reflector_index,pose_his,thres_dist_match,thres_dist_large)
   [mea_status,Lidar_trace,rotation_trace,Lidar_update_Table,match_reflect_pool,match_reflect_ID,reflector_index,detected_reflector,detected_ID] = measurement_mode(num_ref_pool,num_detect_pool,Reflector_map,Reflector_map_polar,Reflector_ID,measurement_data4,scan_data,amp_thres,dist_thres,reflector_diameter,dist_delta,Lidar_trace,rotation_trace,thres_dist_match,thres_dist_large,thres_angle_match);
   match_reflect_pool;
   match_reflect_ID;                                                             
   detected_reflector;
   detected_ID;
   ii
   tlapsed_m(ii)=toc(tstart);
   minTime=min(tlapsed_m(ii),minTime);
      if mea_status==3
       disp('Cant find any matched reflector, go to the another round data......')
      else%if mod(ii,20)==0
       Plot_world_map(Lidar_update_Table,match_reflect_pool,match_reflect_ID,detected_reflector,detected_ID,Lidar_trace)
   end
 end
 figure(110)
 subplot(1,3,1);plot(tlapsed_m)
 title('measurement mode loop time')
 subplot(1,3,2);plot(tlapse_mea)
 title('measrement single time')
 subplot(1,3,3);plot(tlapsed_cali)
 title('calibration time')
 tlapsed_cali
 minTime
 tlapsed_m
 pose_hist
 status='done';
 
 