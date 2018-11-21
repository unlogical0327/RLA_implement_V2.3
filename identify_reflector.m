%% Screen the data point based on the reflection and the adjacent point
% continuity. Identify the reflector from background and check if the reflector is identical.
function [detected_ID,detected_reflector,detected_reflector_polar,reflector_index]=identify_reflector(amp_thres,dist_thres,reflector_diameter,distance_delta,Lidar_Table,Lidar_data)
detected_ID=0;
iii=0;
%amp=2096;    % max amplitude from calibration
%reflector_diameter=80;  % reflector diameters
amp_thres;
detected_reflector=0;
size(Lidar_data)
for ii=2:length(Lidar_data)-1
    if Lidar_data(3,ii)>=amp_thres && Lidar_data(2,ii)>=dist_thres
            if Lidar_data(3,ii)>Lidar_data(3,ii-1) && Lidar_data(3,ii)>Lidar_data(3,ii+1)
                iii=iii+1;
                detected_ID_total(iii)=iii;
                detected_reflector_x(detected_ID_total(iii))=Lidar_Table(ii,1);
                detected_reflector_y(detected_ID_total(iii))=Lidar_Table(ii,2);
                detected_reflector_angle(detected_ID_total(iii))=Lidar_data(1,ii);
                detected_reflector_dist(detected_ID_total(iii))=Lidar_data(2,ii);
                raw_reflector_index(iii)=ii;
                disp('Raw detect reflector!!!');
                disp(sprintf('Reflector ID: %i', detected_ID_total(iii)));
            end
    end
end

if (iii<=1)  % in case no reflector detected
    detected_reflector=0;
    etected_reflector_polar=0;
    disp('No reflector Detected!!!');
else
%%%%--- merge all peaks from the same refletor
   %-- initilze array 1st cell
   l=1;iii=1;
   reflector_index(1)=int8(raw_reflector_index(1));
   detected_ID(1)=1;
   detected_reflector(1,1) = detected_reflector_x(1);
   detected_reflector(1,2) = detected_reflector_y(1);
   detected_reflector_polar(1,1) = detected_reflector_angle(1);
   detected_reflector_polar(1,2) = detected_reflector_dist(1);
   detected_reflector_angle_g(1) = detected_reflector_angle(1);

   for ii=(l+1):length(detected_reflector_angle)
               angle_delta(ii)=reflector_diameter/Lidar_data(2,ii)/pi*180;
               if (abs(detected_reflector_dist(ii)-detected_reflector_dist(l))<distance_delta) && (abs(detected_reflector_angle(ii)-detected_reflector_angle(l))<angle_delta(ii))  
             % check if the detected point is from the same reflector
                reflector_index(iii)=int8((raw_reflector_index(ii)+reflector_index(iii))/2);
                detected_ID(iii)=iii;
                detected_reflector(detected_ID(iii),1) = (detected_reflector_x(ii)+detected_reflector(detected_ID(iii),1))/2;
                detected_reflector(detected_ID(iii),2) = (detected_reflector_y(ii)+detected_reflector(detected_ID(iii),2))/2;
                detected_reflector_angle_g(detected_ID(iii)) = (detected_reflector_angle(ii)+detected_reflector_angle_g(detected_ID(iii)))/2;
                detected_reflector_polar(detected_ID(iii),1) = (detected_reflector_angle(ii)+detected_reflector_polar(detected_ID(iii),1))/2;
                detected_reflector_polar(detected_ID(iii),2) = (detected_reflector_dist(ii)+detected_reflector_polar(detected_ID(iii),2))/2;               
            else %(abs(Lidar_data(1,ii)-Lidar_data(1,ii-1))<angle_delta)
                iii=iii+1;
                reflector_index(iii)=int8(raw_reflector_index(ii));
                detected_ID(iii)=iii;
                detected_reflector(detected_ID(iii),1) = detected_reflector_x(ii);
                detected_reflector(detected_ID(iii),2) = detected_reflector_y(ii);
                detected_reflector_angle_g(detected_ID(iii)) = detected_reflector_angle(ii);
                detected_reflector_polar(detected_ID(iii),1) = detected_reflector_angle(ii);
                detected_reflector_polar(detected_ID(iii),2) = detected_reflector_dist(ii);
                disp('Detect reflector!!!');
                disp(sprintf('Reflector ID: %i', detected_ID(iii)));
               end
               l=l+1;
   end
   ll=length(detected_reflector_angle_g)
if detected_reflector_angle_g(1)-detected_reflector_angle_g(ll)<(angle_delta(2)-360) || detected_reflector_angle_g(1)-detected_reflector_angle_g(ll)<angle_delta(2) 
    if (abs(detected_reflector_dist(1)-detected_reflector_dist(end))<distance_delta)
   detected_reflector(1,1) = (detected_reflector(ll,1)+detected_reflector(1,1))/2;
   detected_reflector(1,2) = (detected_reflector(ll,2)+detected_reflector(1,2))/2;
   detected_reflector(ll,:) = [];
   detected_ID(ll) = [];
   detected_reflector_angle_g(ll) = []
    end
end
size(detected_reflector)
% figure(104);plot(detected_reflector(:,1),detected_reflector(:,2),'+')
% detected_reflector
end
if (length(detected_ID)<=1 && detected_ID(1)==0)  % in case no reflector detected
    detected_reflector=0;
    detected_reflector_polar=0;
    disp('No reflector Detected!!!');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-- Below code tests the algorithm to find and merge conditional detected reflectors 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% aa=[   1525
%     1524
%     1523
%     1520
%         1487
%         1484
%         1480
%         1407
%         1314
%         1303
%         12990
%         12380];
%     l=1;
%     k=1;
%     zz(1)=aa(1);
% for i=(l+1):length(aa)
%     if abs(aa(l)-aa(i))<10
%     zz(k)=(aa(l)+aa(i))/2;
%     else 
%         k=k+1;
%         zz(k)=aa(i);
%     end
%     l=l+1;
% end