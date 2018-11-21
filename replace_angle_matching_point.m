% this program replace the nearest min in matching points 
%%%%%%%%%%%%%%%%%%%%%%%%%%
function [matched_reflect_ID,matched_detect_ID]=replace_angle_matching_point(diff_ref_detect,Reflect_angle_ID,matched_reflect_angle_ID,matched_detect_angle_ID)
         min_val=min(min(diff_ref_detect));  % find next min value to match the reflectors and update 
         [idx,idy]=find(diff_ref_detect == min_val)
         matched_reflect_angle_ID(idy,1)=Reflect_angle_ID(idx,1);  %
         matched_reflect_angle_ID(idy,2)=Reflect_angle_ID(idx,2);
         matched_reflect_angle_ID(idy,3)=Reflect_angle_ID(idx,3);
         matched_reflect_ID=unique(matched_reflect_angle_ID(:,2),'stable')'
         matched_detect_ID=unique(matched_detect_angle_ID(:,2),'stable')'