% this program match reflectors with calculated angles abd return the matched reflector x/y and reflector ID 
function [matched_reflect_ID,matched_reflect_angle_ID,matched_detect_ID,matched_detect_angle_ID,result] = match_angle_reflector(Reflect_angle_vector,Reflect_angle_ID,detect_Ref_angle_vector,detect_angle_ID,thres_angle_match)
% Define matching threshold value here
%-- match the angle matrix with reflector tables
matched_reflect_angle_ID=0;
matched_detect_angle_ID=0;
matched_reflect_ID=0;
matched_detect_ID=0;
m=0;n=0;
match_angle_flag(1:length(detect_Ref_angle_vector))=0;
match_angle_ref_flag(1:length(Reflect_angle_vector),1:length(detect_Ref_angle_vector))=0;
% -- Need to filter the distance larger than certain threshold
for j=1:length(Reflect_angle_vector)   % Reference reflector
    for i=1:length(detect_Ref_angle_vector)  % detetced reflector
        diff_ref_detect(j,i) = abs(Reflect_angle_vector(1,j)-detect_Ref_angle_vector(1,i));
    end
end
diff_ref_detect;
for i=1:length(detect_Ref_angle_vector)  % detetced reflector
    [min_det(i),idx_ref(i)]=min(diff_ref_detect(:,i));
    if (match_angle_ref_flag(idx_ref(i),i) == 0) && (diff_ref_detect(idx_ref(i),i)<thres_angle_match)
                match_angle_ref_flag(idx_ref(i),:) = 1;
                m=m+1;
                n=n+1;
                matched_reflect_angle_ID(m,1)=Reflect_angle_ID(idx_ref(i),1);  %
                matched_reflect_angle_ID(m,2)=Reflect_angle_ID(idx_ref(i),2);
                matched_reflect_angle_ID(m,3)=Reflect_angle_ID(idx_ref(i),3);
                matched_detect_angle_ID(n,1)=detect_angle_ID(i,1);  %
                matched_detect_angle_ID(n,2)=detect_angle_ID(i,2);
                matched_detect_angle_ID(n,3)=detect_angle_ID(i,3);
                diff_ref_detect(idx_ref(i),i)=inf;
    elseif match_angle_ref_flag(idx_ref(i),i) == 1  && (diff_ref_detect(idx_ref(i),i)<thres_angle_match)
        diff_ref_detect(idx_ref(i),:)=[];
        [min_det(i),idx_ref(i)]=min(diff_ref_detect(:,i));
            if (match_angle_ref_flag(idx_ref(i),i) == 0) && (diff_ref_detect(idx_ref(i),i)<thres_angle_match)
                match_angle_ref_flag(idx_ref(i),:) = 1;
                m=m+1;
                n=n+1;
                matched_reflect_angle_ID(m,1)=Reflect_angle_ID(idx_ref(i),1);  %
                matched_reflect_angle_ID(m,2)=Reflect_angle_ID(idx_ref(i),2);
                matched_reflect_angle_ID(m,3)=Reflect_angle_ID(idx_ref(i),3);
                matched_detect_angle_ID(n,1)=detect_angle_ID(i,1);  %
                matched_detect_angle_ID(n,2)=detect_angle_ID(i,2);
                matched_detect_angle_ID(n,3)=detect_angle_ID(i,3);
            end
    end
end
matched_reflect_angle_ID;
match_angle_ref_flag;
m;
diff_ref_detect;
if  m>=3
     matched_reflect_ID=unique(matched_reflect_angle_ID(:,2),'stable')';
     matched_detect_ID=unique(matched_detect_angle_ID(:,2),'stable')';
    if length(matched_reflect_ID)~= length(matched_detect_ID)
        % replace the wrong ID with the next near-to-min point 
        [matched_reflect_ID,matched_detect_ID]=replace_angle_matching_point(diff_ref_detect,Reflect_angle_ID,matched_reflect_angle_ID,matched_detect_angle_ID);
        if length(matched_reflect_ID)~= length(matched_detect_ID)
           result=1;
        else
            result=0;
        end
    elseif m>length(detect_angle_ID)
        result=1;
        disp('matched reflector exceeds the number of detected reflectors');
    else
        disp('matched ref reflectors: ');
        disp(sprintf('Reflector ID(angle):-%i ', matched_reflect_ID));
        disp('matched detected reflectors: ');
        disp(sprintf('Reflector ID(angle):-%i ', matched_detect_ID));
        result=0;
    end
elseif m<=2   %length(detect_Ref_angle_vector)
    disp('Not enough matched reflectors found');
    result=1;
    return
end
