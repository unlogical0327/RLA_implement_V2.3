% this program match reflectors with calculated angles abd return the matched reflector x/y and reflector ID 
function [matched_reflect_ID,matched_reflect_angle_ID,matched_detect_ID,matched_detect_angle_ID,result] = match_angle_reflector(Reflect_angle_vector,Reflect_angle_ID,detect_Ref_angle_vector,detected_angle_ID,thres_angle_match)
% Define matching threshold value here
%-- match the angle matrix with reflector tables
matched_reflect_angle_ID=0;
matched_detect_angle_ID=0;
matched_reflect_ID=0;
matched_detect_ID=0;
m=0;n=0;
%match_angle_flag(1:length(Reflect_angle_vector))=0;
match_angle_flag(1:length(detect_Ref_angle_vector))=0;
min_angle=thres_angle_match;
% -- Need to filter the distance larger than certain threshold
for j=1:length(Reflect_angle_vector)   % Reference reflector
    for i=1:length(detect_Ref_angle_vector)  % detetced reflector
        %if (Reflect_angle_vector(1,j)<thres_angle_large && detect_Ref_angle_vector(1,i)<thres_angle_large)
        if abs(Reflect_angle_vector(1,j)-detect_Ref_angle_vector(1,i))<min_angle
           if (match_angle_flag(i)==0) %&& (match_angle_ref_flag==0)
             for ll=1:m
                if (Reflect_angle_ID(j,2)~=matched_reflect_angle_ID(ll,2)) % exclude the repeated points
                %min_angle=abs(Reflect_angle_vector(1,j)-detect_Ref_angle_vector(1,i));
                m=m+1;
                n=n+1;
                matched_reflect_angle_ID(m,1)=Reflect_angle_ID(j,1);  %
                matched_reflect_angle_ID(m,2)=Reflect_angle_ID(j,2);
                matched_reflect_angle_ID(m,3)=Reflect_angle_ID(j,3);
                %matched_detect_angle_ID(n,1)=detected_angle_ID(i,1);
                %matched_detect_angle_ID(n,2)=detected_angle_ID(i,2);
                matched_detect_angle_ID(n,1)=Reflect_angle_ID(j,1);
                matched_detect_angle_ID(n,2)=Reflect_angle_ID(j,2);
                matched_detect_angle_ID(n,3)=Reflect_angle_ID(j,3);
                match_angle_flag(i)=Reflect_angle_ID(j,2);
                match_angle_ref_flag(j)=Reflect_angle_ID(j,2);
                disp(sprintf('Matched reflector_angle ID: %i', j));
                disp(sprintf('Matched detect_angle ID: %i', i));
                end
             end
           end
        end
    end
end
match_angle_flag
%match_angle_ref_flag

if  m>=3
    matched_reflect_ID=unique(matched_reflect_angle_ID(:,2),'stable')';
    matched_detect_ID=unique(matched_detect_angle_ID(:,2),'stable')';
    if length(matched_reflect_ID)~= length(matched_detect_ID)
        result=1;
    elseif m>length(detected_angle_ID)
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
