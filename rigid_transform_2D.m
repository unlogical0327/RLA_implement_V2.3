% This function finds the optimal Rigid/Euclidean transform in 2D space
% It expects as input a Nx2 matrix of 2D points.
% It returns R, T
% expects row data
function [R,t] = rigid_transform_2D(A, B)
if nargin ~= 2
    error('Missing parameters');
end
A;
B;
centroid_A = mean(A);  %
centroid_B = mean(B);
N=size(A,1);
H=(A-repmat(centroid_A,N,1))'*(B-repmat(centroid_B,N,1));
% H = zeros(2,2);
% for i=1:size(A,1)
% H = H+(A(i,:)-centroid_A)'*(B(i,:)-centroid_B);
% % NOTE: the transpose is on A, different to my tutorial due to convention used
% end
[U,S,V] = svd(H);
R = V*U';
% R(1,2);
% if R(1,2) < 0
%    disp('90 degree flip detected\n');
%    % FP= [0 -1;1 0];
%     FP= [0 1;-1 0];
%     R = R*FP;
% end

if det(R) < 0
    disp('Reflection detected\n');
    V(:,2)= -V(:,2);
    %V(:,2)= -1;
    R = V*U';
end

t = -R*centroid_A' + centroid_B';
end
