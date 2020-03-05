function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
H = [];
A = [];
a_x = [-video_pts(:,1) -video_pts(:,2) -1*ones(4,1) zeros(4,1) zeros(4,1) zeros(4,1) video_pts(:,1).*logo_pts(:,1) video_pts(:,2).*logo_pts(:,1) logo_pts(:,1)];
a_y = [zeros(4,1) zeros(4,1) zeros(4,1) -video_pts(:,1) -video_pts(:,2) -1*ones(4,1) video_pts(:,1).*logo_pts(:,2) video_pts(:,2).*logo_pts(:,2) logo_pts(:,2)];
A = [a_x(1,:); a_y(1,:); a_x(2,:); a_y(2,:); a_x(3,:); a_y(3,:); a_x(4,:); a_y(4,:)];
[U,S,V] = svd(A);
V = V(:,end);
H = reshape(V,[3,3])';

end

