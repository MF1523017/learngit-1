function [ output_args ] = showPoints4d( XYZcam )
%SHOWPOINTS4D Summary of this function goes here
%   Detailed explanation goes here
xyz_ = reshape(XYZcam(:,:,1:3),[480*640,3]);
index_zero = find(xyz_(:,3)==0);
xyz_(index_zero,: ) =[]; 
size_points = numel(xyz_)/3;
showPointCloud(xyz_,repmat([1 0 0], [size_points,1]));
end

