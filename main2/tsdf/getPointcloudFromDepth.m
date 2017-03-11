function [ currentdepth_pts ] = getPointcloudFromDepth( cameraIntrinsicParam,depth,depthinfo )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function: Get point cloud from depth
%   Method:   As the depth image comes from pin-hole camera, we first
%            transform it to CMAERA coordinate then to world cordinate.
%   Input:    cameraIntrinsicParam:      Camera intrinsic matrix
%             depth:          Orthographical depth image
%                             voxel
%             depthinfo:      Depth info relating to "depth"
%   Returns:  currentdepth_pts:
%   Author:   Jingwei Song.   23/08/2016 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

addpath(genpath('pinholeModel'));
[num_imagerow,num_imagecolunm] = size(depth);
currentdepth_pts = depth2XYZcamera(cameraIntrinsicParam,depth,num_imagerow,num_imagecolunm);

camera_center_w = depthinfo(2,:);
deviation_angle = depthinfo(3,:);
currentdepth_pts = CameraCord2world(currentdepth_pts,camera_center_w,deviation_angle);
end

