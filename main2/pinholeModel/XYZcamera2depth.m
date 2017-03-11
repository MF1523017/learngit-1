function [ u,v,value ] = XYZcamera2depth( K,pointcloud )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Script:   Transform a pointcloudcloud [X,Y,Z] to depth image [u,v,value]
%   Method:
%   Input:
%             K:                Camera intrinsic matrix
%             pointcloud:            N*3 pointcloud
%   Returns:
%             u:                the 'u'th column of image
%             v:                the 'v'th row of image
%             value:            value of depth image [u,v]
%   Author:   Jingwei Song.     02/08/2016
%              Jun    filter the valid uv;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
num_point = size(pointcloud,1);
u       = zeros(num_point,1);
v       = zeros(num_point,1);
value   = zeros(num_point,1);

for i = 1 : num_point
    u(i)      = round(K(1,1)*(pointcloud(i,1)./pointcloud(i,3)) + K(1,3));
    v(i)      = round(K(2,2)*(pointcloud(i,2)./pointcloud(i,3)) + K(2,3));
    value(i)  = pointcloud(i,3);
end

size_image = [480,640];
index_invalid = u<=0 | u> size_image(2) | v<=0 | v> size_image(1) | value<=0;
u(index_invalid) = [];
v(index_invalid) = [];
value(index_invalid) = [];
end

