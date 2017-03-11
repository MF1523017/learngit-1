function [depth_image,rgb_image]=pointcloud2Depth(pointcloud_positions,pointcloud_colors,camera_intrinsic,image_properties)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function: Surface fusion
%   Method:   fuse the surface points to depth
%   Input:    pointcloud_positions:    [3,N];
%             pointcloud_colors:    [3,N];
%             camera_intrinsic:      Camera intrisice matrix
%             image_properties:       e.g.[480,640]
%   Returns:  Live_model:     	Updated Live_model

%   Author:   Jingwei Song.     23/08/2016
%             Jun 15/10/2016    modified. 
%             Jun 16/10/2016    modified, separate the positions and
%             colors.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%�ӵ���ת����ͼ��
%delete the nan points
K =  camera_intrinsic;
index_invalid = isnan(pointcloud_positions(1,:));
pointcloud_positions(:,index_invalid) = [];

depth_image=zeros(image_properties(1),image_properties(2));
rgb_image  = zeros(image_properties(1),image_properties(2),3,'uint8');
rgb_image_1  = zeros(image_properties(1),image_properties(2),'uint8');
rgb_image_2  = zeros(image_properties(1),image_properties(2),'uint8');
rgb_image_3  = zeros(image_properties(1),image_properties(2),'uint8');

px = round(K(1,1)*(pointcloud_positions(1,:)./pointcloud_positions(3,:)) + K(1,3));
py = round(K(2,2)*(pointcloud_positions(2,:)./pointcloud_positions(3,:)) + K(2,3));
%����Щ�����ֵ�ģ�����depth ���£�
is_valid_pixel = (1<=px & px <= image_properties(2) & 1<=py & py<= image_properties(1));
py(~is_valid_pixel) = [];
px(~is_valid_pixel) = [];
pointcloud_positions(:,~is_valid_pixel) = [];
%���px��py
index_inimage = sub2ind(size(depth_image),py,px);
depth_image(index_inimage)= pointcloud_positions(3,:);
rgb_image_1(index_inimage) = uint8(pointcloud_colors(1,:));
rgb_image_2(index_inimage) = uint8(pointcloud_colors(2,:));
rgb_image_3(index_inimage) = uint8(pointcloud_colors(3,:));
rgb_image(:,:,1) = rgb_image_1;
rgb_image(:,:,2) = rgb_image_2;
rgb_image(:,:,3) = rgb_image_3;
%save('data_pointcloud2depth',px,py,depth_image(index_inimage));
end

