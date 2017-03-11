function [ global_tsdf ] = joinMap( global_tsdf,sub_tsdf )
%JOINMAP Summary of this function goes here
%   Detailed explanation goes here
%��submap��global map���кϲ����ο�submap update�����е�������
%{���߲�ͬ�ĵط����ڣ�
%   1����submap update�У��ǽ�ÿһ���㣨������depthimage ��ɵ���ά���ƣ�?%          ת����reference tsdf������Ȼ�������tsdf�е���꣬����������ںϡ�
%   2����������Խ�����tsdf grid ��ת���ɾֲ���꣬Ȼ��ת����reference���ϵ�£�?%          ȥ������reference tsdf�еĵ㣬Ȼ��ת����grid����㣬����������ںϡ�
%          %}
%            
%��current submap�е�tsdf��ת�����ֲ�����£�?
[y_grid,x_grid,z_grid] = meshgrid([1:sub_tsdf.voxel.size_grid(2)],[1:sub_tsdf.voxel.size_grid(1)],...
                                                                     [1:sub_tsdf.voxel.size_grid(3)]);%[x_grid,y_grid,z_grid]  
[x_current_local ,y_current_local ,z_current_local] = sub_tsdf.transGrid2PointCloud(x_grid,...
                                                                    y_grid,z_grid);
[m,n,q] = size(x_current_local);
x_current_local = reshape(x_current_local,[1,m*n*q]);
y_current_local = reshape(y_current_local,[1,m*n*q]);
z_current_local = reshape(z_current_local,[1,m*n*q]);
current_weights = reshape(sub_tsdf.tsdf_weight,[1,m*n*q]);
current_value = reshape(sub_tsdf.tsdf_value,[1,m*n*q]);
current_points_local = [x_current_local;y_current_local;z_current_local;ones(1,m*n*q)];

%%delete those value== 1 or weights == 0
index_value_1 = current_value == 1;
current_weights(index_value_1) = [];
current_value(index_value_1) = [];
current_points_local(:,index_value_1) = [];


%%
% current_points_global = transRtPoints(current_points_local(1:3,:),...
%    sub_tsdf.original_pose_H);
% current_points_global = current_points_global';
% homo_trans = inv(global_map.original_pose_H)*sub_map.original_pose_H;

homo_trans = inv(sub_map.original_pose_H)*global_map.original_pose_H;
current_points_global = homo_trans*current_points_local;
% ��������Χ�Ľ��й���
offset_little = global_tsdf.voxel.unit/4;
index_invalid_x = current_points_global(1,:) < global_tsdf.voxel.range(1,1) - offset_little | current_points_global(1,:) ...
                          > global_tsdf.voxel.range(1,2) + offset_little;
current_points_global(:,index_invalid_x) = [];
current_weights(:,index_invalid_x) = [];
current_value(:,index_invalid_x) = [];
index_invalid_y = current_points_global(2,:) < global_tsdf.voxel.range(2,1) - offset_little | current_points_global(2,:) ...
                          > global_tsdf.voxel.range(2,2) + offset_little;
current_points_global(:,index_invalid_y) = [];
current_weights(:,index_invalid_y) = [];
current_value(:,index_invalid_y) = [];
index_invalid_z = current_points_global(3,:) < global_tsdf.voxel.range(3,1) - offset_little| current_points_global(3,:)...
                          > global_tsdf.voxel.range(3,2) + offset_little;
current_points_global(:,index_invalid_z) = [];
current_weights(:,index_invalid_z) = [];
current_value(:,index_invalid_z) = [];
%filter out the 0 weights points
index_invalid_weights = current_weights == 0;
current_points_global(:,index_invalid_weights) = [];
current_weights(:,index_invalid_weights) = [];
current_value(:,index_invalid_weights) = [];

%current_grids_global(1,:) = uint16(ceil((current_points_global(1,:)-global_map.voxel.range(1,1))/global_map.voxel.unit)+1);
%current_grids_global(2,:) = uint16(ceil((current_points_global(2,:)-global_map.voxel.range(2,1))/global_map.voxel.unit)+1);
%current_grids_global(3,:) = uint16(ceil((current_points_global(3,:)-global_map.voxel.range(3,1))/global_map.voxel.unit)+1);
% round and check if zeros or max;
current_grids_global(1,:) = round((current_points_global(1,:)-global_tsdf.voxel.range(1,1))/global_tsdf.voxel.unit);
current_grids_global(2,:) = round((current_points_global(2,:)-global_tsdf.voxel.range(2,1))/global_tsdf.voxel.unit);
current_grids_global(3,:) = round((current_points_global(3,:)-global_tsdf.voxel.range(3,1))/global_tsdf.voxel.unit);
index_zero = current_grids_global(1,:)==0;
current_grids_global(1,index_zero) = 1;
index_zero = current_grids_global(2,:) == 0;
current_grids_global(2,index_zero) = 1;
index_zero = current_grids_global(3,:) == 0;
current_grids_global(3,index_zero) = 1;
index_max = current_grids_global(1,:) == global_tsdf.voxel.size_grid(1)+1;
current_grids_global(1,index_max) = global_tsdf.voxel.size_grid(1);
index_max = current_grids_global(2,:) == global_tsdf.voxel.size_grid(2)+1;
current_grids_global(2,index_max) = global_tsdf.voxel.size_grid(2);
index_max = current_grids_global(3,:) == global_tsdf.voxel.size_grid(3)+1;
current_grids_global(3,index_max) = global_tsdf.voxel.size_grid(3);





indexgrids_current_global = sub2ind(global_tsdf.voxel.size_grid',current_grids_global(1,:),...
      current_grids_global(2,:),current_grids_global(3,:));
  

% ʹ��������λ�ý�global ��tsdf���и��� 
% �ȸ���tsdf values��Ȼ���ٸ���weights��
%debug;

old_weights = global_tsdf.tsdf_weight(indexgrids_current_global);
new_weights = old_weights + current_weights;
global_tsdf.tsdf_weight(indexgrids_current_global) = new_weights;
global_tsdf.tsdf_value(indexgrids_current_global) = (global_tsdf.tsdf_value(indexgrids_current_global).*...
    old_weights+current_value.*current_weights)./new_weights;

% debug;
% new_values = (global_tsdf.tsdf_value(indexgrids_current_global).*...
%      old_weights+current_value.*current_weights)./new_weights;


end

