function [ output_args ] = tsdf2ply(tsdf_global,file_name)
%TSDF2PLY Summary of this function goes here
%   Detailed explanation goes here
[y,x,z] = meshgrid(1:tsdf_global.voxel.size_grid(2),1:tsdf_global.voxel.size_grid(1),1:tsdf_global.voxel.size_grid(3));
tsdf_global.tsdf_value(tsdf_global.tsdf_value == 1) = nan;
[f1,v1]=MarchingCubes(x,y,z,tsdf_global.tsdf_value,0);

%transform to global points
[pointcloud_local(:,1),pointcloud_local(:,2),pointcloud_local(:,3)] = tsdf_global.transGrid2PointCloud(v1(:,1),v1(:,2),v1(:,3));
global_pose = tsdf_global.original_pose_H;
global_pose(4,:) = [];
pointcloud_global = transformRT(pointcloud_local',global_pose);
pointcloud_global = pointcloud_global';

%write_ply(v1,f1,'data/tsdf_current.ply');
write_ply(pointcloud_global,f1,file_name);

end

