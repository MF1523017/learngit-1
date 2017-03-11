function [ is_overlay ] = check_overlap(pointcloud,pose_pointcloud,tsdf_global)
% this is a roughly check.
% check_overlap_between_poingcloud_glolbal_tsdf_volume
%debug 
% range1 = tsdf_current.voxel.range;
% range2 = tsdf_global.voxel.range;

x1 = [min(pointcloud(1,:)),max(pointcloud(1,:))];
y1 = [min(pointcloud(2,:)),max(pointcloud(2,:))];
z1 = [min(pointcloud(3,:)),max(pointcloud(3,:))];

range1 = transRtPoints([x1;y1;z1],pose_pointcloud);

X1 = range1(1,:);
Y1 = range1(2,:);
Z1 = range1(3,:);

range2 = transRtPoints(tsdf_global.voxel.range,tsdf_global.original_pose_H);

X2 = range2(1,:);
Y2 = range2(2,:);
Z2 = range2(3,:);

is_overlay = false;

if max(X1)<min(X2) || min(X1) > max(X2)
    return;
end

if max(Y1)<min(Y2) || min(Y1) > max(Y2)
    return;
end

if max(Z1)<min(Z2) || min(Z1) > max(Z2)
    return;
end

is_overlay = true;
end

