function [ VMap,NMap] = getPointsFrmRaycasting(sub_map,pose_current,camera_intrinsic)
%输入：submap，当前的姿态（相对姿态，相对第一帧的）
%输出：depth_image;
global raycastingDirectionC;
global voxel;
global tsdf_value;%这里设置全局变量是因为在raycasting中是这样的，以后再更改。

K = camera_intrinsic;
f=K(1,1);
[pX,pY]=meshgrid(1:640,1:480);%这里需要再检查一下顺序。
raycastingDirectionC = [pX(:)'-K(1,3); pY(:)'-K(2,3); f*ones(1,640*480)]; % clipping at 8 meter is the furthest depth of kinect
raycastingDirectionC = raycastingDirectionC ./ repmat(sqrt(sum(raycastingDirectionC.^2,1)),3,1);
tsdf_value = sub_map.tsdf_value;

voxel = sub_map.voxel;
voxel.mu = sub_map.voxel.mu_grid*sub_map.voxel.unit;  %voxel.mu_grid; %这个地方有严重问题；
%[VMap,NMap,tMap,CMap] = raycastingTSDFvectorized([eye(3) [0;0;-1]], [0.4 8]);

 [VMap,NMap,tMap,CMap] = raycastingTSDFvectorized(pose_current, [0 15]);%from the camera to frustum
 
 
 
 %% 这里全部采用相机不变eye(4,4), 所以全局和局部是一样的。
 %根据p3=p2+th2, 其中p3是VMap的坐标，P2是当前pose的坐标，h2是
%当前pose的heading,t 其实就是VMap在P2中的局部坐标了。
% %delete the nan
% index_invalid = isnan(VMap(1,:));
% VMap(:,index_invalid) =[];
% 
% P2 = pose_current(1:3,4);
% %VMap_local = inv(pose_current(1:3,1:3))*(VMap(1:3,:)-repmat(P2,[1,size(VMap,2)]));
% VMap_local = pose_current(1:3,1:3)\(VMap(1:3,:)-repmat(P2,[1,size(VMap,2)]));
% %VMap_local = (VMap(1:3,:)'-repmat(P2,[size(VMap,2),1] ))/(pose_current(1:3,1:3));
% % When do this, the accuracy will be decrease. so we need not use this;
% %depth_image =  pointcloud2Depth(VMap_local,parameter_settings);
% 
% points_global = VMap(1:3,:);
% 
% points_local = VMap_local;
end

function depth_image=pointcloud2Depth(point_cloud,parameter_settings)
%从点云转换到图像
K = parameter_settings.camera_intrinsic;
depth_image=nan(480,640);
px = round(K(1,1)*(point_cloud(1,:)./point_cloud(3,:)) + K(1,3));
py = round(K(2,2)*(point_cloud(2,:)./point_cloud(3,:)) + K(2,3));
%对那些计算出值的，进行depth 更新；
is_valid_pixel = (1<=px & px <= 640 & 1<=py & py<= 480);
py(~is_valid_pixel) = [];
px(~is_valid_pixel) = [];
point_cloud(:,~is_valid_pixel) = [];
%获得px、py
index_inimage = sub2ind(size(depth_image),py,px);
depth_image(index_inimage)= point_cloud(3,:);
end

