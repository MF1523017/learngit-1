function [ VMap,NMap] = getPointsFrmRaycasting(sub_map,pose_current,camera_intrinsic)
%���룺submap����ǰ����̬�������̬����Ե�һ֡�ģ�
%�����depth_image;
global raycastingDirectionC;
global voxel;
global tsdf_value;%��������ȫ�ֱ�������Ϊ��raycasting���������ģ��Ժ��ٸ��ġ�

K = camera_intrinsic;
f=K(1,1);
[pX,pY]=meshgrid(1:640,1:480);%������Ҫ�ټ��һ��˳��
raycastingDirectionC = [pX(:)'-K(1,3); pY(:)'-K(2,3); f*ones(1,640*480)]; % clipping at 8 meter is the furthest depth of kinect
raycastingDirectionC = raycastingDirectionC ./ repmat(sqrt(sum(raycastingDirectionC.^2,1)),3,1);
tsdf_value = sub_map.tsdf_value;

voxel = sub_map.voxel;
voxel.mu = sub_map.voxel.mu_grid*sub_map.voxel.unit;  %voxel.mu_grid; %����ط����������⣻
%[VMap,NMap,tMap,CMap] = raycastingTSDFvectorized([eye(3) [0;0;-1]], [0.4 8]);

 [VMap,NMap,tMap,CMap] = raycastingTSDFvectorized(pose_current, [0 15]);%from the camera to frustum
 
 
 
 %% ����ȫ�������������eye(4,4), ����ȫ�ֺ;ֲ���һ���ġ�
 %����p3=p2+th2, ����p3��VMap�����꣬P2�ǵ�ǰpose�����꣬h2��
%��ǰpose��heading,t ��ʵ����VMap��P2�еľֲ������ˡ�
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
%�ӵ���ת����ͼ��
K = parameter_settings.camera_intrinsic;
depth_image=nan(480,640);
px = round(K(1,1)*(point_cloud(1,:)./point_cloud(3,:)) + K(1,3));
py = round(K(2,2)*(point_cloud(2,:)./point_cloud(3,:)) + K(2,3));
%����Щ�����ֵ�ģ�����depth ���£�
is_valid_pixel = (1<=px & px <= 640 & 1<=py & py<= 480);
py(~is_valid_pixel) = [];
px(~is_valid_pixel) = [];
point_cloud(:,~is_valid_pixel) = [];
%���px��py
index_inimage = sub2ind(size(depth_image),py,px);
depth_image(index_inimage)= point_cloud(3,:);
end

