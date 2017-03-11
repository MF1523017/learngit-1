function [ pts_occlusion1, pts_occlusion2, pts_free] = getOcclusionPts( pts_model,cameraIntrinsicParam,image_size )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function: Get occlusion points from given point cloud model.
%   Method:   Separate two (or three) parts of points in the model.
%   Input: pts_model:           point cloud of the model
%             cameraIntrinsicParam:       Camera intrinsic matrix
%             image_size:          usually: [480,640]
%   Returns:  
%             pts_occlusion1: occlusion points in the model (part 1)
%             pts_occlusion2: occlusion points in the model (part 2)
%             pts_free:             free points that can be used in tsdf fusion
%   Author:   Jun Wang.   03/09/2016 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%һ���������ֵ�occlusionҪ�ֿ�
%1��������ǰ����¹۲ⷶΧ������Ϳ������ĵ� point_cloud_occlusion1
%2���ڵ�ǰ����۲ⷶΧ��������ǰ��ĵ㵲�ſ������ģ�point_cloud_occlusion2
%3:   ��Ȼû�е㵲�ţ��������ڸ����ľ���ԭ����Щ�㲻�����tsdfֵ������Щ��Ҳ��ȡ����.
%          point_cloud_occlusion3.  (��һ���ֵ���ʱ�����أ��Ժ����)
%find occlusion points of current pose
% uv_list; index_list; flag_occlusion_list;
%calculate the uv of each point
K =  cameraIntrinsicParam;
index_invalid = isnan(pts_model (:,1));
pts_model(:,index_invalid) = [];
px = round(K(1,1)*(pts_model(:,1)./pts_model(:,3)) + K(1,3));
py = round(K(2,2)*(pts_model(:,2)./pts_model(:,3)) + K(2,3));
is_valid_pixel = (1<=px & px <= image_size(2) & 1<=py & py<= image_size(1));
py(~is_valid_pixel) = [];
px(~is_valid_pixel) = [];
%%%%%%%%%%%%%���ǵ�һ���ֵ�occlusion�㡣
pts_occlusion1= pts_model(~is_valid_pixel,:);
pts_model(~is_valid_pixel,:) = [];
%���px��py
uv_list = [py,px];
index_list = sub2ind(image_size,py,px);
index_list_unique = unique(index_list);
flag_occlusion_list = logical(ones(size(pts_model,1),1));
%for every index;
for i=1:size(index_list_unique,1)
index_unique = index_list_unique(i);
index_unique_points = find(index_list==index_unique);
[min_z,index_min_unique_points] = min(pts_model(index_unique_points,3));
index_min_points = index_unique_points(index_min_unique_points);
flag_occlusion_list(index_min_points) = logical(0);
end
%%%%%%%%%���ǵڶ����ֵ�occlusion�㣻
pts_occlusion2 = pts_model(flag_occlusion_list,:);
pts_free = pts_model(~flag_occlusion_list,:);

%�ҳ��������ֵ�occlusion��; ��ʱ������
% index_occlusion_3 = get_occlusion3(K,image_size);
% flag_occlusion3_list = logical(zeros(size(flag_occlusion_list,1),1));
% %for every index;
% for i=1:size(index_occlusion_3,2)
%     index = index_occlusion_3(i);
%     %��������ط��Ƿ��е㣬����У���Ѹõ���Ϊocclusion
%     index_points=find(index_list == index);
%     flag_occlusion3_list(index_points) = logical(1);
% end
% pts_occlusion3 = pts_model(flag_occlusion3_list,:);
end

function index_occlusion_3 = get_occlusion3(camera_intrinsic,image_size)
tsdf_class_1 = getDefaultTSDF(camera_intrinsic);
view_frustum_c = tsdf_class_1.getViewFrustumC();
rangeGrid = tsdf_class_1.getRangeGrid(view_frustum_c);
[Y,X,Z]=meshgrid(rangeGrid(2,1):rangeGrid(2,2),rangeGrid(1,1):rangeGrid(1,2),rangeGrid(3,1):rangeGrid(3,2)); % strange matlab syntax
X = X(:)'; Y = Y(:)'; Z = Z(:)';
[gridCoordinateC(1,:),gridCoordinateC(2,:),gridCoordinateC(3,:)]= tsdf_class_1.transGrid2PointCloud(single(X),single(Y),single(Z));
gridCoordinateC = gridCoordinateC';
K = camera_intrinsic;
px = round(K(1,1)*(gridCoordinateC(:,1)./gridCoordinateC(:,3)) + K(1,3));
py = round(K(2,2)*(gridCoordinateC(:,2)./gridCoordinateC(:,3)) + K(2,3));
%����Щ�����ֵ�ģ�����depth ���£�
is_valid_pixel = (1<=px & px <= image_size(2) & 1<=py & py<= image_size(1));
py(~is_valid_pixel) = [];
px(~is_valid_pixel) = [];
gridCoordinateC(~is_valid_pixel,:) = [];
index_list_occlusion3 = sub2ind(image_size,py,px);
index_unique_list_occusion3 = unique(index_list_occlusion3);
index_all_images = 1:image_size(1)*image_size(2)';
index_have_cube = ismember(index_all_images,index_unique_list_occusion3);
index_not_have_cube = index_all_images(~index_have_cube);
index_occlusion_3 = index_not_have_cube;
end

