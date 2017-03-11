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
%一共有三部分的occlusion要分开
%1：超出当前相机下观测范围，本身就看不到的点 point_cloud_occlusion1
%2：在当前相机观测范围，但是有前面的点挡着看不见的，point_cloud_occlusion2
%3:   虽然没有点挡着，但是由于格网的精度原因，有些点不会计算tsdf值，把这些点也提取出来.
%          point_cloud_occlusion3.  (这一部分点暂时不返回，以后可能)
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
%%%%%%%%%%%%%这是第一部分的occlusion点。
pts_occlusion1= pts_model(~is_valid_pixel,:);
pts_model(~is_valid_pixel,:) = [];
%获得px、py
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
%%%%%%%%%这是第二部分的occlusion点；
pts_occlusion2 = pts_model(flag_occlusion_list,:);
pts_free = pts_model(~flag_occlusion_list,:);

%找出第三部分的occlusion点; 暂时不处理。
% index_occlusion_3 = get_occlusion3(K,image_size);
% flag_occlusion3_list = logical(zeros(size(flag_occlusion_list,1),1));
% %for every index;
% for i=1:size(index_occlusion_3,2)
%     index = index_occlusion_3(i);
%     %查找这个地方是否有点，如果有，则把该点置为occlusion
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
%对那些计算出值的，进行depth 更新；
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

