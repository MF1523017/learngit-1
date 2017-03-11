function depth_image=pointcloud2Depth(point_cloud,camera_intrinsic,image_properties)
%�ӵ���ת����ͼ��
%delete the nan points
K =  camera_intrinsic;
index_invalid = isnan(point_cloud(1,:));
point_cloud(:,index_invalid) = [];
depth_image=zeros(480,640);
px = round(K(1,1)*(point_cloud(1,:)./point_cloud(3,:)) + K(1,3));
py = round(K(2,2)*(point_cloud(2,:)./point_cloud(3,:)) + K(2,3));
%����Щ�����ֵ�ģ�����depth ���£�
is_valid_pixel = (1<=px & px <= image_properties(2) & 1<=py & py<= image_properties(1));
py(~is_valid_pixel) = [];
px(~is_valid_pixel) = [];
point_cloud(:,~is_valid_pixel) = [];
%���px��py
index_inimage = sub2ind(size(depth_image),py,px);
depth_image(index_inimage)= point_cloud(3,:);
%save('data_pointcloud2depth',px,py,depth_image(index_inimage));
end

