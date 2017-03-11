function [ pts_model_deleted ] = deleteModelptsInFrontScan( pts_model,pts_scan,cameraIntrinsicParam,image_size )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function: delete the model points in front of the new scan
%   Method:   
%   Input: pts_model:           point cloud of the model(just the free points)
%              pts_new:              point cloud of current scan. (please be sure that both points are in the camera frustum)
%             cameraIntrinsicParam:      Camera intrinsic matrix
%              image_size:          [480 640]
%   Returns:  
%             pts_model_deleted: model points without points in front of
%                                                   the pts_model
%             
%   Author:   Jun Wang.   06/09/2016 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% for testing
% pts_model = pts_free;
% pts_scan = pts_new;

%%
K =  cameraIntrinsicParam;
model_px = round(K(1,1)*(pts_model(:,1)./pts_model(:,3)) + K(1,3));
model_py = round(K(2,2)*(pts_model(:,2)./pts_model(:,3)) + K(2,3));
%model_uv_list = [model_py,model_px];
model_index_list = sub2ind(image_size,model_py,model_px);
%model_index_unique_list = unique(model_index_list);
model_flag_infront_list = logical(zeros(size(pts_model,1),1));

scan_px = round(K(1,1)*(pts_scan(:,1)./pts_scan(:,3)) + K(1,3));
scan_py = round(K(2,2)*(pts_scan(:,2)./pts_scan(:,3)) + K(2,3));
%scan_uv_list = [scan_py,scan_px];
scan_index_list = sub2ind(image_size,scan_py,scan_px);
%scan_index_unique_list = unique(scan_index_list);

%for every index in model
model_index_points = 1:size(pts_model);
model_index_image = model_index_list(model_index_points);

[is_member,scan_index_points]= ismember(model_index_image,scan_index_list);

for i=1:size(is_member,1)
    if is_member(i) == 0
        continue;
    end
    model_depth = pts_model(model_index_points(i),3);
    scan_depth = pts_scan(scan_index_points(i),3);
    if model_depth < scan_depth
        model_flag_infront_list(model_index_points(i)) = logical(1);
    end
end

% delete the points infront
pts_model_deleted = pts_model(~model_flag_infront_list,:);

end

