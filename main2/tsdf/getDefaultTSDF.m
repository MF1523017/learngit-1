function [ tsdf_class_1 ] = getDefaultTSDF(camera_intrinsic,pose)
tsdf_class_1 = TSDF_CLASS();
tsdf_class_1.init(pose,camera_intrinsic);
tsdf_class_1.setoffset([0;0;0]);
end

