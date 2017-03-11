function [ tsdf_global ] = initGlobalMap( camera_intrinsic_param )
%generate a global (for map joining)
tsdf_global = TSDF_CLASS;
depth_image = zeros(480,640);
tsdf_global.init(eye(4,4),camera_intrinsic_param);
tsdf_global.setoffset([0;0;0.2]);
tsdf_global.getTSDF(depth_image);

end

