function [xyzPoints, xyzNormal] = RGBD2XYZcamera(K, depth,RGB_image,image_row,image_col)
% column:1-3:positions 4-6:RGB 7-9:normals

    [x_size,y_size]  = size(depth);
    [x,y] = meshgrid(1:y_size, 1:x_size);
    XYZcamera(:,:,1) = (x-K(1,3)).*depth/K(1,1);
    XYZcamera(:,:,2) = (y-K(2,3)).*depth/K(2,2);
    XYZcamera(:,:,3) = depth;
    XYZcamera(:,:,4) = depth~=0;
    
    x = XYZcamera(:,:,1);
    y = XYZcamera(:,:,2);
    z = XYZcamera(:,:,3);
    xyzPoints(:,1)=reshape(x,[image_row*image_col,1]);
    xyzPoints(:,2)=reshape(y,[image_row*image_col,1]);
    xyzPoints(:,3)=reshape(z,[image_row*image_col,1]);
    xyzPoints(:,4)=reshape(RGB_image(:,:,1),[image_row*image_col,1]);
    xyzPoints(:,5)=reshape(RGB_image(:,:,2),[image_row*image_col,1]);
    xyzPoints(:,6)=reshape(RGB_image(:,:,3),[image_row*image_col,1]);
    
    %   Estimating normals
    [ d_depth_X,d_depth_Y] = calculateDerivativeImage(depth,'robert');
    xyzPoints(:,7) = reshape(d_depth_X,[image_row*image_col,1]);
    xyzPoints(:,8) = reshape(d_depth_Y,[image_row*image_col,1]);
    xyzPoints(:,9) = -1;
    xyzPoints(:,7:8) = xyzPoints(:,7:8) * (-1);
    
    %   Delete NaN
    [I,J] = find(isnan(xyzPoints));
    xyzPoints(I,:) = [];
    
    %   Delete points with Z==0
    [I,J] = find(xyzPoints(:,3)==0);
    xyzPoints(I,:) = [];
    
    %   Regularize normal
    magnitude = sqrt(xyzPoints(:,7).^2+xyzPoints(:,8).^2+xyzPoints(:,9).^2);
    xyzPoints(:,7:9) = xyzPoints(:,7:9) ./ repmat(magnitude,[1,3]);
    
    xyzNormal   = xyzPoints(:,7:9);
    xyzPoints   = xyzPoints(:,1:6);
%     %   For test
%     scatter3(xyzPoints(:,1),xyzPoints(:,2),xyzPoints(:,3),'.');
%     hold on;
%     quiver3(xyzPoints(:,1),xyzPoints(:,2),xyzPoints(:,3),xyzPoints(:,7),xyzPoints(:,8),xyzPoints(:,9));
end
