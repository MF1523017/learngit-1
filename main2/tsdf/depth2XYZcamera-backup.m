function XYZcamera = depth2XYZcamera(K, depth)
    [x,y] = meshgrid(1:640, 1:480);
    XYZcamera(:,:,1) = x;
    XYZcamera(:,:,2) = y;
    XYZcamera(:,:,3) = 0;
    %firstly find the depth==0;
    [rows,cols] = find(depth); 
    XYZcamera(rows,cols,1) = (x(rows,cols)-K(1,3)).*depth(rows,cols)/K(1,1);
    XYZcamera(rows,cols,2) = (y(rows,cols)-K(2,3)).*depth(rows,cols)/K(2,2);
    XYZcamera(rows,cols,3) = depth(rows,cols);
    XYZcamera(:,:,4) = depth~=0;
    
end
