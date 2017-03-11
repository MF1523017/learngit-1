%之前呢，主要还是用的原来代码的数据存储结构，现在我需要自己规划一个数据存储结构
%

addpath('data\depth')

dep_image1=imread('1.png');
dep_image2=single(dep_image1);
camera_in=importdata('a.txt');%读取相机内参

pointcloud=depth2Pointcloud(camera_in,dep_image1)./160;
%pointcloud=depth2XYZcamera(camera_in,dep_image1);
plot3(pointcloud(:,1),pointcloud(:,2),pointcloud(:,3),'.')
grid on