%֮ǰ�أ���Ҫ�����õ�ԭ����������ݴ洢�ṹ����������Ҫ�Լ��滮һ�����ݴ洢�ṹ
%

addpath('data\depth')

dep_image1=imread('1.png');
dep_image2=single(dep_image1);
camera_in=importdata('a.txt');%��ȡ����ڲ�

pointcloud=depth2Pointcloud(camera_in,dep_image1)./160;
%pointcloud=depth2XYZcamera(camera_in,dep_image1);
plot3(pointcloud(:,1),pointcloud(:,2),pointcloud(:,3),'.')
grid on