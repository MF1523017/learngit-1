addpath('jsonlab')
data=LoadData()
dep_image=imread(char(data.depth(1)));
K=data.K;
pointcloud=depth2Pointcloud(K,dep_image);
plot3(pointcloud(:,1),pointcloud(:,2),pointcloud(:,3),'.')
grid on