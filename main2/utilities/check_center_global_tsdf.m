% check for the best location of global tsdf;

addpath('utilities');
%%
%===================for dataset kitchen 5=====================%
kitchen_room = read_ply_noface('/var/www/html/data/kitchen_room/5/sun3d_output/kitchen_room/5/box_BA_type.ply');
showPointCloud(kitchen_room);
xlabel('x');ylabel('y');
% the range of x/y/z are:
% x[-1.8,1.5]; y[-0.2,3];z[-2,2];

%%
%===================for dataset kitchen 10=====================%
kitchen_room = read_ply_noface('/var/www/html/data/kitchen_room/10/sun3d_output/kitchen_room/10/box_BA_type.ply');
showPointCloud(kitchen_room);
xlabel('x');ylabel('y');
% the range of x/y/z are:
% x[-1,2]; y[-0.1,3];z[-1,3];

  


%%
%===================for dataset lab1 2=======================%
sequenceName = 'lab1/2/';
SUN3Dpath = '/var/www/html/data/';
ply_file_name = [SUN3Dpath,sequenceName,'sun3d_output/',sequenceName,'box_BA_type.ply'];
lab1_2_vertices = read_ply_noface(ply_file_name);
showPointCloud(lab1_2_vertices);
xlabel('x');ylabel('y');zlabel('z');
% the range of x/y/z are:
% x[-2.6, 4.5] y[-0.5,3.5] z[-5,5];
%designed four global tsdf for the room; 
% x[-2,6, 1, 4.5]; y[-0.5,3.5],z [-5,0,5];
% offset for each global tsdf; overlap = 0.06;
% 1: [-0.97,1.5,-2.47]  3: [-0.97,1.5,2.47], 2:[2.97,1.5,-2.47 ]; 4:[2.97,1.5,2.47]; 


%%
addpath('utilities');
%====================fro dataset meeting room=======================%
sequenceName = 'meeting_room/4/';
SUN3Dpath = '/var/www/html/data/';
ply_file_name = [SUN3Dpath,sequenceName,'sun3d_output/',sequenceName,'box_BA_type.ply'];
lab1_2_vertices = pcread(ply_file_name);
showPointCloud(lab1_2_vertices);
xlabel('x');ylabel('y');zlabel('z');
% the range of x/y/z are:
overlap = 0.03;
half_size_tsdf = [2,2,2];
x = [-5, 2.6]; y=[-1,3]; z = [-2.5,3.5];
model_extend = [x;y;z];
% center x:        y: 1.5;
% calculate the center;
x_mid = (x(2)-x(1))/2 + x(1); z_mid = (z(2)-z(1))/2+z(1); 
x_center(2) = (x_mid-overlap+half_size_tsdf(1));
x_center(1) = (x_mid+overlap-half_size_tsdf(1));
z_center(2) = (z_mid-overlap+half_size_tsdf(3));
z_center(1) = (z_mid+overlap-half_size_tsdf(3));
y_center(1) = (y(2)-y(1))/2+y(1);
save('center','x_center','y_center','z_center','model_extend');

%designed four global tsdf for the room; 
% x[-2,6, 1, 4.5]; y[-0.5,3.5],z [-5,0,5];
% offset for each global tsdf; overlap = 0.06;
% 1: [-0.97,1.5,-2.47]  3: [-0.97,1.5,2.47], 2:[2.97,1.5,-2.47 ]; 4:[2.97,1.5,2.47]; 




