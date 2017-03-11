%clear;clc
addpath(genpath('main2'));
addpath('data\depth');

depth1 = imread('1.png');
depth2 = imread('2.png');
depth3 = imread('3.png');
camera_in = importdata('a.txt');

%initialise the tsdf
for x_count=0:1:200
    for y_count=0:1:200
        for z_count=0:1:200
            x=x_count-100;
            y=y_count-100;
            z=z_count;
            count=x_count+201*y_count+201^2*z+1;%here is explained in the paper of Kintinuous
            %First we should calculate the distance between point and line
            [point_count,point_xyz] = size(pointcloud);
            count2=0;
            for count1=1:point_count
                point_d(count1,1)=norm(cross([x,y,z]-[0,0,0],pointcloud(count1,:)))/norm([x,y,z]-[0,0,0]);%this equaltion is from internet
                point_d(count1,2)=pointcloud(count1,1);
                point_d(count1,3)=pointcloud(count1,2); 
                point_d(count1,4)=pointcloud(count1,3);
                if point_d<10
                    count2=count2+1;
                end
            end

            %norm(cross([x,y,z]-[0,0,0],pointcloud(count1,:)))/norm([x,y,z]-[0,0,0])
            
            tsdf_grid{count}=[x,y,z,0,0];%tsdf_grid{count}=[x,y,z,tsdf_distance,tsdf_weight]
        end
    end
end





