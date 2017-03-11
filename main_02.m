clear;
addpath(genpath('main2'));
addpath('data\depth');

frameIDs = [];
load data1.mat

tsdf_global1 = getDefaultTSDF(data.K,eye(4,4));%只是传了相机的内参外参？？
tsdf_global1.setoffset([0.7;1.5;0.5]);


num_frame = size(data.depth,2);
sampling_interval = 20;
%num_frame = 810;
start_id = 1;


%%    
% merge with global1;
frame_id = start_id;
while(frame_id< 2)%这里的num_frame是需要重合的帧数
    disp(frame_id);
    tsdf_current = getDefaultTSDF(data.K,data.extrinsicsC2W(:,:,frame_id));
    tsdf_current.setoffset([0;0;1]);
    tsdf_current.voxel.range
    tsdf_current.original_pose_H
%     if ~check_overlap(transRtPoints(tsdf_current.voxel.range,tsdf_current.original_pose_H),...
%             transRtPoints(tsdf_global1.voxel.range,tsdf_global1.original_pose_H))
%         disp('no overlap');
%         continue
%     end
    depth = depthRead(data.depth{1,frame_id});
    %depth = single(imread('1.png'));
    disp('tsdf creating');
    tic;
    tsdf_current.getTSDF(depth);
    toc;
    disp('tsdf join');
    tic;
    %tsdf_global1 = joinMap(tsdf_global1,tsdf_current);
    toc;
    frame_id = frame_id+sampling_interval;
end
%save('tsdf_current','tsdf_current');
save('tsdf_global1','tsdf_current');
%clear tsdf_global1;


%{
%% 
%========================for the second global tsdf==============%
frame_id = start_id;
%first global (0.7,1.5,-0.5); second global (0.7,1.5,1.5);
tsdf_global2 = getDefaultTSDF(data.K,eye(4,4));
tsdf_global2.setoffset([0.7;1.5;1.5]);
while(frame_id< num_frame)
    disp(frame_id);
    tsdf_current = getDefaultTSDF(data.K,data.extrinsicsC2W(:,:,frame_id));
    tsdf_current.setoffset([0;0;1]);
    if ~check_overlap(transRtPoints(tsdf_current.voxel.range,tsdf_current.original_pose_H),...
            transRtPoints(tsdf_global2.voxel.range,tsdf_global2.original_pose_H))
        disp('no overlap');
        continue
    end
    depth = depthRead(data.depth{1,frame_id});
    disp('tsdf creating');
    tic;
    tsdf_current.getTSDF(depth);
    toc;
    disp('tsdf join');
    tic;
    tsdf_global2 = joinMap(tsdf_global2,tsdf_current);
    toc;
    frame_id = frame_id+sampling_interval;
end
save('tsdf_global2','tsdf_global2');
clear tsdf_global2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}


%%
%%%%%%%%%%%%%%%%%%%save to ply%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% marching cube
[y,x,z] = meshgrid(1:200,1:200,1:200);
tsdf_current.tsdf_value(tsdf_current.tsdf_value == 1) = nan;
[f1,v1]=MarchingCubes(x,y,z,tsdf_current.tsdf_value,0);

%write_ply(v1,f1,'data/tsdf_current.ply');
write_ply(v1,f1,'data/tsdf_global1.ply');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
