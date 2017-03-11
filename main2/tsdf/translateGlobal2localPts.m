%generate point cloud list
num_pts = 2;
i=1
    filename_depthinfo = ['data/', 'depthinfo', int2str(i), '.mat'];
    depthinfo = load(filename_depthinfo);
    filename_depthpts =  ['data/', 'depthpts', int2str(i), '.mat'];
    depthpts = load(filename_depthpts);
    % translate point clouds to local frame;
    %showPointCloud(pointCloud(depthpts.depthpts2));
    [pointcloud_local pose]=  world2CameraCord(depthpts.depthpts1,depthinfo.datainfo_depth(2,:),depthinfo.datainfo_depth(3,:));
    filename_out = ['data/','depthpts_local',int2str(i),'.mat'];
    save(filename_out,'pointcloud_local','pose');

 i=2
    filename_depthinfo = ['data/', 'depthinfo', int2str(i), '.mat'];
    depthinfo = load(filename_depthinfo);
    filename_depthpts =  ['data/', 'depthpts', int2str(i), '.mat'];
    depthpts = load(filename_depthpts);
    % translate point clouds to local frame;
    %showPointCloud(pointCloud(depthpts.depthpts2));
    [pointcloud_local pose]=  world2CameraCord(depthpts.depthpts2,depthinfo.datainfo_depth(2,:),depthinfo.datainfo_depth(3,:));
    filename_out = ['data/','depthpts_local',int2str(i),'.mat'];
    save(filename_out,'pointcloud_local','pose');