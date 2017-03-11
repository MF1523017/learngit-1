function [ points_1 ] = transRtPoints(points_2,pose2)
%TRANSRTPOINTS Summary of this function goes here
R = eye(3,3);
t = points_2;
%P2坐标系（假设的坐标系）的位置为：
points_1 = repmat(pose2(1:3,4), [1,size(points_2,2)])+...
pose2(1:3,1:3)*t;
end

