function [ points_1 ] = transRtPoints(points_2,pose2)
%TRANSRTPOINTS Summary of this function goes here
R = eye(3,3);
t = points_2;
%P2����ϵ�����������ϵ����λ��Ϊ��
points_1 = repmat(pose2(1:3,4), [1,size(points_2,2)])+...
pose2(1:3,1:3)*t;
end

