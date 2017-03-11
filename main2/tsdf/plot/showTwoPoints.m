function [ output_args ] = showTwoPoints( points1,points2 )
%SHOWTWOPOINTS Summary of this function goes here
%   Detailed explanation goes here
figure;
showPointCloud(points1,repmat([1,0,0],[size(points1,1),1]));
hold on;showPointCloud(points2,repmat([0,1,0],[size(points2,1),1]));
end

