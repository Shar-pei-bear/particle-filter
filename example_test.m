%% Load data
clear all;
close all;

load practice.mat
% This will load four variables: ranges, scanAngles, t, pose
% [1] t is K-by-1 array containing time in second. (K=3701)
% [2] ranges is 1081-by-K lidar sensor readings. 
%     e.g. ranges(:,k) is the lidar measurement at time index k.
% [3] scanAngles is 1081-by-1 array containing at what angles the 1081-by-1 lidar
%     values ranges(:,k) were measured. This holds for any time index k. The
%     angles are with respect to the body coordinate frame.
% [4] M is a 2D array containing the occupancy grid map of the location
%     e.g. map(x,y) is a log odds ratio of occupancy probability at (x,y)

%% Set parameters
param = {};
% 1. Decide map resolution, i.e., the number of grids for 1 meter.
param.resol = 25;

% 2. Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
param.M = 400;                          

% 3. Indicate the origin in pixels
param.origin = [685,572]';

param.init_pose = -init_pose;
param.size = size(M);
%% Run algorithm
param.sigma_x = 0.05;% sigma_x represents the uncertainty in the x velocity
param.sigma_y = 0.05;% sigma_y represents the uncertainty in the y velocity
param.sigma_o = 0.05; % 0.05 sigma_o is an additional perturbation on the orientation

param.TP = 0.01; % true positive correlation score
param.FP = 0.005 ;% false positive correlation score
param.TN = -1;%  true negative correlation score
param.FN = -5;% false negative correlation score
   
pose = particleLocalization(ranges, scanAngles, M, param);
%load practice-answer.mat;

%% Plot final solution
% The final grid map:
figure;
imagesc(M); hold on;

%% Plot LIDAR data
lidar_global(:,1) =  (ranges(:,1).*cos(scanAngles + pose(3,1)) + pose(1,1))*param.resol + param.origin(1);
lidar_global(:,2) =  (-ranges(:,1).*sin(scanAngles + pose(3,1)) + pose(2,1))*param.resol + param.origin(2);

plot(lidar_global(:,1), lidar_global(:,2), 'r.'); 
% 
colormap('gray');
axis equal;
hold on;
plot(pose(1,:)*param.resol+param.origin(1), ...
    pose(2,:)*param.resol+param.origin(2), 'r.-');
