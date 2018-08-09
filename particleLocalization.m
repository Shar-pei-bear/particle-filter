function myPose = particleLocalization(ranges, scanAngles, map, param)
% map is set to zero when grid status is unknown
map = map - 0.5;
% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

% The initial pose is given
myPose(:,1) = param.init_pose;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, param.M]);
% initialize particle weights.
W = ones(1,param.M)/param.M;
for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
% 
%     % 1) Propagate the particles
    dx = param.sigma_x*randn(1,param.M);
    dy = param.sigma_y*randn(1,param.M);
    do = param.sigma_o*randn(1,param.M);
    P = P + [dx;dy;do];
%     % 2) Measurement Update 
%     %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame) 
    lidar_global_x = ceil(( ranges(:,j).*cos(scanAngles + P(3,:)) + P(1,:))*param.resol + param.origin(1));
    lidar_global_y = ceil((-ranges(:,j).*sin(scanAngles + P(3,:)) + P(2,:))*param.resol + param.origin(2));
    
    lidar_global_x = min(lidar_global_x, param.size(2));
    lidar_global_x = max(lidar_global_x,             1);
    lidar_global_y = min(lidar_global_y, param.size(1));
    lidar_global_y = max(lidar_global_y,             1);
%     %   2-2) For each particle, calculate the correlation scores of the particles
    lidar_global_ind = sub2ind(param.size, lidar_global_y(:),lidar_global_x(:));
    score = map(lidar_global_ind).*((map(lidar_global_ind) >= 0)*param.TP + (map(lidar_global_ind) < 0)*param.FP); 
    score = reshape(score,[],param.M);
    score = sum(score,1);
%     %   2-3) Update the particle weights         
    W = W.*exp(score);
    W = W/sum(W);
%     %   2-4) Choose the best particle to update the pose
    [~,I] = max(W);
    myPose(:,j) = P(:,I);
%     % 3) Resample if the effective number of particles is smaller than a threshold
    N_eff = 1./sum(W.^2)/param.M;
    if N_eff < 0.1
           %resample
           P = datasample(P',param.M,'Weights',W)';
           W = ones(1,param.M)/param.M;
    end
end

end

