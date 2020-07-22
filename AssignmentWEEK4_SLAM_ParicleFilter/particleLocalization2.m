% 
% function myPose = particleLocalization(ranges, scanAngles, map, param)
% 
% % Number of poses to calculate
% N = size(ranges, 2);
% % Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
% myPose = zeros(3, N);
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% % Map Parameters 
% % 
% % the number of grids for 1 meter.
% myResolution = param.resol;
% % the origin of the map in pixels
% myOrigin = param.origin; 
% 
% % The initial pose is given
% myPose(:,1) = param.init_pose;
% % You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% % The pose(:,1) should be the pose when ranges(:,j) were measured.
% 
% 
% 
% % Decide the number of particles, M.
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% M = 200;   						% Please decide a reasonable number of M, 
%                                	% based on your experiment using the practice data.
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Create M number of particles
% P = repmat(myPose(:,1), [1, M]);
% Sigma = diag([0.1 0.1 0.3]);
% % R = chol(Sigma);
% % R = diag([0.01 0.01 pi/6]);
% % Adding initialize noise
% % P = P + (randn(size(P')) * R)';
% 
% theta_mu = 0;
% theta_sigma = pi/7;
% encoder_mu = 0;
% encoder_sigma = 0.05;
% % system noise 
% R = diag([0.005 0.005 0.0005]);
% 
% occScore = 10;
% occMapValue = 0.5;
% 
% freeScore = 2;
% freeMapValue = -0.2;
% w = ones(1, M) * (1/M);
% 
% figure, 
% h = imagesc(map);
% colormap('gray'); axis equal;
% hold on;
% 
% for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
% 
%     % 1) Propagate the particles 
% 
%     % Get odometry and encoder
%     theta = random('Normal', theta_mu, theta_sigma);
%     encoder = random('Normal', encoder_mu, encoder_sigma);
%     
%     % rot = [ cos(theta)  -sin(theta)  encoder;
%     %         sin(theta)  cos(theta)   0;
%     %         0           0            1];
%     % P = rot * P;
%     % Adding system noise
% R = diag([0.005 0.005 0.0005]);
%     % P = P + (randn(size(P')) * R)';
%     
%     % P = P + mvnrnd([0 0 0], R, M)';
%     P = repmat(myPose(:,j-1), [1, M]) + mvnrnd([0 0 0], R, M)';
% 
%     corrP = [];
%     % 2) Measurement Update 
% 	for k = 1:M    
%     %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)    
%         pose = P(:, k);
%   %       pose(3) = theta + pose(3);
% 	 %    pose(1) = pose(1) + cos(pose(3)) * encoder;
% 	 %    pose(2) = pose(2) + sin(pose(3)) * encoder;
% 		% pose = pose + randn(size(pose)) .* diag(R);
% 		% P(:, k) = pose;
% 		
%     	% compute occLoc for each P_k based on observer range from step j 
%     	realLoc = [ranges(:, j) .* cos(scanAngles + pose(3)), -ranges(:,j) .* sin(scanAngles + pose(3))]' + repmat(pose(1:2), [1 numel(scanAngles)]);
%   		occLoc = ceil(realLoc .* myResolution) + repmat(myOrigin, [1 numel(scanAngles)]);  	
%     	% occLoc = unique(occLoc', 'rows')'; % find unique occ Location to reduce computation load
% 
%     %   2-2) For each particle, calculate the correlation scores of the particles
% %         max(occLoc(1, :))
% %         max(occLoc(2, :))
%         outOfRangeXIdx = find(occLoc(1, :) < 1 | occLoc(1, :) > size(map, 2));
%         outOfRangeYIdx = find(occLoc(2, :) < 1 | occLoc(2, :) > size(map, 1));
%     	outOfRange = unique([outOfRangeXIdx, outOfRangeYIdx]);
%     	occLoc(:, outOfRange) = [];
%         
%     	occInd = sub2ind( size(map), occLoc(2, :), occLoc(1, :));
%     	cellHit = map(occInd) > occMapValue;
%     	cellMiss = map(occInd) < freeMapValue;
%         % corrPoint = sum(cellHit) * occScore + sum(cellMiss) * freeScore;
%     	corrPoint = sum(map(cellHit) * occScore) + sum(map(cellMiss) * freeScore);
%     	corrP = [corrP, corrPoint];		    		
%     end
% 
%     %   2-3) Update the particle weights   
%     corrP(corrP < 0) = 0;
%  	% w = w .* corrP;
%     w = corrP;
%     w = w / sum(w);
%     %   2-4) Choose the best particle to update the pose
%     [~, bestId] = max(w);
%     myPose(:, j) = P(:, bestId);
%     % 3) Resample if the effective number of particles is smaller than a threshold
%     % w2 = w;
%     % w2(w<0) = 0;
%     n_effective = sum(w)^2 / sum(w.^2);
% 
%     if (n_effective < (0.5 *  M))
%     	[new_sample] = resamplingWheel(w);
%     	P = P(:, new_sample);
%     	w = w(new_sample);
%         % w = ones(1, M) * (1/M);
%         w = w / sum(w);
%     end    
%     % 4) Visualize the pose on the map as needed
%     poseLoc = ceil(myPose(1:2, j) .* myResolution) + myOrigin;
%     plot(poseLoc(1), poseLoc(2), 'r.');
%     drawnow
% 
% end
% 
% end
% 
% 
% 
% 
% 

% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of myPoses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% the number of grids for 1 meter.
myResolution = param.resol;
% the origin of the map in pixels
myOrigin = param.origin; 

% The initial myPose is given
myPose(:,1) = param.init_pose;
% You should put the given initial myPose into myPose for j=1, ignoring the j=1 ranges. 
% The myPose(:,1) should be the myPose when ranges(:,j) were measured.

% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 20;                      % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, M]);
score = zeros(1, M);
weight = ones(1, M) * 1/M;

% Build the odometry noise model
mu = zeros(1, 3);
sig = diag([0.01, 0.01, pi/6]);

load testing.mat;

figure;

for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
    % 1) Propagate the particles 
    noises = mvnrnd(mu, sig, M)';
    P = P + noises;
    %R = diag([0.005 0.005 0.0005]);
     %P = (P + (randn(size(P')) * R)');
    
    % 2) Measurement Update 
    %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)    
    rays = ranges(:, j);

    %   2-2) For each particle, calculate the correlation scores of the particles
    for i = 1:M
        x = P(1, i);
        y = P(2, i);
        theta = P(3, i);

        ix_robot = ceil(x * myResolution) + myOrigin(1);
        iy_robot = ceil(y * myResolution) + myOrigin(2);

        x_occ = rays .* cos(scanAngles + theta) + x;
        y_occ = -rays .* sin(scanAngles + theta) + y;
        ix_occ = ceil(x_occ * myResolution) + myOrigin(1);
        iy_occ = ceil(y_occ * myResolution) + myOrigin(2);
        occ=[ix_occ,iy_occ]
        occ = unique(occ', 'rows')';
        ix_occ=occ(:,1);
        iy_occ=occ(:,2);
        % Drop invalid values
        validPos1 = ( (ix_occ < 1) | (iy_occ < 1)); 
        ix_occ(validPos1)=[];
        iy_occ(validPos1)=[];
        validPos2 = (ix_occ > size(map, 2)) | (iy_occ > size(map, 1)) ; 
        ix_occ(validPos2)=[];
        iy_occ(validPos2)=[];
        occ = sub2ind(size(map), iy_occ, ix_occ);

        free = [];
%         for k = 1:numel(occ)
%             [ix_free, iy_free] = bresenham(ix_robot, iy_robot, ix_occ(k), iy_occ(k));  
%             % Drop invalid values
%             validPos = (ix_free > size(map, 2)) | (iy_free > size(map, 1)) | (ix_free < 1) | (iy_free < 1);
%             ix_free(validPos)=[];
%             iy_free(validPos)=[];
% 
%             free = [free; iy_free, ix_free];
%         end

%         free = sub2ind(size(map), free(:, 1), free(:, 2));

        numGrids = numel(occ) + numel(free);
        %numOcc = sum(map(occ) )>0.5;
       % numFree = sum(map(free) < -0.2);
         numOcc = (map(occ) )>0.5;
        numFree = (map(occ) < -0.2);
       
        %score(i) = 10 * numOcc + 1 * numFree + (-5) * (numGrids - numOcc - numFree);
         score(i)=sum(map(numOcc) * 10) + sum(map(numFree) * 2);
    end

    %   2-3) Update the particle weights
    weight =  score;
    weight = weight ./ sum(weight);
 
    %   2-4) Choose the best particle to update the myPose
    [~, bestIndex] = max(weight);
    myPose(:, j) = P(:, bestIndex);
    score(:)=0;
        % 3) Resample if the effective number of particles is smaller than a threshold
    % w2 = w;
    % w2(w<0) = 0;
    n_effective = sum(weight)^2 / sum(weight.^2);

%     if (n_effective < (0.5 *  M))
%     	[new_sample] = resamplingWheel(weight);
%     	P = P(:, new_sample);
%     	w = weight(new_sample);
%         % w = ones(1, M) * (1/M);
%         w = w / sum(w);
%     end   
    
     poseLoc = ceil(myPose(1:2, j) .* myResolution) + myOrigin;
    plot(myPose(1, j), myPose(2, j), 'r.');
    drawnow

end
    % 3) Resample if the effective number of particles is smaller than a threshold
%     num_eff = floor(sum(weight)^2 / sum(weight .^2));
%     if num_eff < M * 0.5
%         newP = zeros(size(myPose, 1), M);
%         newW = zeros(1, M);
%         resampleProbs = unifrnd(0, 1, 1, M);
%         sumOfTopKWeight = cumsum(weight);
%         for i = 1:M
%             idx = find(sumOfTopKWeight >= resampleProbs(i), 1);
%             newP(:, i) = P(:, idx);
%             newM(i) = weight(idx);
%         end
%         P = newP;
%         weight = newM;
%         weight = weight ./ sum(weight);
%     end
% 
%     % 4) Visualize the myPose on the map as needed
%     ix_robot = ceil(x * myResolution) + myOrigin(1);
%     iy_robot = ceil(y * myResolution) + myOrigin(2);
%     ix_robot_true = ceil(pose(1, j) * myResolution) + myOrigin(1);
%     iy_robot_true = ceil(pose(2, j) * myResolution) + myOrigin(2);
% 
%     imagesc(map); hold on;
%     plot(ix_robot, iy_robot, 'r*');
%     for i = 1:M % Plot all particles
%         ix_robot = ceil(P(1, i) * myResolution) + myOrigin(1);
%         iy_robot = ceil(P(2, i) * myResolution) + myOrigin(2);
%         plot(ix_robot, iy_robot, 'y.');
%     end
%     plot(ix_robot_true, iy_robot_true, 'g*');
%     pause(0.001);
% end

%end

% % Robotics: Estimation and Learning 
% % WEEK 4
% % 
% % Complete this function following the instruction. 
% % function myPose = particleLocalization(ranges, scanAngles, map, param)
% % 
% % % Number of poses to calculate
% % N = size(ranges, 2);
% % % Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
% % myPose = zeros(3, N);
% % 
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% % % Map Parameters 
% % % 
% % % % the number of grids for 1 meter.
% %  myResolution = param.resol;
% % % % the origin of the map in pixels
% %  myOrigin = param.origin; 
% % 
% % % The initial pose is given
% % myPose(:,1) = param.init_pose;
% % % You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% % % The pose(:,1) should be the pose when ranges(:,j) were measured.
% % 
% % 
% % 
% % % Decide the number of particles, M.
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % M =20                            % Please decide a reasonable number of M, 
% %                                % based on your experiment using the practice data.
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % Create M number of particles
% % P = repmat(myPose(:,1), [1, M]);
% % F=size(ranges,1);
% % mu=[0,0,0];
% % sigma=diag([0.01 , 0.01 , 0.01] );
% % weights=ones(M,1)/M;
% % scores=ones(M,1)/M;
% %  for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
% % % 
% % %     % 1) Propagate the particles 
% % %
% %     noise=mvnrnd(mu,sigma,M);%
% %     %noise=(noise)'
% %     P=P+noise;
% %     rays=ranges(:,j);
% %     for i=1:M
% %                 x = P(1, i);
% %         y = P(2, i);
% %         theta = P(3, i);
% % 
% %         ix_robot = ceil(x * myResolution) + myOrigin(1);
% %         iy_robot = ceil(y * myResolution) + myOrigin(2);
% % 
% %         x_occ = rays .* cos(scanAngles + theta) + x;
% %         y_occ = -rays .* sin(scanAngles + theta) + y;
% %         ix_occ = ceil(x_occ * myResolution) + myOrigin(1)
% %         iy_occ = ceil(y_occ * myResolution) + myOrigin(2)
% %         X=P(1,i);
% %         Y=P(2,i);
% %         x=ceil(X.*myResolution)+myOrigin(1);
% %         y=ceil(Y.*myResolution)+myOrigin(2);
% %         global_lid_x=ix_occ
% %         global_lid_y=iy_occ
% %         %global_lid_x=ceil((ranges(:,j).*cos(scanAngles+P(3,i))+X)*myResolution)+myOrigin(1);
% %         %global_lid_y=ceil((-ranges(:,j).*sin(scanAngles+P(3,i))+Y)*myResolution)+myOrigin(2);
% %         %pos= (global_lid_x<size(map,1)) & (global_lid_y<size(map,2))  &(global_lid_x>0) & (global_lid_y >0)
% %         global_lid_x=global_lid_x(global_lid_x<size(map,2) & global_lid_x>0&global_lid_y<size(map,1) & global_lid_y>0) ;
% %         global_lid_y=global_lid_y(global_lid_x<size(map,2) & global_lid_x>0&global_lid_y<size(map,1) & global_lid_y>0) ;
% %         
% %         global_lid=sub2ind(size(map),global_lid_y,global_lid_x);
% %         free=[]
% %         freey=[]
% %         for n=1:numel(global_lid)
% %         
% %             [freex1 freey1]=bresenham(x,y,global_lid_x(n),global_lid_y(n));
% %             validPos = (freex1 < size(map, 2)) & (freey1 < size(map, 1)) & (freex1 > 0) & (freey1 > 0);
% %         %P_corr=
% %         %freex=[freex freex1(validPos)];
% %         %freey=[freey freey1(validPos)]
% %          free = [free; freey1, freex1];
% %         end
% %         free=sub2ind(size(map),freex1,freey1);
% %         total=numel(free)+numel(global_lid);
% %         nfree=numel(map(free)<0.5);
% %         noccu=numel(map(global_lid)>0.5);
% %         
% %         score(i) = 10 * noccu + 1 * nfree + (-5) * (total - noccu - nfree);
% %     end
% % %     % 2) Measurement Update 
% %     weights=weights.*score;
% %     
% % %     %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)    
% %     
% %      [~, bestIndex] = max(weights);
% %      myPose(:, j) = P(:, bestIndex);
% % %myPose(j)=P(3,weights==max(weights(:)));
% %  %   weights=weights./sum(weights);
% % %     %   2-2) For each particle, calculate the correlation scores of the particles
% % %
% % %     %   2-3) Update the particle weights         
% % %  
% % %     %   2-4) Choose the best particle to update the pose
% % %     
% % %     % 3) Resample if the effective number of particles is smaller than a threshold
% % % 
% % %     % 4) Visualize the pose on the map as needed
% % %    
% % % 
% %  end
% % 
% % end
% % 
