

function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
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

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.



% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 200;   						% Please decide a reasonable number of M, 
                               	% based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
score = zeros(1, M);


% R = chol(Sigma);
% R = diag([0.01 0.01 pi/6]);
% Adding initialize noise
% P = P + (randn(size(P')) * R)';
P = repmat(myPose(:,1), [1, M]);
%score = zeros(1, M);
weight = ones(1, M) * 1/M;

% Build the odometry noise model
mu = zeros(1, 3);
sig = diag([0.01 0.01 0.0005]);
P = repmat(myPose(:,1), [1, M]);

%freeS = 2;
%freeMaptrans = -0.2;
weight = ones(1, M) * (1/M);

figure, 
h = imagesc(map);
colormap('gray'); axis equal;
hold on;

for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).

    % 1) Propagate the particles 


    % Adding system noise
%R = diag([0.005 0.005 0.0005]);
    
    
    
    P = repmat(myPose(:,j-1), [1, M]) + mvnrnd(mu, sig, M)';

    %corrP = [];
    % 2) Measurement Update 
	for i = 1:M    
    %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)    
        pose = P(:, i);%occ
  %       pose(3) = theta + pose(3);
	 %    pose(1) = pose(1) + cos(pose(3)) * encoder;
	 %    pose(2) = pose(2) + sin(pose(3)) * encoder;
		% pose = pose + randn(size(pose)) .* diag(R);
		% P(:, i) = pose;
		theta=pose(3);
        x=pose(1);
        y=pose(2);
        rays=ranges(:,j);
         x_occ = rays .* cos(scanAngles + theta) + repmat(x, [1 numel(scanAngles)])';
        y_occ = -rays .* sin(scanAngles + theta) + repmat(y, [1 numel(scanAngles)])';
        ix_occ = ceil(x_occ.* myResolution) + repmat(myOrigin(1), [1 numel(scanAngles)])';
        iy_occ = ceil(y_occ.* myResolution) + repmat(myOrigin(2), [1 numel(scanAngles)])';
        occ=[ix_occ iy_occ];
        
        
    	% compute occ for each P_i based on observer range from step j 
    	%realLoc = [ranges(:, j) .* cos(scanAngles + pose(3)), -ranges(:,j) .* sin(scanAngles + pose(3))]' + repmat(pose(1:2), [1 numel(scanAngles)]);
  		%occ = ceil(realLoc .* myResolution) + repmat(myOrigin, [1 numel(scanAngles)]);  	
    	% occ = unique(occ', 'rows')'; % find unique occ Location to reduce computation load

    %   2-2) For each particle, calculate the correlation scores of the particles
%         max(occ(1, :))
%         max(occ(2, :))
%         outOfRangeXIdx = find(occ(1, :) < 1 | occ(1, :) > size(map, 2));
%         outOfRangeYIdx = find(occ(2, :) < 1 | occ(2, :) > size(map, 1));
%     	outOfRange = unique([outOfRangeXIdx, outOfRangeYIdx]);
%     	occ(:, outOfRange) = [];
%         
        validPos1 = find( ( occ(:,1 )< 1) | (occ(:,1 ) >size(map, 2))); 

        validPos2 = find(occ(:,2 )> size(map, 1) |(occ(:,2 ) < 1) ) ;
        
        occy = unique([validPos1,validPos2] );
        occ(occy,:)=[];
        
         ix_occ=occ(:,1);
        iy_occ=occ(:,2);
        
        occ = sub2ind(size(map), iy_occ, ix_occ);
        occInd=occ;
        occ=occ';
         numOcc = (map(occ) )>0.5;
         numFree = (map(occ) < -0.2);
        i=i;
%        score(i) = 10 * numOcc + 1 * numFree + (-5) * (numGrids - numOcc - numFree);
         score(i)=sum(numOcc) * 10 + sum(numFree) * 2;
    	%occInd = sub2ind( size(map), occ(2, :), occ(1, :));
%     	cellHit = map(occInd) > occMapValue;
%     	cellMiss = map(occInd) < freeMapValue;
%         % corrPoint = sum(cellHit) * occScore + sum(cellMiss) * freeScore;
%     	corrPoint = sum(map(cellHit) * occScore) + sum(map(cellMiss) * freeScore);
%     	corrP = [corrP, corrPoint];		    		
    end
 score(score<0)=0;
    %   2-3) Update the particle weights
    weight = score;
    weight = weight ./ sum(weight);
 
    %   2-4) Choose the best particle to update the myPose
    [~, bestIndex] = max(weight);
    %   2-3) Update the particle weights   
%     corrP(corrP < 0) = 0;
%  	% w = w .* corrP;
%     w = corrP;
%     w = w / sum(w);
%     %   2-4) Choose the best particle to update the pose
    %[~, bestId] = max(w);
    myPose(:, j) = P(:, bestIndex);
    % 3) Resample if the effective number of particles is smaller than a threshold


%     if (n_effective < (0.5 *  M))
%     	[new_sample] = resamplingWheel(w);
%     	P = P(:, new_sample);
%     	w = w(new_sample);
%         % w = ones(1, M) * (1/M);
%         w = w / sum(w);
%     end    
    % 4) Visualize the pose on the map as needed
    poseLoc = ceil(myPose(1:2, j) .* myResolution) + myOrigin;
    plot(poseLoc(1), poseLoc(2), 'r.');
    drawnow

end

end




