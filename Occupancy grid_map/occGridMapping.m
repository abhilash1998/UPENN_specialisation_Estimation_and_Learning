% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% % the number of grids for 1 meter.
 myResol = param.resol;
% % the initial map size in pixels
 myMap = zeros(param.size);
% % the origin of the map in pixels
 myorigin = param.origin; 
% 
% % 4. Log-odd parameters 
 lo_occ = param.lo_occ;
 lo_free = param.lo_free; 
 lo_max = param.lo_max;
 lo_min = param.lo_min;
%origin=[0 ,0];
%grid=zeros(100,100);
m=size(pose,2);
n_scans = size(scanAngles,1);
for i =1:m
    res=10;
    D=[ranges(:,i)];
    theta=pose(3,i);
    x=pose(1,i);
    y=pose(2,i);
    dist=[D.*cos(theta+scanAngles) -D.*(sin(theta+scanAngles)  )]'+repmat([x; y],1,n_scans);

    current_pos=([ceil(myResol*x) ; ceil(myResol*y)])+ myorigin;
    pos = ceil(myResol*dist) + repmat(myorigin,1,n_scans);
    %pos=([ceil(dist(1).*myResol); ceil(dist(2).*myResol)])+repmat(myorigin,1,n_scans);
    pos_ind=sub2ind(size(myMap),pos(2,:),pos(1,:));
    % N = size(pose,2);
    % for j = 1:N % for each time,
    myMap(pos_ind)=myMap(pos_ind)+lo_occ;
for j=1:n_scans
    

    [freex , freey]=bresenham(current_pos(1),current_pos(2),pos(1,j) , pos(2,j));
    %endsub=sub2ind(size(myMap),endpoint(2),endpoint(1));
    free=sub2ind(size(myMap),freey,freex);
    
    
          
    

    myMap(free)=myMap(free)-lo_free;
    

    end
    % 
    %       
    %     % Find grids hit by the rays (in the gird map coordinate)
        myMap(myMap<lo_min) = lo_min; % Prevent the map from becoming too certain
    myMap(myMap>lo_max) = lo_max;

    %   
    % 
    %     % Find occupied-measurement cells and free-measurement cells
    %    
    % 
    %     % Update the log-odds
    %   
    % 
    %     % Saturate the log-odd values
    %     
    % 
    %     % Visualize the map as needed
    %    
%
    end
 end



