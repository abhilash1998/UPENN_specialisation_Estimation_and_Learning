
function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)
    
 %   persistent P,R;
  %  if isempty(P)
  %P = 0.1 * eye(4);
%end
   
 %   if isempty(R)
 % R = 0.01 * eye(2);
%end         
        
        
    % Check if the first time running this function
    if previous_t<0
        C=[];
        state = [x, y, 0, 0];
        param.P=0.1 * eye(4);
        param.R=0.1 * eye(2);
        %sigm=diag(0.1 , 0.1 ,0.1 ,0.1);
        %sigo=diag(0.1 , 0.1 );
        predictx = x;
        predicty = y;
        
        return;
    end
    
    dt=t - previous_t;
    A=[1 0 dt 0;0 1 0 dt; 0 0 1 0;0 0 0 1];
    C=[1 0 0 0 ; 0 1 0 0 ];

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    vx = (x - state(1)) / ((t - previous_t));
    vy = (y - state(2)) / ((t - previous_t));
    % Predict 330ms into the future
    %predictx = x + vx * 0.330;
    %predicty = y + vy * 0.330;
    % State is a four dimensional element
    %state = [state(1), state(2), vx, vy];
    sigm=[00.001 0 0.9 0; 0 .001 0 0.9 ; 0 0 1 0; 0 0 0 1];
    sigo=[.01 0;0 0.01];
    %sigm = [dt*dt/4  0    dt/2 0 ;
    %         0    dt*dt/4  0  dt/2;
     %      dt/2     0    1   0;
      %   0     dt/2     0  1];
    
    %measurement error
    %sigo = [0.01, 0;
     %          0, 0.01];
    
    
    Z=[x ,y]';
    P=(A*param.P*A') +sigm; 
    R=C*P*C'+sigo;
    %R=sigo;
    K=P*C'*(inv(R+(C*P*C')));
     
    
    
    state = (A*state') + (K*(Z-C*A*(state')));
    state=state'
    predictx = state(1) + state(3) * 0.330;
    predicty = state(2) + state(4) * 0.330;
    param.P=P-K*C*P;
    
    %state = [state(1), state(2), vx, vy];
    
end
