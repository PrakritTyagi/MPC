clc; close all; clear all;
% heights were changed from 45 to 47
% FOV orientation should change with direction off UAV

dt = 0.2; %discrete time step

%% Camera Parameters //////Should be same in ' ex01StepPlotFunction.m '////// 
VFOV_deg = 94.4; % vertical field of view in degree
HFOV_deg = 122.6; % horizontal field of view in degree

VFOV_rad = VFOV_deg*pi/180;
HFOV_rad = HFOV_deg*pi/180;
%% Obstacle properties //////Should be same in ' ex01StepPlotFunction.m '//////
a0 = 140; % abscissa of Obstacle0
b0 = 65;  % ordinate of Obstacle0
h0 = 47;  % height of Obstacle0
a1 = 260; % abscissa of Obstacle1
b1 = 100; % ordinate of Obstacle2 
h1 = 47;  % height of obstacle1
a2 = 220; % abscissa of Obstacle2
b2 = 160;  % ordinate of Obstacle2
h2 = 47;  % height of obstacle2

r = 20;   % radius of obstacle / Equal for each


%% Unicycle Model
% 5 DOF UAV model
% 3 DOF target Model
sys = ICtSystem(...
    'StateEquation', @(t,x,u,varargin) [
    u(1)*cos(x(4))*cos(x(5)); % Xcoordinate_dot
    u(1)*sin(x(4))*cos(x(5)); % Ycoordinate_dot
    u(1)*sin(x(5)); % Zcoordinate_dot
    u(2) % Yaw angle rate rad/sec
    u(3) % Pitch angle rate rad/sec
    12*cos(x(8));%x(6) Xcoordinate_dot
    12*sin(x(8));%x(7) Ycoordinate_dot
    0.03-0.3*floor((16-t)/1000)+0.3*floor((17.45-t)/1000)],... 
    'nx',8,'nu',3 ...
);
% initial conditions
sys.initialCondition = {[0;-5;40;0;0;1;10;0]};

%% Dependence of altitude of UAV on FOV
a = @(x) x(3)*tan(VFOV_rad/2);
b = @(x) x(3)*tan(HFOV_rad/2);

%FOV equation modeled as an ellipse
A = @(x) (cos(x(4)))^2/(a(x)^2) + (sin(x(4)))^2/(b(x)^2);
B = @(x) 2*cos(x(4))*sin(x(4))*( (1/a(x)^2) - (1/b(x)^2) );
C = @(x) (sin(x(4)))^2/(a(x)^2) + (cos(x(4)))^2/(b(x)^2);

ellipse = @(x) A(x)*(x(6) - x(1))^2 + B(x)*(x(6) - x(1))*(x(7) - x(2)) + C(x)*(x(7) - x(2))^2 - 1;

%% cost functions for relative distance between UAV and Target
cost = @(x)  (5*(x(1)-x(6))^2 + 5*(x(2)-x(7))^2 +6*(x(3)-45)^2); % final cost function 0

%% obstacle cost function for avoiding collision

obs  = @(x) (r^2 - (x(1)-a0)^2 - (x(2)-b0)^2  );
obs1 = @(x) (r^2 - (x(1)-a1)^2 - (x(2)-b1)^2  );
obs2 = @(x) (r^2 - (x(1)-a2)^2 - (x(2)-b2)^2  );
%check  = @(x) ceil(obs(x)/10000);
%check1 = @(x) ceil(obs1(x)/10000);
%check2 = @(x) ceil(obs2(x)/10000);
%costobs  = @(x) check(x)*(max(0,h0-x(3)))^2 + 5*max(0,obs(x));
%costobs1 = @(x) check1(x)*(max(0,h1-x(3)))^2 + 5*max(0,obs1(x));
%costobs2 = @(x) check2(x)*(max(0,h2-x(3)))^2 + 5*max(0,obs2(x));
%max(0,obs(x))*(max(0,h0-x(3)))^2 + 
costobs  = @(x) max(0,obs(x))*(max(0,h0-x(3)))^2 + 5*max(0,obs(x));
costobs1 = @(x) max(0,obs1(x))*(max(0,h1-x(3)))^2 + 5*max(0,obs1(x));
costobs2 = @(x) max(0,obs2(x))*(max(0,h2-x(3)))^2 + 5*max(0,obs2(x));
cost1 = @(x) costobs(x) + costobs1(x) + costobs2(x); % final cost function 1 

%% cost function for FOV
% Equation of ellipse is  above
cost2 = @(x) max(0,ellipse(x)); % final cost function 2

%% cost function for maximum visiblity of target under hinderance from obstacle

% Target[x(6,x(7),0]   UAV[x(1),x(2),x(3)]    Obstacle_ref[ax,bx,hx]
%slopelos  = @(x) atan( x(3) /sqrt( (x(6)-x(1))^2 + (x(7)-x(2))^2 )); %Slope of LOS Between UAV and Target in 3D
slopeRef0 = @(x)  (h0/(sqrt( (x(6)-a0)^2 + (x(7)-b0)^2 )-r)) - (x(3)/(sqrt( (x(6)-x(1))^2 + (x(7)-x(2))^2 ))); % Slope of LOS between obstacle_0 Top and target in 3D
slopeRef1 = @(x)  (h1/(sqrt( (x(6)-a1)^2 + (x(7)-b1)^2 )-r)) - (x(3)/(sqrt( (x(6)-x(1))^2 + (x(7)-x(2))^2 ))); % Slope of LOS between obstacle_1 Top and target in 3D
slopeRef2 = @(x)  (h2/(sqrt( (x(6)-a2)^2 + (x(7)-b2)^2 )-r)) - (x(3)/(sqrt( (x(6)-x(1))^2 + (x(7)-x(2))^2 )));% Slope of LOS between obstacle_2 Top and target in 3D

slope = @(x) (x(2)-x(7))/(x(1)-x(6)); % slope of ray(x-yplane) from target to UAV in 2D
center0_to_LOS = @(x) ((abs(slope(x)*a0 - 1*b0 + x(7) - x(6)*slope(x))/sqrt(slope(x)^2 + (-1)^2))-r); % distance between line LOS and center of obstacle0
center1_to_LOS = @(x) ((abs(slope(x)*a1 - 1*b1 + x(7) - x(6)*slope(x))/sqrt(slope(x)^2 + (-1)^2))-r);
center2_to_LOS = @(x) ((abs(slope(x)*a2 - 1*b2 + x(7) - x(6)*slope(x))/sqrt(slope(x)^2 + (-1)^2))-r);

logic_TargetInVision = @(x) floor(ellipse(x)/10000)*-1; % logic to check if Target is visible
logic_LosIntersectObstacle0 = @(x) floor((center0_to_LOS(x))/10000)*-1;  %logic to check if Los intersects obstacle0
logic_LosIntersectObstacle1 = @(x) floor((center1_to_LOS(x))/10000)*-1;
logic_LosIntersectObstacle2 = @(x) floor((center2_to_LOS(x))/10000)*-1;
costlos0 = @(x) logic_TargetInVision(x)*logic_LosIntersectObstacle0(x)*max(0,slopeRef0(x)); % cost to increase altitude if visiblity obstructed
costlos1 = @(x) logic_TargetInVision(x)*logic_LosIntersectObstacle1(x)*max(0,slopeRef1(x));
costlos2 = @(x) logic_TargetInVision(x)*logic_LosIntersectObstacle2(x)*max(0,slopeRef2(x));
cost3 = @(x) costlos0(x) + costlos1(x) + costlos2(x); % aggregation of costs  final cost function 3

%cost4 = @(x) max(norm(x(4))-0.521599,0);

mpcOp = ICtMpcOp( ...
    'System'               , sys,...
    'HorizonLength'        , 15*dt,...
    'StageConstraints'     , BoxSet([15;-0.174533;-0.174533],9:11,[25;0.174533;0.174533],9:11,11),... % 0.0873 on the variable z=[x;u];
    'StageCost'            , @(t,x,u,varargin) 5*cost(x)+30*cost1(x)+cost2(x)+cost3(x)...
    );

dtMpcOp      = DiscretizedMpcOp(mpcOp,dt);

dtRealSystem = DiscretizedSystem(sys,dt);

dtRealSystem.controller = MpcController(...
    'MpcOp'       , dtMpcOp ,...
    'MpcOpSolver' , FminconMpcOpSolver('MpcOp', dtMpcOp,'UseSymbolicEvaluation',0) ...
    );
%e = @(t,x)
va = VirtualArena(dtRealSystem,...
    'StoppingCriteria'  , @(t,sysList)t>40/dt,...
    'PlottingStep'      , 1/dt, ...
    'StepPlotFunction'  , @ex01StepPlotFunction ...
    );

log = va.run();
