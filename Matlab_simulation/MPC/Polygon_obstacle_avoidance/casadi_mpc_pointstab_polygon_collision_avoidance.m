%% Clear
clear all
close all
clc

%% Import Casadi
addpath('/Users/reiserbalazs/Documents/MATLAB/casadi-osx-matlabR2015a-v3.5.1')
import casadi.*

%% MPC parameters
%Controller frequency and Prediction horizon
buildtime = tic;
Ts = 0.1;   % sampling time in [s]
N = 25;     % prediction horizon

% TIAGo Robot Params
rob_diameter = 0.54; 
v_max = 1;          % m/s
v_min = -0.0;
w_max = pi/2;       % rad/s
w_min = -w_max;
acc_v_max = 0.4;    % m/ss
acc_w_max = pi/4;   % rad/ss

% Moving Obstacle (MO) params
MO_init = [0.5, 2.0, -pi/4, 0.5, 0.4;    %X, Y, Theta, velocity, radius
           7.0, 9.0, -3*pi/4, 0.6, 0.35;
           7.0, 3.0, 3*pi/4, 0.5, 0.25;
           0.5, 4.0, -pi/2,  0.6, 0.3];
n_MO = size(MO_init, 1);    %number of MOs
n_MOst = 5; 

% Static polygon obstacle params
%SO_polygon(1).point(1).x = {2.0}; SO_polygon(1).point(1).y = {4.0};
SO_polygon(1).point(1).x = {2.0}; SO_polygon(1).point(1).y = {4.0};
SO_polygon(1).point(2).x = {2.5}; SO_polygon(1).point(2).y = {4.7};
SO_polygon(1).point(3).x = {2.1}; SO_polygon(1).point(3).y = {5.0};
SO_polygon(1).point(4).x = {1.6}; SO_polygon(1).point(4).y = {4.5};
 
SO_polygon(2).point(1).x = {2.0}; SO_polygon(2).point(1).y = {0.0};
SO_polygon(2).point(2).x = {2.0}; SO_polygon(2).point(2).y = {0.5};

SO_polygon(3).point(1).x = {2.0}; SO_polygon(3).point(1).y = {0.6};
SO_polygon(3).point(2).x = {2.0}; SO_polygon(3).point(2).y = {1.1};

SO_polygon(4).point(1).x = {2.0}; SO_polygon(4).point(1).y = {1.2};
SO_polygon(4).point(2).x = {2.0}; SO_polygon(4).point(2).y = {1.7};

SO_polygon(5).point(1).x = {2.0}; SO_polygon(5).point(1).y = {1.8};
SO_polygon(5).point(2).x = {2.0}; SO_polygon(5).point(2).y = {2.3};

SO_polygon(6).point(1).x = {2.0}; SO_polygon(6).point(1).y = {2.4};
SO_polygon(6).point(2).x = {2.0}; SO_polygon(6).point(2).y = {3.0};
SO_polygon(6).point(3).x = {2.5}; SO_polygon(6).point(3).y = {3.0};

SO_polygon(7).point(1).x = {2.6}; SO_polygon(7).point(1).y = {3.0};
SO_polygon(7).point(2).x = {3.1}; SO_polygon(7).point(2).y = {3.0};

SO_polygon(8).point(1).x = {4.0}; SO_polygon(8).point(1).y = {4.0};
SO_polygon(8).point(2).x = {4.2}; SO_polygon(8).point(2).y = {4.3};
SO_polygon(8).point(3).x = {4.5}; SO_polygon(8).point(3).y = {4.7};
SO_polygon(8).point(4).x = {4.3}; SO_polygon(8).point(4).y = {4.9};
SO_polygon(8).point(5).x = {4.0}; SO_polygon(8).point(5).y = {4.9};
SO_polygon(8).point(6).x = {3.8}; SO_polygon(8).point(6).y = {4.4};

SO_polygon(9).point(1).x = {2.0}; SO_polygon(9).point(1).y = {6.0};
SO_polygon(9).point(2).x = {2.5}; SO_polygon(9).point(2).y = {6.0};
SO_polygon(9).point(3).x = {2.5}; SO_polygon(9).point(3).y = {6.5};
SO_polygon(9).point(4).x = {2.0}; SO_polygon(9).point(4).y = {6.5};

SO_polygon(10).point(1).x = {0.0}; SO_polygon(10).point(1).y = {1.0};
SO_polygon(10).point(2).x = {0.0}; SO_polygon(10).point(2).y = {1.5};

SO_polygon(11).point(1).x = {0.0}; SO_polygon(11).point(1).y = {1.5};
SO_polygon(11).point(2).x = {0.0}; SO_polygon(11).point(2).y = {2.0};

SO_polygon(12).point(1).x = {0.0}; SO_polygon(12).point(1).y = {2.0};
SO_polygon(12).point(2).x = {0.0}; SO_polygon(12).point(2).y = {2.5};

SO_polygon(13).point(1).x = {0.0}; SO_polygon(13).point(1).y = {2.5}; 
SO_polygon(13).point(2).x = {0.0}; SO_polygon(13).point(2).y = {3.0};

SO_polygon(14).point(1).x = {0.0}; SO_polygon(14).point(1).y = {3.0};
SO_polygon(14).point(2).x = {0.0}; SO_polygon(14).point(2).y = {3.5};

SO_polygon(15).point(1).x = {0.0}; SO_polygon(15).point(1).y = {3.5};
SO_polygon(15).point(2).x = {0.0}; SO_polygon(15).point(2).y = {4.0};

SO_polygon(16).point(1).x = {0.0}; SO_polygon(16).point(1).y = {4.0};
SO_polygon(16).point(2).x = {0.0}; SO_polygon(16).point(2).y = {4.5};

SO_polygon(17).point(1).x = {0.0}; SO_polygon(17).point(1).y = {4.5};
SO_polygon(17).point(2).x = {0.0}; SO_polygon(17).point(2).y = {5.0};

SO_polygon(18).point(1).x = {0.0}; SO_polygon(18).point(1).y = {1.0};
SO_polygon(18).point(2).x = {-0.5}; SO_polygon(18).point(2).y = {1.0};

SO_polygon(19).point(1).x = {-0.5}; SO_polygon(19).point(1).y = {1.0};
SO_polygon(19).point(2).x = {-1.0}; SO_polygon(19).point(2).y = {1.0};

SO_polygon(20).point(1).x = {-1.0}; SO_polygon(20).point(1).y = {1.0};
SO_polygon(20).point(2).x = {-1.5}; SO_polygon(20).point(2).y = {1.0};

SO_polygon(21).point(1).x = {-1.5}; SO_polygon(21).point(1).y = {0.0};
SO_polygon(21).point(2).x = {-1.0}; SO_polygon(21).point(2).y = {0.3};
SO_polygon(21).point(3).x = {-1.0}; SO_polygon(21).point(3).y = {0.7};

n_SO = 15;  %Number of considered polygons nearby
[SO_matrix, SO_dims] = SO_struct2Matrix(SO_polygon);

%% State declaration
% System states
x = SX.sym('x');
y = SX.sym('y');
theta = SX.sym('theta');
states = [x;y;theta];
n_states = length(states);

% Control system
v = SX.sym('v');
omega = SX.sym('omega');
controls = [v;omega];
n_controls = length(controls);

rhs = [v*cos(theta);v*sin(theta);omega];     %right hand side of the system

%% System setup for Casadi
mapping_func = Function('f',{states,controls},{rhs});   % nonlinear mapping function f(x,u)

% Declear empty sys matrices
U = SX.sym('U',n_controls,N+1);                 % Decision variables (controls)

% Parameter Matrix
P = SX.sym('P',n_states ...                     % Initial states (x0)
               + n_states ...                   % Goal position
               + n_MO*(N+1)*n_MOst ...          % MO states in each prediction
               + n_SO*6 ...                    % closest n_SO obstacle (added as 3 point polygons)
               + n_SO);                         % Real dimensions of SO-s
X = SX.sym('X',n_states,(N+1));                 % Prediction matrix.

%% Objective and Constrains
% Weighing matrices (states)
Q = zeros(3,3);
Q(1,1) = 1;     % x
Q(2,2) = 5;     % y
Q(3,3) = 0.1;   % th

% Weighing matrices (controls)
R = zeros(4,4);
R(1,1) = 5;   % v
R(2,2) = 0.05;  % omega
R(3,3) = 50;    % v accelaration
R(4,4) = 5;    %  omega acceleration

obj = 0;           % objective (Q and R)
const_vect = [];   % constraints vector

% Lifting for Multiple shooting 
st = X(:,1);            % initial state
const_vect = [const_vect; st-P(1:3)];    % initial condition constraints

%% Objective function for multiple shooting and Trajectory tracking
for k = 1:N
    st = X(:,k); 
    cont = U(:,k);
    cont_next = U(:,k+1);
    
    % Objective function
    obj = obj + (st-P(4:6))'*Q*(st-P(4:6)) ...
              + (cont)'*R(1:2,1:2)*(cont) ...
              + (cont-cont_next)'*R(3:4,3:4)*(cont-cont_next);
    st_next = X(:, k+1);
    mapping_func_value = mapping_func(st, cont);
    st_next_euler = st + (Ts*mapping_func_value);          % Euler decritization
    const_vect = [const_vect; st_next-st_next_euler];      % compute constraints
end

%% Constraints on MO
i_pos = n_states + n_states + 1;
for k = 1:N+1      
    for i = 1:n_MO
        const_vect = [const_vect ; -sqrt((X(1,k)-P(i_pos))^2+(X(2,k)-P(i_pos+1))^2) + (rob_diameter/2 + P(i_pos+4))];
        i_pos = i_pos+5;
    end
end

%% Constraints on Static Obstacles
%At dis point we already consider only the nearest n_SO number of Static
%obstacles and converted all polygon obstacles into the closest point to
%the robot position
PolyXY = P(i_pos:i_pos+n_SO*6-1);
i_pos = i_pos+n_SO*6;
PolyDims = P(i_pos:i_pos+n_SO-1);
disp('Fill up SO parameter matrix');
for k = 1:N+1
    k_pos = 1;
    for i = 1:n_SO
        const_vect = if_else(PolyDims(i)<3, ...
                     if_else(PolyDims(i)<2, ...
                     [const_vect; -sqrt((X(1,k)-PolyXY(k_pos))^2+(X(2,k)-PolyXY(k_pos+1))^2) + rob_diameter/2], ...
                     IfPolyDimsEqTwo(const_vect, PolyXY(k_pos:k_pos+3), [X(1,k), X(2,k)], rob_diameter)), ...
                     IfPoly3(const_vect, PolyXY(k_pos:k_pos+5), [X(1,k), X(2,k)], rob_diameter));
        k_pos = k_pos+6;
    end
    fprintf('.');
end

%% Nonlinear Programming setup
% Optimization variables (Reshape it into a huge vector)
OPT_variables = [reshape(X,3*(N+1),1);reshape(U,2*(N+1),1)]

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', const_vect, 'p', P);

%IPOPT optimization setup
opts = struct;
opts.ipopt.max_iter = 200;
opts.ipopt.print_level =0; %0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

%% Setup constraint and states Upper and Lower bound
args = struct;

% Equality constraints on states
i_pos = n_states*(N+1);
args.lbg(1:i_pos) = 0;
args.ubg(1:i_pos) = 0;

% MO constraints
args.lbg(i_pos+1:i_pos+n_MO*(N+1)) = -inf;
args.ubg(i_pos+1:i_pos+n_MO*(N+1)) = 0;

% SO constraints
i_pos = i_pos+n_MO*(N+1);
args.lbg(i_pos+1:i_pos+n_SO*(N+1)) = -inf;
args.ubg(i_pos+1:i_pos+n_SO*(N+1)) = 0;

% Constraints on states
i_pos = n_states*(N+1);
args.lbx(1:n_states:i_pos,1) = -inf;      %state x lower bound
args.ubx(1:n_states:i_pos,1) = inf;      %state x upper bound
args.lbx(2:n_states:i_pos,1) = -inf;      %state y lower bound
args.ubx(2:n_states:i_pos,1) = inf;      %state y upper bound
args.lbx(3:n_states:i_pos,1) = -inf;   %state th lower bound
args.ubx(3:n_states:i_pos,1) = inf;    %state th upper bound

% Constraints on control variables
args.lbx(i_pos+1:n_controls:i_pos+n_controls*(N+1),1) = v_min;
args.ubx(i_pos+1:n_controls:i_pos+n_controls*(N+1),1) = v_max; 
args.lbx(i_pos+2:n_controls:i_pos+n_controls*(N+1),1) = w_min;
args.ubx(i_pos+2:n_controls:i_pos+n_controls*(N+1),1) = w_max;

%% Simulation setup
t0 = 0;
x0 = [0 ; 0 ; 0.0];             % initial states
u0 = zeros(N+1,2);              % initial control inputs
x_st_0 = repmat(x0,1,N+1)';     % initial states decision variables
t(1) = t0;
x_ol(:,1) = x0;                 % Initial predictied predicted states (open loop)
sim_time = 15;                  % Maximum simulation time
goal_tolerance = 0.02;          % Goal tolerance in [m]

%% Trajectory reference
% Strait line
x_start = [0, 0, 0.0];
x_goal = [6, 6, 0.0];
x_ref = x_goal;          % Intial postion for reference trajectory

%% Start MPC
mpc_i = 0;    % Couter for the MPC loop
x_cl = [];    % Store predicted states in the closed loop
u_cl = [];      % Store control inputs in the closed loop
o_cl = [];    % Store obstacle position in closed loop

build_time = toc(buildtime)
runtime = tic;
while(norm((x0-x_goal'),2) > goal_tolerance && mpc_i < sim_time / Ts)
    mpctime = tic;        % start of mpc frequency measurement
    args.p(1:3) = x0;     % initial states
    args.p(4:6) = x_ref;  % goal position
    
    %% MO constraint
    i_pos = 7;
    for k = 1:N+1
        for i = 1:n_MO
            t_predicted = (mpc_i + k-1) * Ts;     % Time at predicted state
            
            %MO X and Y
            obs_x = MO_init(i,1) + t_predicted*MO_init(i,4)*cos(MO_init(i,3));
            obs_y = MO_init(i,2) + t_predicted*MO_init(i,4)*sin(MO_init(i,3));
            args.p(i_pos:i_pos+1) = [obs_x, obs_y];
            o_cl(i,k,1:2,mpc_i+1) = [obs_x, obs_y];
                
            % MO orientation, velocity, radius
            args.p(i_pos+2:i_pos+4) = [MO_init(i,3), MO_init(i,4), MO_init(i,5)];
            o_cl(i,k,3:5,mpc_i+1) = [MO_init(i,3), MO_init(i,4), MO_init(i,5)];
            i_pos = i_pos+5;
        end
    end

    %% SO constraints
    % Calculate polygon centroids, and radiuses
    cent_rad_list = [];
    k_pos = 1;
    for k = 1:size(SO_polygon, 2)
        poly_x = SO_matrix(k_pos:k_pos+SO_dims(k)-1,1);
        poly_y = SO_matrix(k_pos:k_pos+SO_dims(k)-1,2);
        cent_rad_list = [cent_rad_list; CalculatePolygonCentroid(poly_x, poly_y)];
        k_pos = k_pos+SO_dims(k);
    end
    
    % Find the index of the closest n_SO number of obstacle
    closest_SO_index = GetListOfClosestNObstacleIndex(cent_rad_list, x0, n_SO);
    
    k_pos = i_pos+n_SO*6; % index for obstacle size
    
    %SO_cl_index(mpc_i+1, 1:n_SO) = closest_SO_index(1, 1:n_SO);
    
    for k = 1:n_SO
        index = closest_SO_index(k);  %Get the next index
        obs_poses = GetClosest3PointOfPoly(SO_polygon(index), x0);
        obs_size = size(SO_polygon(index).point, 2); %Get the size of the obstacle
        SO_cl_obs_poses(k, 1:6, mpc_i+1) =  obs_poses(1, 1:6);
        SO_cl_obs_sizes(mpc_i+1, k) = obs_size(1);
        args.p(i_pos:i_pos+5) = obs_poses; % Each SO occupies 3 point pair (x,y)
        args.p(k_pos) = obs_size; % Real size of obstacle
        i_pos = i_pos+6;
        k_pos = k_pos+1;
    end
    
    
    %% Set Optimization parameters
    % initial value of the optimization variables
    args.x0 = [reshape(x_st_0',3*(N+1),1);reshape(u0',2*(N+1),1)];   
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);          
    u = reshape(full(sol.x(3*(N+1)+1:end))',2,N+1)'; % Control inputs from solution
    u_cl= [u_cl ; u(1,:)];  % Store first control from each prediction
    x_cl(:,1:3,mpc_i+1)= reshape(full(sol.x(1:3*(N+1)))',3,N+1)';  % Store all state predictions
    
    % Shift the calculated states into the next initiation
    t(mpc_i+1) = t0;                                     % Store time
    [t0, x0, u0] = shift(Ts, t0, x0, u,mapping_func);    % Update x0 and u0
    x_ol(:,mpc_i+2) = x0;                                % Store calculated states
    x_st_0 = reshape(full(sol.x(1:3*(N+1)))',3,N+1)';    % get solution trajectory
    
    % Shift trajectory to initialize the next step
    x_st_0 = [x_st_0(2:end,:); x_st_0(end,:)];
    
    mpc_time = toc(mpctime);
    fprintf('MPC iteration = %d, MPC calculation time = %4.3f \n',mpc_i, mpc_time);
    mpc_i = mpc_i + 1;
end

run_time = toc(runtime)
position_error = norm((x0-x_goal'),2)
average_mpc_cl_time = run_time/(mpc_i+1)

%x_ol
%u_cl
xyaxis = [-2 7 -.2 7]
Simulate_MPC_with_polygon (x_ol,x_cl,o_cl,SO_cl_obs_poses,SO_cl_obs_sizes,SO_polygon,x_ref,N,rob_diameter,xyaxis)
Plot_Control_Input (t, u_cl, v_min, v_max, w_min, w_max)
