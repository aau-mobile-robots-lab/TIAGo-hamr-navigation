%% Clear
clear all
close all
clc

%% Import Casadi
addpath('/Users/reiserbalazs/Documents/MATLAB/casadi-osx-matlabR2015a-v3.5.1')
import casadi.*

%% MPC parameters
%Controller frequency and Prediction horizon
Ts = 0.1;   % sampling time in [s]
N = 20;     % prediction horizon

% TIAGo Robot Params
rob_diameter = 0.54; 
v_max = 1;          % m/s
v_min = -v_max;
w_max = pi/4;       % rad/s
w_min = -w_max;

%% State declaration
% System states
x = SX.sym('x');
y = SX.sym('y');
theta = SX.sym('theta');
states = [x;y;theta];
n_states = length(states);

% Control system
v_traj = SX.sym('v');
omega = SX.sym('omega');
controls = [v_traj;omega];
n_controls = length(controls);

rhs = [v_traj*cos(theta);v_traj*sin(theta);omega];     %right hand side of the system

%% System setup for Casadi
mapping_func = Function('f',{states,controls},{rhs});   % nonlinear mapping function f(x,u)
%mapping_func.print_dimensions                          % shows the number of inputs and outputs

% Declear empty sys matrices
U = SX.sym('U',n_controls,N);                % Decision variables (controls)
%P = SX.sym('P',n_states + n_states);         % Parameters which include the initial state and the reference state of the robot
P = SX.sym('P',n_states + N*(n_states+n_controls));
% first n_states are the initial contidition
%the second part is responsible for the time varying x_reference
X = SX.sym('X',n_states,(N+1));              % Prediction matrix.

%% Symbolic solution for single shooting
% Fill up U, P, X matrices
%   X(:,1) = P(1:n_states);                  % Add initial states from P to X
% In lifted (multiple shooting), we not fill X up like this.
%for k = 1:N                                  
%    st = X(:,k);    % previous state
%    con = U(:,k);   % control for next state
%    f_value = system_function(st,con) ;
%    st_next = st + (Ts*f_value); % Euler discretization
%    X(:,k+1) = st_next;
%end

% Discretized cost funtion
% this function to get the optimal trajectory knowing the optimal solution
%optimal_solution = Function('ff',{U,P},{X});

%% Objective and Constrains
% weighing matrices (states)
Q = zeros(3,3);
Q(1,1) = 1; % x
Q(2,2) = 5; % y
Q(3,3) = 0.1; %th
% weighing matrices (controls)
R = zeros(2,2);
R(1,1) = 0.5; % v
R(2,2) = 0.05; % w

obj = 0;                % objective (Q and R)
const_vect = [];        % constraints vector

% Lift 
st = X(:,1);            % initial state
const_vect = [const_vect; st-P(1:3)];    % initial condition constraints

%Calculate the objective function and constraints
for k = 1:N
    st = X(:,k);
    cont = U(:,k);
    %obj = obj + (st-P(4:6))'*Q*(st-P(4:6)) + cont'*R*cont; % calculate objective function
    obj = obj + (st-P((5*k-1):(5*k+1)))'*Q*(st-P((5*k-1):(5*k+1))) + (cont-P((5*k+2):(5*k+3)))'*R*(cont-P((5*k+2):(5*k+3)));
    st_next = X(:, k+1);
    mapping_func_value = mapping_func(st, cont);
    st_next_euler = st + (Ts*mapping_func_value);          % Euler decritization
    const_vect = [const_vect; st_next-st_next_euler];      % compute constraints
end

%% Single shooting objective and constraints
% Calculate objective function
%for k=1:N
%    st = X(:,k);
%    cont = U(:,k);
%    obj = obj+(st-P(4:6))'*Q*(st-P(4:6)) + cont'*R*cont; % calculate objective funtion
%end

% Create constraints
% Firstly inticiate them by the the positions in each prediction (including initial states)
%for k = 1:N+1           % box constraints due to the map margins (planning area in X and Y)
%    const_vect = [const_vect ; X(1,k)];   %state x
%    const_vect = [const_vect ; X(2,k)];   %state y
%end

%% Nonlinear Programming setup
%OPT_variables = reshape(U,2*N,1);       %Single shooting, create a vector from U [v,w,v,w,...]
OPT_variables = [reshape(X,3*(N+1),1);reshape(U,2*N,1)];

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', const_vect, 'p', P);
%nlp_prob

opts = struct;
opts.ipopt.max_iter = 200;
opts.ipopt.print_level =0; %0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);  %Ipopt optimization

%% Setup Constraints
args = struct;

% Equality constraints
args.lbg(1:3*(N+1)) = 0;
args.ubg(1:3*(N+1)) = 0;

% Constraints on states
args.lbx(1:3:3*(N+1),1) = -5;     %state x lower bound
args.ubx(1:3:3*(N+1),1) = 5;      %state x upper bound
args.lbx(2:3:3*(N+1),1) = -5;     %state y lower bound
args.ubx(2:3:3*(N+1),1) = 5;      %state y upper bound
args.lbx(3:3:3*(N+1),1) = -inf;   %state th lower bound
args.ubx(3:3:3*(N+1),1) = inf;    %state th upper bound

%args.lbg = -5;  % lower bound of the states x and y [m]
%args.ubg = 5;   % upper bound of the states x and y [m]

% Input constraints on U
args.lbx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_min;
args.ubx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_max; 
args.lbx(3*(N+1)+2:2:3*(N+1)+2*N,1) = w_min;
args.ubx(3*(N+1)+2:2:3*(N+1)+2*N,1) = w_max;

%% Simulation setup
t0 = 0;
x0 = [0 ; 0 ; 0.0];             % initial condition - starting position and orientation
u0 = zeros(N,2); 
%x_goal = [4 ; 4 ; 0.0];         % Reference position and orientation

x_st_0 = repmat(x0,1,N+1)';     % initial states decision variables

t(1) = t0;
x_ol(:,1) = x0;                 % Initial predictied predicted states (open loop)

sim_time = 10;                  % Maximum simulation time
goal_tolerance = 0.01;          % Goal tolerance in [m]

%% Trajectory reference
% Strait line
x_start = [0, 1, 0.0];
x_goal = [4, 1, 0.0];
u_ref = [0.5, 0];               % Reference control input

x_ref_i = 1;
x_ref = x_start;          % Intial postion for reference trajectory

while(x_goal(1) >= x_ref(x_ref_i,1))
    x_ref = [x_ref; x_ref(x_ref_i, 1)+u_ref(1)*Ts, x_ref(x_ref_i, 2)+u_ref(2)*Ts, 0]; %not consideres orientation
    x_ref_i = x_ref_i + 1;
end
x_ref_end_i = x_ref_i-1;

%% Start MPC
mpc_i = 0;    % Couter for the MPC loop
x_cl = [];    % Store predicted states in the closed loop
u_cl=[];      % Store control inputs in the closed loop

runtime = tic;
%while(norm((x0-x_goal),2) > goal_tolerance && mpc_i < sim_time / Ts)
while(norm((x0-x_goal'),2) > goal_tolerance && mpc_i < sim_time / Ts)
    mpc_time = tic;
    
    %args.p = [x0; x_goal];         % set the values of the parameters vector
    args.p(1:3) = x0;               %first 3 params are the initial states
    
    % Trajectory reference
    for k = 1:N
        if mpc_i+k >= x_ref_end_i    % If the trajectory refernce reach the goal, do point stabilization
            args.p((5*k-1):(5*k+1)) = x_goal(:);
            args.p((5*k+2):(5*k+3)) = [0, 0];
        else                        % Else follow the reference trajectory point on the prediction
            args.p((5*k-1):(5*k+1)) = x_ref(mpc_i+k, :);
            args.p((5*k+2):(5*k+3)) = u_ref;
        end
    end
    
    % initial value of the optimization variables (reshaped to be a vector
    %args.x0 = reshape(u0',2*N,1);
    args.x0 = [reshape(x_st_0',3*(N+1),1);reshape(u0',2*N,1)];   
    
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    
    %u = reshape(full(sol.x)',2,N)';                
    u = reshape(full(sol.x(3*(N+1)+1:end))',2,N)'; % Control inputs from solution
    
    %opt_value = optimal_solution(u',args.p);       % calculate optimal trajectory
    
    x_cl(:,1:3,mpc_i+1)= reshape(full(sol.x(1:3*(N+1)))',3,N+1)';  % Store all the predictions
    u_cl= [u_cl ; u(1,:)];                         % Store all control actions (the first from each prediction)
    
    % Shift the calculated states into the next initiation
    t(mpc_i+1) = t0;                               % Store time
    [t0, x0, u0] = shift(Ts, t0, x0, u,mapping_func);    % Update x0 and u0
    x_ol(:,mpc_i+2) = x0;                          % Store calculated states
    
    x_st_0 = reshape(full(sol.x(1:3*(N+1)))',3,N+1)';   % get solution trajectory
    
    % Shift trajectory to initialize the next step
    x_st_0 = [x_st_0(2:end,:); x_st_0(end,:)];
    
    mpc_i
    % mpc_time = toc(mpc_time)
    
    mpc_i = mpc_i + 1;
end

run_time = toc(runtime)
position_error = norm((x0-x_goal'),2)
average_mpc_cl_time = run_time/(mpc_i+1)

clf
Simulate_MPC_trajectory_tracking (t,x_ol,x_cl,u_cl,x_ref,x_goal,N,rob_diameter)
Plot_Control_Input (t, u_cl, v_min, v_max, w_min, w_max)
