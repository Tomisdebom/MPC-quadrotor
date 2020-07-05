function [results_mpc] = MPC_Controller(mpc_sim, x0, simT, Ts, ulim, ring1, ring2, goal)
%% Initialise
m = 0.50;   % weight of the drone
g = 9.81;   % Gravitational constant

Q = mpc_sim.Q;
Qt = mpc_sim.Qt;
W = mpc_sim.W;
R = mpc_sim.R;
N = mpc_sim.N;

nx = size(Q, 1);    % Number of states
nu = size(R, 1);    % Number of inputs

u = sdpvar(repmat(nu,1,N),ones(1,N));       % Define YALMIP input vector
x = sdpvar(repmat(nx,1,N+1),ones(1,N+1));   % Define YALMIP state vector

%% Calculate waypoints based on rings
fac = 0.05;     % Factor that determines how close the waypoints are to the center of the rings
wayp1 = [ring1(1)-fac*ring1(2) ring1(2) ring1(3)-fac*ring1(4) ring1(4) ring1(5)-fac*ring1(6) ring1(6)]';
wayp2 = [ring1(1)+fac*ring1(2) ring1(2) ring1(3)+fac*ring1(4) ring1(4) ring1(5)+fac*ring1(6) ring1(6)]';
wayp3 = [ring2(1)-fac*ring2(2) ring2(2) ring2(3)-fac*ring2(4) ring2(4) ring2(5)-fac*ring2(6) ring2(6)]';
wayp4 = [ring2(1)-fac*ring2(2) ring2(2) ring2(3)+fac*ring2(4) ring2(4) ring2(5)+fac*ring2(6) ring2(6)]';
goal = goal';

%% Define constraints
constraints = [];
for k = 1:N
    
    % These are the nonlinear system dynamics------------------------------
    %     constraints = [constraints, x{k+1}(1)  == x{k}(1)  + x{k}(2)*Ts];
    %     constraints = [constraints, x{k+1}(2)  == x{k}(2)  + u{k}(1)/m*x{k}(7)*Ts];
    %     constraints = [constraints, x{k+1}(3)  == x{k}(3)  + x{k}(4)*Ts];
    %     constraints = [constraints, x{k+1}(4)  == x{k}(4)  + u{k}(1)/m*x{k}(9)*Ts];
    %     constraints = [constraints, x{k+1}(5)  == x{k}(5)  + x{k}(6)*Ts];
    %     constraints = [constraints, x{k+1}(6)  == x{k}(6)  + (u{k}(1)/m-g)*Ts];
    %     constraints = [constraints, x{k+1}(7)  == x{k}(7)  + x{k}(8)*Ts];
    %     constraints = [constraints, x{k+1}(8)  == x{k}(8)  + u{k}(2)*Ts];
    %     constraints = [constraints, x{k+1}(9)  == x{k}(9)  + x{k}(10)*Ts];
    %     constraints = [constraints, x{k+1}(10) == x{k}(10) + u{k}(3)*Ts];
    
    %     constraints = [constraints, abs(x{k}(7)) <= 15/180*pi];
    %     constraints = [constraints, abs(x{k}(9)) <= 15/180*pi];
    % ---------------------------------------------------------------------
    
    % Dynamical constraints
    constraints = [constraints, x{k+1}(1)  == x{k}(1)  + x{k}(2)*Ts];
    constraints = [constraints, x{k+1}(2)  == x{k}(2)  + u{k}(1)/m*Ts];
    constraints = [constraints, x{k+1}(3)  == x{k}(3)  + x{k}(4)*Ts];
    constraints = [constraints, x{k+1}(4)  == x{k}(4)  + u{k}(2)/m*Ts];
    constraints = [constraints, x{k+1}(5)  == x{k}(5)  + x{k}(6)*Ts];
    constraints = [constraints, x{k+1}(6)  == x{k}(6)  + (u{k}(3)/m-g)*Ts];
    
    % Input constraints
    constraints = [constraints, -ulim(3) <= u{k}(1)<= ulim(1)];
    constraints = [constraints, -ulim(2) <= u{k}(2)<= ulim(2)];
    constraints = [constraints, 0 <= u{k}(3)<= ulim(3)];
    
    % State constraints
    constraints = [constraints -1.5 <= x{k+1}(1) <= 1.5];
    constraints = [constraints 0 <= x{k+1}(3) <= 10];
    constraints = [constraints -1.5 <= x{k+1}(5) <= 1.5];
    constraints = [constraints 0 <= abs(x{k+1}(2)) + abs(x{k+1}(4)) + abs(x{k+1}(6)) <= 3];
    
end

%% Define objective functions and optimizers
options = sdpsettings('solver', 'quadprog');

% Controller for waypoint 1
objective = 0;
for k = 1:N
    objective = objective + norm(W*(x{k}-wayp1), 1) + norm(R*u{k}, 1);
    if k == N
        objective = objective + norm(Qt*(x{k}-wayp1), 1);
    end
end
controller1 = optimizer(constraints, objective, options,x{1},[u{:}]);

% Controller for waypoint 2
objective = 0;
for k = 1:N
    objective = objective + norm(W*(x{k}-wayp2), 1) + norm(R*u{k}, 1);
    if k == N
        objective = objective + norm(Qt*(x{k}-wayp2), 1);
    end
end
controller2 = optimizer(constraints, objective, options,x{1},[u{:}]);

% Controller for waypoint 3
objective = 0;
for k = 1:N
    objective = objective + norm(Q*(x{k}-wayp3), 1) + norm(R*u{k}, 1);
    if k == N
        objective = objective + norm(Qt*(x{k}-wayp3), 1);
    end
end
controller3 = optimizer(constraints, objective, options,x{1},[u{:}]);

% Controller for waypoint 4
objective = 0;
for k = 1:N
    objective = objective + norm(Q*(x{k}-wayp4), 1) + norm(R*u{k}, 1);
    if k == N
        objective = objective + norm(Qt*(x{k}-wayp4), 1);
    end
end
controller4 = optimizer(constraints, objective, options,x{1},[u{:}]);

% Controller for goal
objective = 0;
for k = 1:N
    objective = objective + norm(Q*(x{k}-goal), 1) + norm(R*u{k}, 1);
    if k == N
        objective = objective + norm(Qt*(x{k}-goal), 1);
    end
end
controller5 = optimizer(constraints, objective, options,x{1},[u{:}]);


x_true = x0;
implementedU = [];

st = 1;

f = waitbar(0,'1','Name','Simulating MPC controller',...
    'CreateCancelBtn','setappdata(gcbf,''canceling'',1)');
for i = 1:simT/Ts  
    
    % Select controller based on drone position
    if st == 1
        U = controller1{x_true};
        U = U(:,1);
    elseif st == 2
        U = controller2{x_true};
        U = U(:,1);
    elseif st == 3
        U = controller3{x_true};
        U = U(:,1);
    elseif st == 4
        U = controller4{x_true};
        U = U(:,1);
    elseif st == 5
        U = controller5{x_true};
        U = U(:, 1);
    end
    
    % Nonlinear dynamics---------------------------------------------------
    %     x_true(1)  = x_true(1)  + x_true(2)*Ts;         % x
    %     x_true(2)  = x_true(2)  + U(1)/m*x_true(7)*Ts;  % xdot
    %     x_true(3)  = x_true(3)  + x_true(4)*Ts;         % y
    %     x_true(4)  = x_true(4)  + U(1)/m*x_true(9)*Ts;  % ydot
    %     x_true(5)  = x_true(5)  + x_true(6)*Ts;         % z
    %     x_true(6)  = x_true(6)  + (U(1)/m-g)*Ts;        % zdot
    %     x_true(7)  = x_true(7)  + x_true(8)*Ts;         % theta
    %     x_true(8)  = x_true(8)  + U(2)*Ts;              % thetadot
    %     x_true(9)  = x_true(9)  + x_true(10)*Ts;        % phi
    %     x_true(10) = x_true(10) + U(3)*Ts;              % phidot
    % ---------------------------------------------------------------------
    
    x_true(1)  = x_true(1)  + x_true(2)*Ts;         % x
    x_true(2)  = x_true(2)  + U(1)/m*Ts;            % xdot
    x_true(3)  = x_true(3)  + x_true(4)*Ts;         % y
    x_true(4)  = x_true(4)  + U(2)/m*Ts;            % ydot
    x_true(5)  = x_true(5)  + x_true(6)*Ts;         % z
    x_true(6)  = x_true(6)  + (U(3)/m-g)*Ts;        % zdot
    
    % Check if waypoint is reached and switch case
    if st == 1 && norm([x_true(1) x_true(3) x_true(5)] - [wayp1(1) wayp1(3) wayp1(5)]) <= 0.1
        st = 2;
    elseif st == 2 && norm([x_true(1) x_true(3) x_true(5)] - [wayp2(1) wayp2(3) wayp2(5)]) <= 0.1
        st = 3;
    elseif st == 3 && norm([x_true(1) x_true(3) x_true(5)] - [wayp3(1) wayp3(3) wayp3(5)]) <= 0.1
        st = 4;
    elseif st == 4 && norm([x_true(1) x_true(3) x_true(5)] - [wayp4(1) wayp4(3) wayp4(5)]) <= 0.1
        st = 5;
    end
    
    % Save control inputs and state
    implementedU = [implementedU;U(:,1)'];
    state(i, :) = x_true';
    
    waitbar(i/(simT/Ts),f,sprintf(num2str(ceil(i/(simT/Ts)*100))))
    
end

results_mpc.U       = implementedU;
results_mpc.state   = state;

results_mpc.title = "MPC controller drone";

delete(f)
end

