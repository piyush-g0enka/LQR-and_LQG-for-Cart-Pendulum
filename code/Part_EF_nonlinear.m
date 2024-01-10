%% Luenberger Observer Implementation References
% https://www.mathworks.com/help/control/ref/dynamicsystem.initial.html
% https://www.mathworks.com/help/ident/ref/dynamicsystem.step.html
% https://www.mathworks.com/help/control/ref/lti.lqr.html
% https://www.mathworks.com/help/matlab/ref/ode45.html

%% Clear all workspaces and command lines
close all; clear; clc

%% User defined inputs can be changed here 

%Define Output Case to evaluate
%Can choose values (cases) 1, 3, or 4
output_case = 4

%Define seconds in time to simulate response
sim_time_sec = 50;

% Define arbitrary poles
pole_values = [-2.3, -2.7, -3.1, -3.5, -3.9, -4.3];

% Define initial condition parameters
initial_condition = [0.1; 0.0; -0.19; 0.0; 0.31; 0; 0; 0; 0; 0; 0; 0];

% Use optimal R and Q symmetric postive matrices obtained from Component 1, Part D
Q = diag([316, 315, 320, 800, 320, 800]);
R = 0.001;

%% Define linearized state space variables and matrices

syms g M m1 m2 l1 l2
syms x x_dot th1 th1d th2 th2d

M = 1000;
m1 = 100;
m2 = 100;
l1 = 20;
l2 = 10;
g = -9.8;
a1 = m1*g/M;
a2 = m2*g/M;
a3 = g*(m1+M)/(l1*M);
a4 = m2*g/(l1*M);
a5 = g*m1/(l2*M);
a6 = g*(m2+M)/(l2*M);
AF = [0 1 0 0 0 0; 0 0 a1 0 a2 0; 0 0 0 1 0 0; 0 0 a3 0 a4 0; 0 0 0 0 0 1; 0 0 a5 0 a6 0]
BF = transpose([0 1/M 0 1/(l1*M) 0 1/(l2*M)])
D = 0;

%% Define C
switch output_case 
    case 1 % Y is function of x(t)
        c1r1 = [1 0 0 0 0 0];
        C1 = c1r1;
        C = C1;
    case 3 % Y is function of x(t) and th2(t)
        c3r1 = [1 0 0 0 0 0];
        c3r3 = [0 0 0 0 1 0];
        C3 = [c3r1; c3r3];
        C = C3;
    case 4 % Y is function of x(t) and th1(t) and th2(t)
        c4r1 = [1 0 0 0 0 0];
        c4r2 = [0 0 1 0 0 0];
        c4r3 = [0 0 0 0 1 0];
        C4 = [c4r1; c4r2; c4r3];
        C = C4;
end


%% Test for Observability

AF_T = transpose(AF);
C_T = transpose(C);

observe_mtx1 = [C; C*AF; C*AF^2; C*AF^3; C*AF^4; C*AF^5];

c1_rank = rank(observe_mtx1);
disp(["Case 1 Rank: ", num2str(c1_rank)])


%% Compute L Gain Matrix
AF_transp = transpose(AF);
L = place(AF_transp,C_T,pole_values); % L = acker(transpose(AF),C,pole_values)
fprintf('L =\n');
fprintf('   %.4f   %.4f   %.4f   %.4f   %.4f   %.4f\n', L);
formattedString = sprintf('%.5e', L);

L_T = transpose(L);

%% Compute K 

K = lqr(AF, BF, Q, R);
fprintf('K =\n');
fprintf('   %.4f   %.4f   %.4f   %.4f   %.4f   %.4f\n', K);

%% Compute state space for the nonlinar system using ODE45
sim_time = 0:0.1:sim_time_sec;

[t,x] = ode45(@track_states,sim_time,initial_condition);

plot(t,x)
xlabel("Time (seconds)")
ylabel("Output States")
title("System Response")

%% ODE45 Function
function dydt = track_states(t,cl_state)

    % Provide covariance values
    process_cov = 0.1*eye(6);
    measure_cov = 1;

    % Redefine params
    R = 0.001;
    Q = diag([316, 315, 320, 800, 320, 800]);

    M = 1000;
    m1 = 100;
    m2 = 100;
    l1 = 20;
    l2 = 10;
    g = -9.8;

    a1 = m1*g/M;
    a2 = m2*g/M;
    a3 = g*(m1+M)/(l1*M);
    a4 = m2*g/(l1*M);
    a5 = g*m1/(l2*M);
    a6 = g*(m2+M)/(l2*M);

    AF = [0 1 0 0 0 0; 0 0 a1 0 a2 0; 0 0 0 1 0 0; 0 0 a3 0 a4 0; 0 0 0 0 0 1; 0 0 a5 0 a6 0];
    BF = transpose([0 1/M 0 1/(l1*M) 0 1/(l2*M)]);
    D = 0;

    % Compute K Gain 
    K = lqr(AF, BF, Q, R);
    
    % Compute input
    input = -(K*cl_state(1:6));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Modify depending on case 1, 3, or 4
    % Recompute C
    % c1r1 = [1 0 0 0 0 0];
    % C1 = c1r1;
    % C = C1;
    
    % c3r1 = [1 0 0 0 0 0];
    % c3r3 = [0 0 0 0 1 0];
    % C3 = [c3r1; c3r3];
    % C = C3;

    c4r1 = [1 0 0 0 0 0];
    c4r2 = [0 0 1 0 0 0];
    c4r3 = [0 0 0 0 1 0];
    C4 = [c4r1; c4r2; c4r3];
    C = C4;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Compute L Gain matrix
    LL = lqr(transpose(AF),transpose(C),process_cov,measure_cov);
    LL = transpose(LL);

    xe_dot = (AF-(LL*C)) * cl_state(7:12);

    % Track states below

    % Derive x(t) state variables
    dydt = []';
    dydt(12, 1) = 0;
    % x'
    dydt(1) = cl_state(2); 
    % x''
    calc_1 = (input - (g/2)*(m1*sind(2*cl_state(3))+m2*sind(2*cl_state(5))) - (m1*l1*(cl_state(4)^2)*sind(cl_state(3)))-(m2*l2*(cl_state(6)^2)*sind(cl_state(5))));
    calc_2 = (M+m1*((sind(cl_state(3)))^2)+m2*((sind(cl_state(5)))^2));
    dydt(2) = calc_1 / calc_2;
    % th1'
    dydt(3) = cl_state(4);
    % th1''
    dydt(4) = (dydt(2) * cosd(cl_state(3)) - g*(sind(cl_state(3))) ) / l1';
    % th2'
    dydt(5) = cl_state(6);
    % th2''
    
    % Derive Xe state variables
    dydt(6) = (dydt(2) * cosd(cl_state(5)) - g*(sind(cl_state(5)))) / l2;
    dydt(7) = cl_state(2) - cl_state(10); 
    dydt(8) = dydt(2)-xe_dot(2);
    dydt(9) = cl_state(4) - cl_state(11);
    dydt(10) = dydt(4)-xe_dot(4);
    dydt(11) = cl_state(6) - cl_state(12);
    dydt(12) = dydt(6)-xe_dot(6);

end
