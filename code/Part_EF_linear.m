%% Luenberger Observer Implementation References
% https://www.mathworks.com/help/control/ref/dynamicsystem.initial.html
% https://www.mathworks.com/help/ident/ref/dynamicsystem.step.html
% https://www.mathworks.com/help/control/ref/lti.lqr.html

%% Clear all workspaces and command lines
close all; clear; clc

%% User defined inputs can be changed here 

%Define Output Case to evaluate
%Can choose values (cases) 1, 3, or 4
output_case = 4

%Define seconds in time to simulate response
sim_time_sec = 400;

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
disp(["Case Rank: ", num2str(c1_rank)])


%% Compute L Gain Matrix
AF_transp = transpose(AF);
L = place(AF_transp,C_T,pole_values); 
fprintf('L =\n');
fprintf('   %.4f   %.4f   %.4f   %.4f   %.4f   %.4f\n', L);
formattedString = sprintf('%.5e', L);

L_T = transpose(L);

%% Compute K 

K = lqr(AF, BF, Q, R);
fprintf('K =\n');
fprintf('   %.4f   %.4f   %.4f   %.4f   %.4f   %.4f\n', K);

%% Compute new closed loop state space 
A_closed = [AF-(BF*K) -BF*K; zeros(size(AF)) AF-(transpose(L)*C)];
B_closed = [BF;BF];
C_closed = [C zeros(size(C))];
D_closed = 0;

%% Pass in state space model matrices to ss function and plots responses
sys = ss(A_closed, B_closed, C_closed, D_closed);

%% Plot simulation results depending on the output case

if output_case == 4
    % Use step function to generate step input response over time_sec seconds
    figure;
    initial(sys, initial_condition, sim_time_sec);
    
    % Find all axes objects in the current figure
    axesHandles = findall(gcf, 'type', 'axes');
    % newLabel1 = 'Amplitude';
    newLabel2 = 'X(t)';
    newLabel3 = '\theta_1(t)'; % Assuming you intended different labels for the third subplot
    newLabel4 = '\theta_2(t)'; % Assuming you intended different labels for the third subplot
    set(get(axesHandles(1), 'YLabel'), 'String', '');
    set(get(axesHandles(2), 'YLabel'), 'String', newLabel4);
    set(get(axesHandles(3), 'YLabel'), 'String', newLabel3);
    set(get(axesHandles(4), 'YLabel'), 'String', newLabel2);
    title(axesHandles(1), "Initial Conditions Response: (x(t), \theta_1(t), \theta_2(t))")
    
    figure, step(sys, sim_time_sec)
    % Find all axes objects in the current figure
    axesHandles = findall(gcf, 'type', 'axes');
    % newLabel1 = 'Amplitude';
    newLabel2 = 'X(t)';
    newLabel3 = '\theta_1(t)'; % Assuming you intended different labels for the third subplot
    newLabel4 = '\theta_2(t)'; % Assuming you intended different labels for the third subplot
    set(get(axesHandles(1), 'YLabel'), 'String', '');
    set(get(axesHandles(2), 'YLabel'), 'String', newLabel4);
    set(get(axesHandles(3), 'YLabel'), 'String', newLabel3);
    set(get(axesHandles(4), 'YLabel'), 'String', newLabel2);
    title(axesHandles(1), "Step Response: (x(t), \theta_1(t), \theta_2(t))")


elseif output_case == 3

    % Use step function to generate step input response over time_sec seconds
    figure, initial(sys, initial_condition, sim_time_sec);
    
    axesHandles = findall(gcf, 'type', 'axes');
    newLabel1 = '';
    newLabel2 = 'X(t)';
    newLabel3 = '\theta_2(t)'; % Assuming you intended different labels for the third subplot
    set(get(axesHandles(1), 'YLabel'), 'String', newLabel1);
    set(get(axesHandles(2), 'YLabel'), 'String', newLabel3);
    set(get(axesHandles(3), 'YLabel'), 'String', newLabel2);
    title(axesHandles(1), "Initial Conditions Response: (x(t), \theta_2(t))")


    figure, step(sys, sim_time_sec)

    axesHandles = findall(gcf, 'type', 'axes');
    newLabel1 = '';
    newLabel2 = 'X(t)';
    newLabel3 = '\theta_2(t)'; % Assuming you intended different labels for the third subplot
    set(get(axesHandles(1), 'YLabel'), 'String', newLabel1);
    set(get(axesHandles(2), 'YLabel'), 'String', newLabel3);
    set(get(axesHandles(3), 'YLabel'), 'String', newLabel2);
    title(axesHandles(1), "Step Response: (x(t), \theta_2(t))")


elseif output_case == 1

    % Use step function to generate step input response over time_sec seconds
    figure, initial(sys, initial_condition, sim_time_sec);
    
    axesHandles = findall(gcf, 'type', 'axes');
    newLabel1 = 'X(t)';
    set(get(axesHandles(1), 'YLabel'), 'String', newLabel1);
    title(axesHandles(1), "Initial Conditions Response: (x(t))")


    figure, step(sys, sim_time_sec)

    axesHandles = findall(gcf, 'type', 'axes');
    newLabel1 = 'X(t)';
    set(get(axesHandles(1), 'YLabel'), 'String', newLabel1);
    title(axesHandles(1), "Step Response: (x(t))")

end
