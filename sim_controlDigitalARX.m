%% Digital Control Simulation — ARX Model + Saturation
clear all; close all; clc;

%% Sampling period
Ts = 0.004;          % 4 ms

%% Reference signal
t = 0:Ts:2;          % 2 seconds simulation
Ref =5* ones(1,length(t)); % Step reference

%% ARX model coefficients
% A(z) = 1 - 1.937 z^-1 + 1.152 z^-2 - 0.2144 z^-3
% B(z) = -0.001961 z^-3
a = [1  -1.9366 1.1523 -0.2144]; % note the sign for implementation
b = [0 0 0 -0.001961];       % B(z) with zeros for delays

%% Initialize variables
y  = zeros(1,length(t));
y1 = 0; y2 = 0; y3 = 0;             % past outputs
u1 = 0; u2 = 0; u3 = 0;u4 = 0;       % past inputs
error1 = 0;                    % past error
Usim = zeros(1,length(t));

%% PI controller parameters
Kp = -11.6083; % propotional gain 
Ti = 0.1014; % integral time

%% PI controller parameters discretization 
K0 = Kp + Kp*Ts/(2*Ti);
K1 = -Kp + Kp*Ts/(2*Ti);

%% Saturation limits
Umax = 100;
Umin = -100;

%% Simulation loop
for k = 1:length(t)
    % ARX model
    y(k) = -a(2)*y1 - a(3)*y2 - a(4)*y3  + b(1)*u1 + b(2)*u2 + b(3)*u3+ b(4)*u4;

    % Control error
    error = Ref(k) - y(k);

    % PI control (incremental form)
    u = u1 + K0*error + K1*error1;

    % Saturation
    if u > Umax
        u = Umax;
    elseif u < Umin
        u = Umin;
    end

    % Update past values
    y3 = y2; 
    y2 = y1; 
    y1 = y(k);
    u4 = u3; 
    u3 = u2; 
    u2 = u1; 
    u1 = u;
    error1 = error;
    Usim(k)=u;
end

%% Plot results
figure;
subplot(2,1,1)
plot(t,y,'b+',t,Ref','r--','MarkerSize',4);
xlabel('Time (s)'); 
ylabel('Output y(k)');
legend('System Output','Reference');
title('ARX System Response')

subplot(2,1,2)
plot(t,Usim,'k+','MarkerSize',4);
xlabel('Time (s)'); 
ylabel('Control Signal u(k)');
title('PI Control Signal with Saturation')
