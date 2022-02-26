% Simulation

clear 
close all
tic
%% Parameter 
g = 9.81; %Gravity in m/s^2

% Motors
M_max = 0.2; % Maximum motor torque in Nm
wM_max = 17.8; % Maximum motor speed in rad/s
P_max = wM_max*M_max; % Maximum electric motor power (Watt)
u_max = 100; %Maximum power level of the motor in percent (gross)

% Roboter
m_R = 0.8; % Mass Robot in kg
r_R = 0.041; % Wheel radius in m
k_r = 7.2e-3; % Rolling resistance coefficient

% Beam 
l_W = 0.6; % Beam length in m
m_W = 0.01; % Beam mass in kg
phi_max = pi/2; % Maximum deflection beam

% Ball
m_B = 0.05; % Ball mass in kg

% Rail
l_S = 5; % Rail length in m

% Disturbance
Disturbance = 0.1; % .1 = Stoerung ein, 0 = aus
disturbanceNoisePower = 0.01;
% rng('shuffle');
rng(1)
disturbanceSeed_1 = round((1e4-1)*rand(1));

% Initial conditions

xStart = 0.25; % Starting position of the robot
x_dotStart = 0; % Starting velocity of the robot
phiStart = 0.0; % Starting beam angle
phi_dotStart = 0; % Starting beam velocity
zStart = 0.00; % Starting ball poistion
z_dotStart = 0; % Starting ball velocity

%% Initialisierung des Reglers
InitController

IC = [xStart; phiStart; zStart; x_dotStart; phi_dotStart; z_dotStart]; % Vector with initial conditions, alpha_sStart is to be initialized in ControllerInit
%% Simulation
tEnd = 600; % in seconds
tSim = [0 tEnd]; % Time interval 
tStep = 1e-4;    % Step size of the ODE-solvers

sim('RobotModel',tSim);
disp('Simulation finished')
disp('End time:')
tout(end,1)
disp('Constraint violated?')
constraint_violation(1,2)

%% Plots
close all

xSim = yout(:,1);
x_dotSim = yout(:,2);
phiSim = yout(:,3);
phi_dotSim = yout(:,4);
zSim = yout(:,5); 
z_dotSim = yout(:,6);
x_sollSim = yout(:,7); 
x_diffSim = yout(:,8); 

figure('Name','States')
subplot(2,3,1)
hold all
title('Robot position in m');
plot(tout,xSim)
plot([tout(1),tout(end)],[l_S/2,l_S/2],'Color','r');
plot([tout(1),tout(end)],[-l_S/2,-l_S/2],'Color','r');
xlabel('Time in s')
subplot(2,3,4)
hold all
plot(tout,x_dotSim)
title('Robot velocity in m/s');
xlabel('Time in s')
subplot(2,3,2)
hold all
plot(tout,180/pi*phiSim)
plot([tout(1),tout(end)],[phi_max*180/pi,phi_max*180/pi],'Color','r');
plot([tout(1),tout(end)],[-phi_max*180/pi,-phi_max*180/pi],'Color','r');
title('Beam angle in degrees')
xlabel('Time in s')
subplot(2,3,5)
hold all
plot(tout,phi_dotSim)
title('Rotation velocity beam in rad/s');
xlabel('Time in s')
subplot(2,3,3)
hold all
title('Ball position in m');
plot(tout,zSim)
plot([tout(1),tout(end)],[l_W/2,l_W/2],'Color','r');
plot([tout(1),tout(end)],[-l_W/2,-l_W/2],'Color','r');
xlabel('Time in s')
subplot(2,3,6)
hold all
plot(tout,z_dotSim)
title('Ball velocity in m/s');
xlabel('Time in s')

figure('Name','Reference position')
hold all
grid on
plot(tout,xSim)
plot(tout,x_sollSim)
legend({'Position', 'Reference position'})
title('Position robot und reference position')
xlabel('Time in s')
ylabel('in m')

figure('Name','Deviation position')
hold all
grid on
plot(tout,x_diffSim)
title('Difference to reference position')
xlabel('Time in s')
ylabel('Difference in m')


J = x_diffSim'*x_diffSim;
toc