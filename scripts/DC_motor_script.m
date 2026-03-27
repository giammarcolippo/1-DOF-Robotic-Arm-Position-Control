%% PROJECT 2: Robotic Arm Angle-Control using PD

clc
clear all
close all

%Plant Parameters
Vmax=24;                             % Max Voltage [V]
g=9.81;                              % grav. acc [kg m/s2]
m=1.5;                               % arm mass [kg]
m_load=1;                            % ext. load [kg]
x=0.3;                               % arm length [m]
Jm=0.00003;                          % rotor Inertia [kg*m^2]
J_arm=1/3*m*x^2;                     % rotational arm Inertia [kg*m^2]
J_load=m_load*x^2;                   % load Inertia [kg*m^2]
Jtot=Jm+J_arm+J_load;                % tot Inertia [kg*m^2]
b=0.05;                              % Viscous friction coefficient [Nms]
Kb=1.25;                             % Back EMf constant [V/rad/sec]
Kt=0.5;                              % Torque constant [Nm/Amp]
R=1;                                 % Armature Resistance [ohm]
L=0.02;                              % Armature Inductance [H]
C=0.85*(m*x/2+m_load*x)*g*R/Kt;      % Sim-Constant
sim_time=6;                          % simulation time [s]
dt=0.01;

% Linearized System--> no gravity
% System Requirements
OS = 0.05;                           % Overshoot 5%
Ts = 1;                              % Settling time [s]

% Parameters Calculation
zeta = -log(OS)/sqrt(pi^2 + log(OS)^2);   % Damping ratio
wn = 4/(zeta*Ts);                         % Natural Frequency [rad/s]

% S-Curve profile parameters
theta_final = pi/2;                    % 90 deg
T_ramp = 3;                            % Rise time
t = 0:dt:sim_time;

% Normalized time
s = min(t / T_ramp, 1);

% S-curve profile
theta_ref = theta_final * (3*s.^2 - 2*s.^3);

theta_data = [t', theta_ref'];

ref_deg=rad2deg(theta_ref);

%Pole Placement PID Tuning Method
%PD coefficients calculation
Kp = (Jtot*wn^2)/Kt*5;
Kd = (Jtot*2*zeta*wn-b)/Kt*5.5;
Ki =0*Kp;

out = sim('DC_motor');                                     % Sim running
friction = out.friction;                                   % friction torque array
motor = out.motor;                                         % motor torque array
gravity = out.gravity;                                     % load torque array

theta = rad2deg(out.theta_out);                            % theta position [deg]
volt=out.sig_limited;                                      % Saturated Control real model signal
current=out.current;
velocity=out.velocity;
acceleration=out.acceleration;

time=out.tout;                                             % time-array [s]


%plotting section
figure
set(gcf,'Units','normalized','Position',[0.025 0.025 0.95 0.9]) 
subplot(2,3,1)
hold on
plot(t,ref_deg,'k--','LineWidth',1)
plot(time,theta,'b','LineWidth',1)
xlim([0,sim_time])
ylim([0,120])
xlabel('Time [s]')
ylabel({'\theta','[°]'})
text(4,90+5,'Setpoint = 90°','Color','r','FontSize',10,'FontWeight','bold') 
title({'DC Motor Response - \theta'});
legend('reference','real','Location','southeast')
grid on

subplot(2,3,2)
hold on
plot(time,velocity,'b','LineWidth',1)
xlim([0,sim_time])
ylim([-1,1.5])
xlabel('Time [s]')
ylabel({'\omega','[rad/s]'})
title('Angular velocity');
grid on

subplot(2,3,3)
hold on
plot(time,acceleration,'b','LineWidth',1)
xlim([0,sim_time])
ylim([-20,20])
xlabel('Time [s]')
ylabel({'\alpha','[rad/s²]'})
title('Angular acceleration');
grid on

subplot(2,3,4)
hold on
plot(time,volt,'b','LineWidth',1)
yline(Vmax,'k--','LineWidth', 1.5,'HandleVisibility','off')  
xlim([0,sim_time])
ylim([-2,18])
xlabel('Time [s]')
ylabel({'Voltage','[V]'})
title('Saturated PID Control Signal')
grid on

subplot(2,3,5)
hold on
plot(time,current,'b','LineWidth',1)
xlim([0,sim_time])
ylim([-2,18])
xlabel('Time [s]')
ylabel({'I','[A]'})
title('Direct current')
grid on

subplot(2,3,6)
hold on
plot(time,motor,'b','LineWidth',1)
plot(time,gravity,'r','LineWidth',1)
plot(time,friction,'k','LineWidth',1)
xlim([0,sim_time])
ylim([-1,8])
xlabel('Time [s]')
ylabel({'Torque','[Nm]'})
legend('Motor Torque','Gravity arm + Load Torque','Friction Torque','Location','northeast')
title({'Torques acting: Initial Transient'})
grid on
sgtitle(    {'Project #2: Robotic Arm Position Control using PD'},...
             'FontSize', 12,'FontWeight', 'bold');


