%% =>- %% ECE 497 / 597 - EV Quarter Mile for CBR500 Using AC-51 %%%%%%%%%
%  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
%  Can uncomment Moment of Inertia & Velocity using Moment of Inertia
%  equations if moment of inertia is given for the rotor
%  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
%  Two Wheeled Unicycle
%  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
%  Ross Gann, Ross Kennedy, Katey Faber
%  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% INITIALIZATION
clear; clc;

%% VARIABLES
% -------------------
% ENVIRONMENT
% -------------------
rho = 1.16;            % kg/m^3 (Air Density)
g = 9.81;              % m/s^2 (Gravity)
AF = 0.438;            % approx upright SA of CBR1000
CD = 0.270;            % coefficient of drag of CBR1000rr
% -------------------
% PARAMETERS - VEHICLE - CBR500
% -------------------
distance = 0;
CurbMass = 194.138;                                 % kilograms
EMMass = 9.5;                                       % Electric Machine Mass kilograms
EMass = 29.84638;                                   % Engine Mass kilograms
FTMass = 6.80389;                                   % Fuel Tank Mass kilograms, used 15 lbs from the 12-20 lb range
TMass = 4.9;                                        % Transmission kilograms
CMass = 3.9;                                        % Clutch kilograms
RMass = 60;                                         % Mass of Rider kilograms
Meq = CurbMass - EMass - FTMass - TMass - CMass + EMMass + RMass;      % total mass kilograms

FR = 135.4;             % Rolling Resistance 
% -------------------
% PARAMETERS - AC-51 19 Spline
% -------------------
Pr = 53000;             % Rated Power
Tr = 162.76;            % Rated Torque
% UNCOMMENT THIS SECTION IF MOI IS GIVEN
% J = 9;                  % Moment of Inertia of Rotor

% -------------------
% TIRES - 160/60-17 Rear Wheel Tire
% -------------------
d = 0.6298;             % meters
swh = 0.099;            % meters
Rw = (d + 2*swh)/2;     % meters, assumed both tires are same size (in reality not)
Cr = 0.02;              % Rolling Resistance
vconv = 1609.34/3600;   % mi/h to m/s
% -------------------
% GEARBOX
% -------------------
Ngb = 2.6;              % Gear Ratio
ngb = 0.9;              % gearbox efficiency (no gears in electric motorcycles)

%% Rated Parameters
wr = Pr/Tr;             % (rad/s) Rated angular speed of the rotor
wmr = wr/Ngb;           % (rad/s) Rated anglue speed of the wheel
vmr = wmr*Rw;           % (m/s) Rated vehicle speed
vmr_mph = vmr/vconv;    % (mi/h) Rated vehicle speed

vts = 100*vconv;        % (m/s) Top Speed

t(1) = 0;               % Initial time
dt = 0.1;               % Time step is 1s
vm(1) = 0;              % Initial speed
i =1;                   % Counter

%% CONSTANT TORQUE MODE
while vm(i)< vmr && distance <= 402.336
    % CALCULATE SPEED AT NEXT STEP
    % COMMENT THIS SECTION IF MOI IS GIVEN
    FD = 0.5*AF*CD*rho*vm(i);
    vm(i+1) = vm(i) + dt*(Ngb*ngb*Tr - Rw*(FR + FD))/Rw/Meq;
    
    % UNCOMMENT THIS SECTION IF MOI IS GIVEN
    % vm(i+1) = vm(i) + dt*(Ngb*ngb*Tr - Rw*(FR + FD))/(Rw*Meq+J/Rw);
    
    % CALCULATE TIME AT NEXT STEP
    t(i+1) = t(i)+dt;
    
    % CALCULATE POSITION
    x(i) = vm(i)*t(i);
    
    % SET CURRENT POSITION EQUAL TO DISTANCE TO MONITOR WHEN IT REACHES 1/4
    % MILE
    distance = x(i);
    
    % INCREMENT STEP
    i = i+1;
end

time_rated = t(i);

%% CONSTANT POWER MODE
while vm(i) < vts && distance <= 402.336
    % CALCULATE SPEED AT NEXT STEP
    % COMMENT THIS SECTION IF MOI IS GIVEN
    FD = 0.5*AF*CD*rho*vm(i);
    vm(i+1) = vm(i) + dt*(ngb*Pr/vm(i)*Rw - Rw*(FR + FD))/Rw/Meq;
    
    % UNCOMMENT THIS SECTION IF MOI IS GIVEN
    %  vm(i+1) = vm(i) + dt*(ngb*Pr/vm(i)*Rw - Rw*(FR + FD))/(Rw*Meq+J/Rw);
    
    % CALCULATE TIME AT NEXT STEP
    t(i+1) = t(i)+dt;
    
    % CALCULATE POSITION
    x(i) = vm(i)*t(i);
        
    % SET CURRENT POSITION EQUAL TO DISTANCE TO MONITOR WHEN IT REACHES 1/4
    % MILE
    distance = x(i);
    
    % INCREMENT STEP
    i = i+1;
end

time_final = t(i);

% CREATE ROTOR POWER DATA
Protor = [0, Pr, Pr]/1e3;
trotor = [0, time_rated, time_final ];

%% PLOT RESULTS
vm = vm./vconv;  % Conver to mi/h

H = figure(1);           % PLOT VELOCITY
yyaxis left;
plot(t,vm,'r')
ylabel('Velocity (mi/h)')
xlabel('Time (s)');

yyaxis right       % PLOT ROTOR POWER
plot(trotor,Protor,'b--','LineWidth',2)

ax = gca;           % SET Y-AXIS COLOR
ax.YAxis(1).Color = 'k';
ax.YAxis(2).Color = 'k';
legend('Velocity ({\itv}_m)','Rotor Power ({\itP}_{R})','Location','SouthEast')
fprintf('The Time for A Quarter Mile is: %0.2f seconds', time_final)
%fMUPEL_PLOT('Tesla Model S - Acceraltion Profile','Time (s)','Rotor Power (kW)','OneNote','HW02_PR02_A')
