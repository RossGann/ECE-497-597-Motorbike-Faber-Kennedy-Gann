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
EMMass = 52.2;                                       % Electric Machine Mass kilograms
EMass = 29.84638;                                   % Engine Mass kilograms
FTMass = 6.80389;                                   % Fuel Tank Mass kilograms, used 15 lbs from the 12-20 lb range
TMass = 4.9;                                        % Transmission kilograms
CMass = 3.9;                                        % Clutch kilograms
RMass = 60;                                         % Mass of Rider kilograms
massBatt = 22.68;                                   % mass of battery calculated below
Meq = CurbMass - EMass - FTMass - TMass - CMass + EMMass + RMass + massBatt;      % total mass kilograms

FR = 135.4;             % Rolling Resistance 
% -------------------
% PARAMETERS - AC-51 19 Spline
% -------------------
% Pr = 53000;                     % Rated Power Watts
% Tr = 162.76;                    % Rated Torque

% % AC-51 96 Volt
PrHP = 64.57;                   % Rated Power HP
Pr = PrHP*0.746*1000;           % Rated Power Watts
Trftlbs = 137.32;               % Rated Torque   
Tr = Trftlbs*1.36;              % Converting ft-lbs to N-m

% These are some of the other electric machines that were tested %

% AC-51 108 Volt
% PrHP = 74.14;                   % Rated Power HP
% Pr = PrHP*0.746*1000;           % Rated Power Watts
% Trftlbs = 136.44;               % Rated Torque   
% Tr = Trftlbs*1.36;              % Converting ft-lbs to N-m

% AC-51 144 Volt
% PrHP = 88.09;                   % Rated Power HP
% Pr = PrHP*0.746*1000;           % Rated Power Watts
% Trftlbs = 100.45/0.73756;       % Rated Torque   
% Tr = Trftlbs*1.36;              % Converting ft-lbs to N-m

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

%% CONSTANT TORQUE MODE for Quarter Mile
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

%% CONSTANT POWER MODE for Quarter Mile
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
xmiles = x./1609;

% plot for velocity vs. distance
figure(1); plot(xmiles,vm(1:length(x)),'LineWidth',2);
xlabel('Distance (miles)');
ylabel('Velocity (mph)');
title('Velocity vs. Distance')

% plot for power vs. distance
xdist = linspace(0,(distance/1609));
power = ((Pr/1e3)/(distance/1609))*xdist;
figure(2); plot(xdist,power,'r','LineWidth',2);
xlabel('Distance (miles)');
ylabel('Power (kW)');
title('Power vs. Distance')

% plot for torque vs. distance
torque = ones(length(x))*Tr;
figure(3); plot((x/1609),torque,'m','LineWidth',2);
xlabel('Distance (miles)');
ylabel('Torque (N-m)');
title('Torque vs. Distance')

H = figure(4);           % PLOT VELOCITY
yyaxis left;
plot(t,vm,'r','LineWidth',2)
ylabel('Velocity (mi/h)')
xlabel('Time (s)');

yyaxis right       % PLOT ROTOR POWER
plot(trotor,Protor,'b--','LineWidth',2)

ax = gca;           % SET Y-AXIS COLOR
ax.YAxis(1).Color = 'k';
ax.YAxis(2).Color = 'k';
legend('Velocity ({\itv}_m)','Rotor Power ({\itP}_{R})','Location','SouthEast')
fprintf('The Time for A Quarter Mile is: %0.2f seconds\n', time_final)
%fMUPEL_PLOT('Tesla Model S - Acceraltion Profile','Time (s)','Rotor Power (kW)','OneNote','HW02_PR02_A')
%% Battery 
Power = @(t) 4.46*t;                    % setting up to find Energy = integral of power 
tmin = 0;                               % starting time
tmax = time_final;                      % time to finish quarter mile
Energy = integral(Power,tmin,tmax);     % integrating power to find energy
DoD = 100;                              % depth of discharge is set to 100%
Ebt = Energy/DoD;                       % energy available in the battery kW
VBT_max = 96;                           % voltage needed to supply electric machine
Vcell_nom = 3.6;                        % voltage of one battery cell
NcellSeries = ceil(VBT_max/Vcell_nom);  % calculating the number of cells needed in the battery pack n
Icell_max = 45;                         % max amperage rating of single battery cell
Pcell_max = Vcell_nom*Icell_max;        % max power of one cell 
Pstr = NcellSeries*Pcell_max;           % power of string
Ptpeak = Pr;                            % peak power
Ncell_parallel = ceil(Ptpeak/Pstr);     % calculating number of cells in parallel
massCell = 0.07;                        % mass of singluar cell in kg
CellsNeeded = NcellSeries*Ncell_parallel; % cells needed to power electric machine
massBatt = massCell*CellsNeeded;        % mass of battery pack
diameterBatt = 21.4;                    % diameter of singular battery cell mm
lengthBatt = 70;                        % length of singular battery cell mm
widthChassis = 260;                     % width dimension of chassis
Ntop= ceil(widthChassis/lengthBatt);    % batterys that can fit in top
diagChassis = 428.7;                    % diagonal dimension of chassis mm
lengthChassis = 468;                    % length dimension of chassis mm
NcellDiag = ceil((diagChassis*0.85)/diameterBatt);   % number of battery cells that can fit along diagonal chassis
NcellHorz = ceil((lengthChassis*0.85)/diameterBatt); % number of battery cells that can fit along length chassis
Ncells = Ntop*NcellDiag*NcellHorz;                   % total number cells that can fit in chassis
%% Battery Simulation Energy & Power for Quarter Mile
t = 0:(11.3/113):11.3;                  % Ensuring battery simulation is only for time for quarter mile
DC_torque = ones(1,length(t)).*Tr;      % Torque is constant for entirety of quarter mile
w_speed = (vm/Rw).*vconv;               % changing units to rad/s for velocity
powa = DC_torque.*w_speed;              % calculating power using torque and angular velocity                  
figure; plot(t,powa,'b','LineWidth',2); % plotting power
title('Power as a Function of Time');
xlabel('Time (s)');
ylabel('Power (kW)');

Energy2 = powa.*(t/3600);               % calculating energy using power & time for quarter mile in hours
figure; plot(t,Energy2,'b','LineWidth',2);
title('Energy as a Function of Time');
xlabel('Time (s)');
ylabel('Energy (kWh)');

%% CONSTANT TORQUE MODE  %%%%%%%%%%%%%%%% NOT FOR QUARTER MILE FROM HERE ON, JUST TORQUE SPEED CURVE
while vm(i)< vmr %&& distance <= 402.336
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
while vm(i) < vts %&& distance <= 402.336
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

time_speed = t(i);

% CREATE ROTOR POWER DATA
Protor = [0, Pr, Pr]/1e3;
Trotor = [Tr, Tr, 0];
trotor = [0, vmr/vconv, vts/vconv ];

%% PLOT RESULTS
vm = vm./vconv;  % Conver to mi/h

H = figure(7);           % PLOT VELOCITY
title('Torque/Power vs Vehicle speed')
% yyaxis left;
% plot(t,vm,'r')
% ylabel('Velocity (mi/h)')
% xlabel('Time (s)');

yyaxis left;
plot(trotor,Trotor,'r','LineWidth',2)
ylabel('Torque (N-m)')
xlabel('Vehicle speed (mph)')

yyaxis right       % PLOT ROTOR POWER
plot(trotor,Protor,'b--','LineWidth',2)
ylabel('Power (kW)')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Ross G end
ax = gca;           % SET Y-AXIS COLOR
ax.YAxis(1).Color = 'k';
ax.YAxis(2).Color = 'k';
legend('Torque ({\itv}_m)','Rotor Power ({\itP}_{R})','Location','SouthEast')
%fMUPEL_PLOT('Tesla Model S - Acceraltion Profile','Time (s)','Rotor Power (kW)','OneNote','HW02_PR02_A')


