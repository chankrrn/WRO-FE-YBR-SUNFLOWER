%% CHP-20GP-180 drivetrain screening
% MANUFACTURER-BASED ESTIMATES + HYPOTHETICAL MODELS; NOT PHYSICAL TEST DATA.

clear; clc; close all;
root=fileparts(mfilename('fullpath')); if isempty(root),root=pwd;end
out=fullfile(root,'results'); if ~exist(out,'dir'),mkdir(out);end

%% USER INPUTS
p.motorName='CHP-20GP-180';
p.voltageV=12.0;
p.gearboxRatio=19; % motor-to-gearbox-output nominal 19:1
p.noLoadOutputRPM=780; % manufacturer-listed; confirm output-shaft interpretation
p.ratedOutputRPM=680;  % manufacturer-listed; confirm output-shaft interpretation
p.ratedOutputTorqueKgfCm=0.40;
p.minStallOutputTorqueKgfCm=2.0; % datasheet states >=2.0, NOT exact stall torque
p.ratedCurrentA=0.550; % datasheet states <=550mA, NOT measured nominal current
p.maxStallCurrentA=2.7; % datasheet states <=2.7A
p.noLoadCurrentA=0.280; % datasheet states <=280mA
p.rearTireDiameterM=0.050; % user reported 50 mm unloaded rear diameter
p.reportedRollingRadiusM=0.024; % user-reported, loaded verification pending
p.additionalReduction=1.0; % ASSUMED gearbox-output to drive-wheel ratio, TBD
p.drivetrainEfficiency=0.85; % ASSUMED downstream mechanical efficiency, not measured
p.exampleVehicleMassKg=0.9; % HYPOTHETICAL (actual final mass pending)
p.rollingResistanceCoefficient=0.03; % HYPOTHETICAL
p.gravity=9.81;
p.speedFraction=linspace(0,1,251);
assert(p.gearboxRatio>0 && p.additionalReduction>0 && p.reportedRollingRadiusM>0);
assert(p.noLoadOutputRPM>p.ratedOutputRPM && p.ratedOutputTorqueKgfCm>0);
assert(p.drivetrainEfficiency>0 && p.drivetrainEfficiency<=1 && p.exampleVehicleMassKg>0);

%% Exact unit conversions; quoted motor torque is gearbox OUTPUT unless proven otherwise
kgfcm_to_Nm=0.0980665;
T_rated=p.ratedOutputTorqueKgfCm*kgfcm_to_Nm;
T_stallLower=p.minStallOutputTorqueKgfCm*kgfcm_to_Nm;
omegaNoLoad=p.noLoadOutputRPM*2*pi/60;
% Calibrated straight-line segment through no-load and rated data points.
% DO NOT extrapolate this beyond measured/rated interval to zero-speed stall.
rpmSegment=linspace(p.ratedOutputRPM,p.noLoadOutputRPM,151);
T_segment=T_rated*(p.noLoadOutputRPM-rpmSegment)/(p.noLoadOutputRPM-p.ratedOutputRPM);
% Torque transmitted from gearbox output through any additional reduction
vWheel=(rpmSegment/p.additionalReduction)*2*pi*p.reportedRollingRadiusM/60;
F_segment=T_segment*p.additionalReduction*p.drivetrainEfficiency/p.reportedRollingRadiusM;
P_mech=T_segment.*(rpmSegment*2*pi/60);
% Three independent manufacturer-reported markers, not a fitted whole-range motor curve.

%% Figure 16: Published points / LIMITED linear segment
f=figure('Color','w','Position',[80 90 1000 540]);hold on;
plot(rpmSegment,T_segment,'b-','LineWidth',2);
plot(p.noLoadOutputRPM,0,'ko','MarkerFaceColor','k','MarkerSize',8);
plot(p.ratedOutputRPM,T_rated,'bs','MarkerFaceColor','b','MarkerSize',8);
plot(0,T_stallLower,'rv','MarkerFaceColor','r','MarkerSize',9);
xlabel('Gearbox output speed (RPM)');ylabel('Output torque (N m)');
title('Manufacturer points + limited illustrative segment');
legend('Linear interpolation between rated and no-load','No-load speed; torque approximated 0','Rated speed and torque','Stall torque LOWER BOUND; not a curve fit','Location','northeast');
grid on;saveFigure(f,fullfile(out,'16_motor_spec_points.png'));

%% Figure 17: nominal wheel speed versus external reduction
red=[1 1.5 2 2.5 3];
wrated=p.ratedOutputRPM./red;
v_rated=wrated*2*pi*p.reportedRollingRadiusM/60;
f=figure('Color','w','Position',[100 90 980 500]);bar(red,v_rated);grid on;
xlabel('Additional output-to-wheel reduction (unitless)');ylabel('Ideal rated vehicle speed (m/s)');
title('Theoretical speed sensitivity, ignoring slip');
for k=1:numel(red),text(red(k),v_rated(k)+0.025,sprintf('%.2f',v_rated(k)),'HorizontalAlignment','center');end
saveFigure(f,fullfile(out,'17_speed_vs_reduction.png'));

%% Figure 18: rated tractive force versus rolling radius / further reduction
radiusMM=linspace(20,27,100); reductionVec=linspace(1,3,90);
[RR,GG]=meshgrid(radiusMM/1000,reductionVec);
forceRated=T_rated.*GG.*p.drivetrainEfficiency./RR;
f=figure('Color','w','Position',[110 100 950 550]);
imagesc(radiusMM,reductionVec,forceRated);axis xy;colorbar;
xlabel('Effective rolling radius (mm)');ylabel('Additional reduction');
title('Rated torque-based tractive force, assuming 85% transmission efficiency');
saveFigure(f,fullfile(out,'18_tractive_force_sensitivity.png'));

%% Figure 19: hypothetical force budget against slope, NOT a measured climb capability
slopeDeg=linspace(0,15,150);
Fgrade=p.exampleVehicleMassKg*p.gravity.*sin(deg2rad(slopeDeg));
Froll=p.exampleVehicleMassKg*p.gravity*p.rollingResistanceCoefficient.*cos(deg2rad(slopeDeg));
Favailable=T_rated*p.additionalReduction*p.drivetrainEfficiency/p.reportedRollingRadiusM;
f=figure('Color','w','Position',[110 100 1000 540]);hold on;
plot(slopeDeg,Fgrade+Froll,'LineWidth',2);
yline(Favailable,'--','Rated force estimate');
xlabel('Hypothetical road incline (degrees)');ylabel('Force (N)');
title('Illustrative required force versus rated output force');grid on;
legend('Grade + rolling resistance (mass ASSUMED)','Rated tractive-force estimate','Location','northwest');
saveFigure(f,fullfile(out,'19_force_budget_slope.png'));

function saveFigure(h,name)
try,exportgraphics(h,name,'Resolution',180);
catch,saveas(h,name);
end
end
