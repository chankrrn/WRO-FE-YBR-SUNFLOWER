%% Steering geometry and turning-radius screening

function steering_geometry()

clc; close all;
p = user_inputs();
validate_inputs(p);
outdir = fullfile(fileparts(mfilename('fullpath')), 'results');
if ~exist(outdir,'dir'), mkdir(outdir); end

% Ideal low-speed planar geometry; no tire slip, servo dynamics or compliance.
deltaDeg = linspace(p.minimumSteeringDeg,p.maximumSteeringDeg,400);
delta = deg2rad(deltaDeg);
R = p.wheelbase_m ./ tan(delta); % rear-axle midpoint to turn centre (m)
Rinner = R-p.track_m/2;
Router = R+p.track_m/2;
deltaInner = rad2deg(atan(p.wheelbase_m./Rinner));
deltaOuter = rad2deg(atan(p.wheelbase_m./Router));
yawRate = p.referenceSpeed_mps ./ R; % radians per second

% Requested servo-angle approximation: equivalent virtual-wheel steering angle.
chosenDeg = p.exampleSteeringDeg;
Rc = p.wheelbase_m/tand(chosenDeg);
innerC = atan2d(p.wheelbase_m,Rc-p.track_m/2);
outerC = atan2d(p.wheelbase_m,Rc+p.track_m/2);

% FIGURE 12: Geometry steering angle vs radius.
f=figure('Color','w','Position',[80 80 920 590]);
plot(deltaDeg,R,'LineWidth',2); grid on;
xlabel('Equivalent front-wheel steering angle (deg)');
ylabel('Rear-axle centre turning radius (m)');
title('Ideal bicycle-model turning radius');
xline(chosenDeg,'--','Example angle');
yline(Rc,'--',sprintf('R = %.3f m',Rc));
export_fig(f,fullfile(outdir,'12_turning_radius_vs_steering.png'));

% FIGURE 13: Ackermann angles, ideal theoretical axle geometries.
f=figure('Color','w','Position',[90 90 920 590]);
plot(deltaDeg,deltaInner,'LineWidth',2); hold on;
plot(deltaDeg,deltaOuter,'LineWidth',2);
plot(deltaDeg,deltaDeg,'k--','LineWidth',1);
hold off; grid on;
xlabel('Equivalent front-wheel steering angle (deg)');
ylabel('Physical front-wheel angle (deg)');
legend('Inner wheel (ideal)','Outer wheel (ideal)','Equivalent angle','Location','northwest');
title('Ideal Ackermann front-wheel steering angles');
export_fig(f,fullfile(outdir,'13_ackermann_angles.png'));

% FIGURE 14: XY trajectories for rear midpoint and ideal four wheels.
% Rear axle centre begins at origin facing +X; CCW left-turn trajectory.
theta = linspace(0,deg2rad(p.pathSweepDeg),500);
xRear=Rc*sin(theta);
yRear=Rc*(1-cos(theta));
% Rigid vehicle: local longitudinal offset x=L; lateral offset y=+/- track/2.
xRL = xRear - (p.track_m/2)*sin(theta);
yRL = yRear + (p.track_m/2)*cos(theta);
xRR = xRear + (p.track_m/2)*sin(theta);
yRR = yRear - (p.track_m/2)*cos(theta);
xFL = xRL + p.wheelbase_m*cos(theta);
yFL = yRL + p.wheelbase_m*sin(theta);
xFR = xRR + p.wheelbase_m*cos(theta);
yFR = yRR + p.wheelbase_m*sin(theta);
f=figure('Color','w','Position',[100 100 900 650]);
plot(xRear,yRear,'k--','LineWidth',2); hold on;
plot(xRL,yRL,'LineWidth',1.4); plot(xRR,yRR,'LineWidth',1.4);
plot(xFL,yFL,'LineWidth',1.4); plot(xFR,yFR,'LineWidth',1.4);
plot(0,Rc,'kx','MarkerSize',10,'LineWidth',2);
hold off; grid on; axis equal;
xlabel('Track-plane X (m)'); ylabel('Track-plane Y (m)');
legend('Rear midpoint','Rear left','Rear right','Front left','Front right','Turn centre','Location','bestoutside');
title(sprintf('Left turn at %.1f deg virtual steer',chosenDeg));
export_fig(f,fullfile(outdir,'14_ideal_wheel_paths.png'));

% FIGURE 15: sensitivity to future measurements of wheelbase and steering angle.
Lvals=linspace(p.wheelbase_m*(1-p.wheelbaseFraction),p.wheelbase_m*(1+p.wheelbaseFraction),100);
Avals=linspace(max(1,chosenDeg-p.angleVariationDeg),chosenDeg+p.angleVariationDeg,100);
[A,L] = meshgrid(Avals,Lvals);
Rgrid = L./tand(A);
f=figure('Color','w','Position',[110 110 920 620]);
imagesc(Avals,Lvals*1000,Rgrid); set(gca,'YDir','normal'); colorbar;
xlabel('Equivalent steering angle (deg)'); ylabel('Wheelbase (mm)');
title('Turning radius sensitivity to geometry');
export_fig(f,fullfile(outdir,'15_wheelbase_steering_sensitivity.png'));
end

function p=user_inputs()
% EDIT THESE PARAMETERS AFTER MEASURING YOUR NEW CHASSIS.
p.wheelbase_m = 0.161;        % USER-REPORTED new chassis wheelbase, 22 Oct 2026
p.track_m = 0.117;            % USER-REPORTED front track, 22 Oct 2026
p.minimumSteeringDeg = 2;     % virtual front-wheel steering
p.maximumSteeringDeg = 35;    % EXAMPLE exploration bound
p.exampleSteeringDeg = 20;    % EXAMPLE virtual steering, NOT servo horn angle
p.referenceSpeed_mps = 0.5;   % EXAMPLE ONLY
p.pathSweepDeg = 120;         % path visualization sweep
p.wheelbaseFraction = 0.20;   % +/-20 percent hypothetical tolerance exploration
p.angleVariationDeg = 8;      % +/-8 degree sensitivity exploration
end

function validate_inputs(p)
assert(p.wheelbase_m>0 && p.track_m>0,'Wheelbase and track must be positive.');
assert(p.minimumSteeringDeg>0 && p.maximumSteeringDeg<85 && ...
    p.minimumSteeringDeg<p.maximumSteeringDeg,'Steering bounds must be 0 < min < max < 85 deg.');
assert(p.exampleSteeringDeg>=p.minimumSteeringDeg && ...
    p.exampleSteeringDeg<=p.maximumSteeringDeg,'Example steering outside limits.');
assert(p.wheelbase_m/tand(p.maximumSteeringDeg)>p.track_m/2,...
    'Geometry invalid: inner-turn centre not outside half track.');
assert(p.referenceSpeed_mps>=0 && p.pathSweepDeg>0 && p.pathSweepDeg<=360,...
    'Invalid speed or sweep.');
assert(p.wheelbaseFraction>0 && p.wheelbaseFraction<1 && p.angleVariationDeg>0,...
    'Invalid sensitivity settings.');
end

function export_fig(f,path)
set(f,'PaperPositionMode','auto');
print(f,path,'-dpng','-r160');
end
