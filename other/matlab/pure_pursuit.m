%% Pure Pursuit path-tracking screening on a rounded-square reference route.

function pure_pursuit()

clc; close all;
p = user_inputs();
validate_inputs(p);

root = fileparts(mfilename('fullpath'));
if isempty(root), root = pwd; end
outdir = fullfile(root,'results');
if ~exist(outdir,'dir'), mkdir(outdir); end

[reference, sNodes, perimeter] = make_rounded_square(p);
caseNames = {'Short lookahead','Long lookahead'};
runs = cell(1,2);
for caseIdx = 1:2
    runs{caseIdx} = simulate_case(p, reference, sNodes, perimeter, ...
        p.lookahead_m(caseIdx));
end

% ONE composite figure: trajectory on illustrative field + signed tracking error.
f = figure('Color','w','Position',[60 60 1450 650]);
subplot(1,2,1);
hold on; axis equal; grid on;
% Simplified field geometry, not a complete competition/obstacle layout.
a = p.fieldHalfWidth_m;
inner = p.innerWallHalfWidth_m;
plot([-a a a -a -a],[-a -a a a -a],'k-','LineWidth',1.6);
fill([-inner inner inner -inner],[-inner -inner inner inner], ...
    [0.82 0.82 0.82],'EdgeColor','k','LineWidth',1.0);
plot([reference(:,1); reference(1,1)], ...
     [reference(:,2); reference(1,2)], 'k--','LineWidth',1.1);
h1 = plot(runs{1}.x, runs{1}.y,'LineWidth',1.8);
h2 = plot(runs{2}.x, runs{2}.y,'LineWidth',1.8);
plot(p.startX_m,p.startY_m,'ko','MarkerFaceColor','y','MarkerSize',7);
xlabel('X [m]'); ylabel('Y [m]');
title('SIMULATED trajectory | ideal localization, no pillars');
legend([h1 h2],{sprintf('Lookahead %.2f m',p.lookahead_m(1)), ...
       sprintf('Lookahead %.2f m',p.lookahead_m(2))}, ...
       'Location','southoutside');
xlim([-a-.1 a+.1]); ylim([-a-.1 a+.1]);

subplot(1,2,2);
plot(runs{1}.time_s,1000*runs{1}.signedError_m,'LineWidth',1.6); hold on;
plot(runs{2}.time_s,1000*runs{2}.signedError_m,'LineWidth',1.6);
plot([0 runs{1}.time_s(end)],[0 0],'k--','LineWidth',0.9);
grid on;
xlabel('Time [s]'); ylabel('Signed centerline error [mm]');
title('SIMULATED cross-track error | negative = route right');
legend({sprintf('Lookahead %.2f m',p.lookahead_m(1)), ...
        sprintf('Lookahead %.2f m',p.lookahead_m(2))}, ...
        'Location','northeast');

annotation(f,'textbox',[0.05 0.945 0.9 0.045], ...
    'String','YBR-SUNFLOWER | Pure Pursuit parameter comparison (assumption-based)', ...
    'EdgeColor','none','HorizontalAlignment','center', ...
    'FontSize',12,'FontWeight','bold');
figureFile = fullfile(outdir,'29_pure_pursuit_tracking.png');
print(f,figureFile,'-dpng','-r160');
save(fullfile(outdir,'simulation_inputs.mat'),'p','perimeter','reference');

fprintf('\nYBR-SUNFLOWER | Step 10 - pure pursuit (SIMULATION ONLY)\n');
fprintf('Wheelbase = %.0f mm (team-reported); speed = %.2f m/s (assumed)\n', ...
    1000*p.wheelbase_m,p.speed_mps);
fprintf('Reference loop length = %.3f m (illustrative rounded-square path)\n',perimeter);
for k = 1:2
    err = runs{k}.signedError_m;
    fprintf('%s (%.2f m): RMS |cross-track| %.1f mm; max |cross-track| %.1f mm; steering saturation %.1f%%\n', ...
      caseNames{k},p.lookahead_m(k),1000*sqrt(mean(err.^2)), ...
      1000*max(abs(err)),100*mean(runs{k}.saturated));
end
fprintf('Created: %s\n',figureFile);
fprintf('No physical test or software implementation is validated by these results.\n\n');
end

function p = user_inputs()
% -------- USER INPUTS --------
% Confirmed physical geometry (reported 2026-10-22)
p.wheelbase_m = 0.161;
p.frontTrack_m = 0.117;            % Documentation only; not used in bicycle model
p.robotLength_m = 0.211;          % Provisional (front camera not final)
p.robotWidth_m = 0.135;           % Provisional, remeasure outside tire edges
% Hypothetical simplified reference route, NOT an official randomized field
p.fieldHalfWidth_m = 1.6;        % Nominal outer extent, illustrative
p.innerWallHalfWidth_m = 0.5;    % Nominal inner wall, illustrative
p.centerlineHalfExtent_m = 1.05;
p.cornerRadius_m = 0.45;
p.pathSpacing_m = 0.01;
% Control parameters: NOT yet measured on or adopted by the final robot
p.lookahead_m = [0.30 0.55];
p.speed_mps = 0.35;
p.maxSteering_deg = 35;           % ASSUMED equivalent road-wheel limit
p.steeringTimeConstant_s = 0.10; % ASSUMED actuator lag
p.commandDelay_s = 0.04;        % ASSUMED controller/communication delay
p.maxSteeringRate_deg_s = 180;   % ASSUMED road-wheel slew limit
p.dt_s = 0.02;
p.extraTime_s = 0.8;
% Start pose: right-side reference straight, heading up (+Y).
p.startX_m = p.centerlineHalfExtent_m + 0.06; % 60 mm initial offset
p.startY_m = -(p.centerlineHalfExtent_m - p.cornerRadius_m);
p.startHeading_deg = 87;        % 3 deg heading perturbation from +Y
end

function validate_inputs(p)
assert(p.wheelbase_m > 0 && p.speed_mps > 0 && p.dt_s > 0, ...
    'Wheelbase, speed and timestep must be positive.');
assert(all(p.lookahead_m > 0),'All lookahead distances must be positive.');
assert(p.cornerRadius_m > 0 && p.centerlineHalfExtent_m > p.cornerRadius_m, ...
    'Invalid rounded-square reference dimensions.');
assert(p.maxSteering_deg > 0 && p.maxSteering_deg < 89, ...
    'Equivalent steering limit must be between 0 and 89 degrees.');
assert(p.steeringTimeConstant_s > 0 && p.commandDelay_s >= 0, ...
    'Invalid steering delay/response time.');
assert(p.maxSteeringRate_deg_s > 0, 'Steering slew limit must be positive.');
assert(p.pathSpacing_m > 0 && p.extraTime_s >= 0, ...
    'Path spacing and extra time must be nonnegative/positive.');
end

function [xy, s, totalLength] = make_rounded_square(p)
% CCW closed rounded-square centerline, with corners of radius r.
a = p.centerlineHalfExtent_m;
r = p.cornerRadius_m; b = a-r;
nL = max(10,ceil(2*b/p.pathSpacing_m));
nA = max(10,ceil((pi*r/2)/p.pathSpacing_m));
xy = zeros(0,2);
% Right straight, heading +Y
v = linspace(-b,b,nL+1)'; v(end)=[];
xy = [xy; a*ones(size(v)), v]; %#ok<AGROW>
% Top-right circular fillet
ang = linspace(0,pi/2,nA+1)'; ang(end)=[];
xy = [xy; b+r*cos(ang),b+r*sin(ang)]; %#ok<AGROW>
% Top straight, heading -X
v = linspace(b,-b,nL+1)'; v(end)=[];
xy = [xy; v,a*ones(size(v))]; %#ok<AGROW>
% Top-left fillet
ang = linspace(pi/2,pi,nA+1)'; ang(end)=[];
xy = [xy; -b+r*cos(ang), b+r*sin(ang)]; %#ok<AGROW>
% Left straight, heading -Y
v = linspace(b,-b,nL+1)'; v(end)=[];
xy = [xy; -a*ones(size(v)),v]; %#ok<AGROW>
% Bottom-left fillet
ang = linspace(pi,3*pi/2,nA+1)'; ang(end)=[];
xy = [xy; -b+r*cos(ang),-b+r*sin(ang)]; %#ok<AGROW>
% Bottom straight, heading +X
v = linspace(-b,b,nL+1)'; v(end)=[];
xy = [xy; v,-a*ones(size(v))]; %#ok<AGROW>
% Bottom-right fillet
ang = linspace(3*pi/2,2*pi,nA+1)'; ang(end)=[];
xy = [xy; b+r*cos(ang),-b+r*sin(ang)]; %#ok<AGROW>
xyClosed = [xy;xy(1,:)];
segment = sqrt(sum(diff(xyClosed,1,1).^2,2));
assert(all(segment > 0),'Reference route contains duplicate consecutive points.');
s = [0;cumsum(segment)];
totalLength=s(end);
end

function r = simulate_case(p, xy, s, totalLength, lookahead_m)
N = floor(((totalLength/p.speed_mps)+p.extraTime_s)/p.dt_s)+1;
time=(0:N-1)'*p.dt_s;
x=zeros(N,1); y=zeros(N,1); psi=zeros(N,1); delta=zeros(N,1);
cmd=zeros(N,1); err=zeros(N,1); saturated=false(N,1);
x(1)=p.startX_m; y(1)=p.startY_m;
psi(1)=p.startHeading_deg*pi/180;
maxAngle=p.maxSteering_deg*pi/180;
maxRate=p.maxSteeringRate_deg_s*pi/180;
delaySteps=round(p.commandDelay_s/p.dt_s);
refCount=size(xy,1);
for k = 1:N
    distances=(xy(:,1)-x(k)).^2+(xy(:,2)-y(k)).^2;
    [~,idx]=min(distances);
    im=mod(idx-2,refCount)+1;
    ip=mod(idx,refCount)+1;
    tangent=xy(ip,:)-xy(im,:);
    tangent=tangent/(hypot(tangent(1),tangent(2))+eps);
    diffVehicle=[x(k)-xy(idx,1), y(k)-xy(idx,2)];
    err(k)=tangent(1)*diffVehicle(2)-tangent(2)*diffVehicle(1);

    sTarget=mod(s(idx)+lookahead_m,totalLength);
    % s includes the duplicate endpoint, so interpolation is continuous at wrap.
    xTarget=interp1(s,[xy(:,1);xy(1,1)],sTarget,'linear');
    yTarget=interp1(s,[xy(:,2);xy(1,2)],sTarget,'linear');
    relativeAngle=atan2(yTarget-y(k),xTarget-x(k))-psi(k);
    alpha=atan2(sin(relativeAngle),cos(relativeAngle));
    targetDistance=max(hypot(xTarget-x(k),yTarget-y(k)),1e-6);
    curvature=2*sin(alpha)/targetDistance;
    rawCmd=atan(p.wheelbase_m*curvature);
    cmd(k)=min(max(rawCmd,-maxAngle),maxAngle);
    saturated(k)=abs(rawCmd)>maxAngle;

    if k==N, break; end
    delayIndex=k-delaySteps;
    if delayIndex < 1
        delayed=0;
    else
        delayed=cmd(delayIndex);
    end
    requestRate=(delayed-delta(k))/p.steeringTimeConstant_s;
    actualRate=min(max(requestRate,-maxRate),maxRate);
    delta(k+1)=min(max(delta(k)+p.dt_s*actualRate,-maxAngle),maxAngle);
    % Ideal kinematic bicycle model. No lateral tire slip or obstacle logic.
    x(k+1)=x(k)+p.dt_s*p.speed_mps*cos(psi(k));
    y(k+1)=y(k)+p.dt_s*p.speed_mps*sin(psi(k));
    psi(k+1)=psi(k)+p.dt_s*p.speed_mps*tan(delta(k))/p.wheelbase_m;
end
r=struct('x',x,'y',y,'time_s',time,'signedError_m',err, ...
         'saturated',saturated,'steering_rad',delta,'command_rad',cmd);
end
