%% Simulated body swept envelope and steering-clearance screening.
% Base MATLAB; no measured steering angles yet. All outputs are SIMULATION.

function swept_envelope()

close all; clc;
p = inputs(); validate(p);
outdir=fullfile(fileparts(mfilename('fullpath')),'results');
if ~exist(outdir,'dir'), mkdir(outdir); end

% Coordinate system: rear axle centre at (0,0), +X forward, +Y left.
% Rectangular body approximation extends rear/front from rear axle using
% reported wheelbase and approximate overhang; NOT a true CAD outline.
rearX=-p.rearOverhang;
frontX=p.wheelbase+p.frontOverhang;
halfW=p.width/2;
poly=[rearX -halfW; frontX -halfW; frontX halfW; rearX halfW; rearX -halfW];
angles=deg2rad(linspace(0,p.sweepDeg,p.numSamples));
delta=p.assumedSteeringDeg;
R=p.wheelbase/tand(delta);
xc=R*sin(angles); yc=R*(1-cos(angles));
pts=zeros(size(poly,1),numel(angles),2);
for k=1:numel(angles)
    Q=[cos(angles(k)) -sin(angles(k));sin(angles(k)) cos(angles(k))];
    v=(Q*poly.').'; v(:,1)=v(:,1)+xc(k); v(:,2)=v(:,2)+yc(k);
    pts(:,k,1)=v(:,1); pts(:,k,2)=v(:,2);
end
fig=figure('Color','w','Position',[80 80 900 650]); hold on;
plot(xc,yc,'k--','LineWidth',1.6,'DisplayName','Rear axle midpoint');
for k=round(linspace(1,numel(angles),8))
    patch(pts(:,k,1),pts(:,k,2),[0.2 0.5 0.8],'FaceAlpha',.08,'EdgeColor',[.2 .45 .75],'HandleVisibility','off');
end
for q=1:4
    plot(squeeze(pts(q,:,1)),squeeze(pts(q,:,2)),'LineWidth',1.2,'DisplayName',sprintf('Body corner %d',q));
end
axis equal; grid on; xlabel('X (m)'); ylabel('Y (m)');
title('Vehicle corner trajectories during a left turn');
legend('Location','bestoutside'); hold off;
saveas(fig,fullfile(outdir,'26_swept_body_paths.png'));

% Conservative outermost radial envelope relative to ideal instantaneous
% centre of rotation at (0,R). For any fixed body point, radius remains
% constant as rigid body rotates about turn centre.
steer=linspace(p.minSteeringDeg,p.maxSteeringDeg,180);
Rvals=p.wheelbase./tand(steer);
outer=zeros(size(steer)); inner=zeros(size(steer));
for k=1:numel(steer)
    radii=sqrt(poly(1:4,1).^2+(poly(1:4,2)-Rvals(k)).^2);
    outer(k)=max(radii); inner(k)=min(radii);
end
fig=figure('Color','w','Position',[90 90 900 590]);
plot(steer,outer*1000,'LineWidth',2); hold on;
plot(steer,inner*1000,'LineWidth',2);
yline(p.width*1000/2,':','Half vehicle width (reference)');
xline(delta,'--','Example steering input');
grid on; xlabel('Equivalent steering angle (degrees)');
ylabel('Radial distance from turn centre (mm)');
title('SIMULATED — Inner/outer rigid-body turn envelope');
legend('Outermost body corner','Innermost body corner','Half body width','Example steer','Location','best');
saveas(fig,fullfile(outdir,'27_turn_envelope_vs_steering.png'));

fprintf('\nSTEP 08: SIMULATED GEOMETRY SCREENING\n');
fprintf('Wheelbase %.1f mm, width %.1f mm, front/rear overhang %.1f/%.1f mm\n',p.wheelbase*1000,p.width*1000,p.frontOverhang*1000,p.rearOverhang*1000);
fprintf('Assumed steering %.1f degrees: rear midpoint radius %.1f mm\n',delta,R*1000);
fprintf('Computed outer/inner radial envelope %.1f/%.1f mm\n',interp1(steer,outer,delta)*1000,interp1(steer,inner,delta)*1000);
fprintf('Important: body box, steering angle and overhang remain unvalidated for final assembly.\n');
save(fullfile(outdir,'simulation_inputs.mat'),'p','steer','outer','inner','poly');
end
function p=inputs()
p.wheelbase=0.161; % m TEAM REPORTED
p.width=0.135; % m TEAM REPORTED provisional (track/tire discrepancy)
p.frontOverhang=0.025; % m APPROXIMATE; camera not mounted
p.rearOverhang=0.025; % m TEAM REPORTED
p.assumedSteeringDeg=20; % degrees ASSUMED, NOT MEASURED
p.minSteeringDeg=8; p.maxSteeringDeg=35; % degrees ASSUMED RANGE
p.sweepDeg=110; % degrees illustrative trajectory
p.numSamples=250;
end
function validate(p)
assert(p.wheelbase>0 && p.width>0 && p.frontOverhang>=0 && p.rearOverhang>=0);
assert(p.minSteeringDeg>0 && p.maxSteeringDeg<80 && p.minSteeringDeg<p.assumedSteeringDeg && p.assumedSteeringDeg<p.maxSteeringDeg);
assert(p.numSamples>=20 && p.sweepDeg>0 && p.sweepDeg<=360);
end
