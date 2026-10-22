% Mechanical loading and quasi-static rollover screening

function cg_stability()

clc; close all;
p = user_inputs();
validate_inputs(p);
outdir=fullfile(fileparts(mfilename('fullpath')),'results');
if ~exist(outdir,'dir'), mkdir(outdir); end

% Rear axle ground-contact midpoint is x=0; front axle at x=L.
% Assumes level ground, static equilibrium and negligible aero forces.
x=linspace(0,p.wheelbase_m,301);
frontFraction=x./p.wheelbase_m;
rearFraction=1-frontFraction;
x0=p.assumedCgFromRear_m;
front0=x0/p.wheelbase_m; rear0=1-front0;

% FIGURE 24: Fraction of gravity load on front and rear axles.
f1=figure('Color','w','Position',[100 80 1000 620],'Visible',p.figureVisible);
plot(1000*x,100*frontFraction,'LineWidth',2); hold on;
plot(1000*x,100*rearFraction,'LineWidth',2);
xline(1000*x0,'k--','Assumed CG location');
plot(1000*x0,100*front0,'ko','MarkerFaceColor','k');
plot(1000*x0,100*rear0,'ks','MarkerFaceColor','k');
axis([0 1000*p.wheelbase_m 0 100]); grid on;
xlabel('CG distance forward from rear axle (mm)');
ylabel('Fraction of vehicle static weight (%)');
title('Longitudinal axle-load distribution');
legend('Front axle','Rear axle','Location','best');
export_png(f1,fullfile(outdir,'24_axle_load_distribution.png'));

% FIGURE 25: idealized lateral tipping acceleration for hypothetical CG height.
% Single-track-width quasi-static rigid-body approximation:
% a_tip/g = T/(2*h). Not a grip, handling or safe driving speed prediction.
h=linspace(p.minCgHeight_m,p.maxCgHeight_m,300);
tip_g=p.track_m./(2*h);
baseTip=p.track_m/(2*p.assumedCgHeight_m);
f2=figure('Color','w','Position',[130 100 1000 620],'Visible',p.figureVisible);
plot(1000*h,tip_g,'LineWidth',2); hold on;
xline(1000*p.assumedCgHeight_m,'k--','Assumed CG height');
yline(baseTip,'--',sprintf('Illustrative %.2f g',baseTip));
plot(1000*p.assumedCgHeight_m,baseTip,'ko','MarkerFaceColor','k');
grid on;
xlabel('Assumed CG height above ground (mm)');
ylabel('Idealized lateral tipping threshold (multiples of g)');
title('Quasi-static tipping sensitivity to CG height');
export_png(f2,fullfile(outdir,'25_cg_height_tipping_sensitivity.png'));

% Archive parameters and calculated relationships; no text/CSV files.
save(fullfile(outdir,'simulation_inputs.mat'),'p','x','frontFraction', ...
    'rearFraction','h','tip_g','front0','rear0','baseTip');
fprintf('STEP 07 | SIMULATION ONLY -- NOT A PHYSICAL MEASUREMENT\n');
fprintf('Wheelbase: %.0f mm; track: %.0f mm (team reported).\n',1000*p.wheelbase_m,1000*p.track_m);
fprintf('Assumed CG X: %.1f mm; front/rear shares: %.1f%%/%.1f%%.\n', ...
  1000*x0,100*front0,100*rear0);
fprintf('Assumed CG height: %.1f mm; theoretical tipping: %.2f g.\n', ...
  1000*p.assumedCgHeight_m,baseTip);
fprintf('Two figures and simulation_inputs.mat saved to: %s\n',outdir);
fprintf('IMPORTANT: lateral grip and dynamic steering may limit motion earlier.\n');
end

function p=user_inputs()
% VERIFIED: team-reported geometry. Review against finished robot.
p.wheelbase_m=0.161;
p.frontTrack_m=0.117;
p.rearTrack_m=0.117;
p.track_m=min(p.frontTrack_m,p.rearTrack_m); % conservative track width
% ASSUMPTIONS -- replace when wheel loads and CG-height measurements exist.
p.assumedCgFromRear_m=0.0805;  % midpoint, NOT measured
p.assumedCgHeight_m=0.045;     % 45 mm, NOT measured
p.minCgHeight_m=0.020;         % hypothetical sensitivity range
p.maxCgHeight_m=0.090;
p.figureVisible='on';          % 'off' on a headless MATLAB session
end

function validate_inputs(p)
assert(isstruct(p) && isfield(p,'wheelbase_m') && isfield(p,'track_m'), ...
  'Run the complete step07_cg_stability file (not a single section).');
assert(p.wheelbase_m>0 && p.track_m>0,'Wheelbase/track must be positive.');
assert(p.assumedCgFromRear_m>=0 && p.assumedCgFromRear_m<=p.wheelbase_m, ...
  'CG position must lie between axle centers for this screening model.');
assert(p.minCgHeight_m>0 && p.maxCgHeight_m>p.minCgHeight_m, ...
  'Invalid hypothetical CG-height range.');
assert(p.assumedCgHeight_m>=p.minCgHeight_m && p.assumedCgHeight_m<=p.maxCgHeight_m, ...
  'Assumed CG height is outside the sensitivity range.');
end

function export_png(fig,path)
if exist('exportgraphics','file')==2
    exportgraphics(fig,path,'Resolution',180);
else
    print(fig,path,'-dpng','-r180');
end
end
