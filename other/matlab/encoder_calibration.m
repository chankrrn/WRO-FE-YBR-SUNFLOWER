%% Encoder count calibration and wheel speed

function encoder_calibration()

clc; close all;
p = inputs();
validateattributes(p.wheelRadius_m,{'numeric'},{'scalar','positive'});
validateattributes(p.gearRatio,{'numeric'},{'scalar','positive'});
if ~exist(fullfile(fileparts(mfilename('fullpath')),'results'),'dir')
    mkdir(fullfile(fileparts(mfilename('fullpath')),'results'));
end
outdir=fullfile(fileparts(mfilename('fullpath')),'results');
% The count interpretation MUST include counting mode and encoder location.
% Counts are measured at drivetrain wheel: this avoids unverified CPR assumptions.
if p.useMeasuredData
    assert(numel(p.testCounts)==numel(p.testSeconds) && numel(p.testCounts)==numel(p.referenceSpeed_mps),...
        'Measured arrays must have the same length.');
    assert(all(p.testSeconds>0),'Each measurement interval must be positive.');
    assert(p.countsPerWheelRev>0,'Measure and enter countsPerWheelRev before enabling measured mode.');
    counts=p.testCounts; seconds=p.testSeconds; reference=p.referenceSpeed_mps;
    label='MEASURED INPUTS (verify reference instrument)';
else
    % Illustrative known wheel revolutions per time; a notional pulse factor
    % generates demonstration counts. No physical measurements are implied.
    wheelRPM=[100 200 300 400 500 600];
    seconds=2*ones(size(wheelRPM));
    assumedCountsPerWheelRev=400; % illustrative, NOT a hardware specification
    counts=round(wheelRPM/60 .* seconds * assumedCountsPerWheelRev);
    reference=wheelRPM/60 * (2*pi*p.wheelRadius_m);
    label='SIMULATION / SYNTHETIC DEMONSTRATION ONLY';
    p.countsPerWheelRev=assumedCountsPerWheelRev;
end
revolutions=counts./p.countsPerWheelRev;
wheelRPMest=60*revolutions./seconds;
speed=wheelRPMest/60 * 2*pi*p.wheelRadius_m;
err=speed-reference;
figure('Color','w','Position',[100 100 1000 560]);
plot(reference,speed,'o-','LineWidth',1.8,'MarkerSize',7); hold on;
lo=min([reference(:);speed(:)]); hi=max([reference(:);speed(:)]);
if hi==lo, hi=lo+1; end
plot([lo hi],[lo hi],'k--','LineWidth',1.2);
grid on; axis equal;
xlabel('Reference speed (m/s)'); ylabel('Encoder-estimated speed (m/s)');
title(['YBR-SUNFLOWER | ' label]);
legend('Encoder estimate','Ideal agreement','Location','northwest');
exportgraphics(gcf,fullfile(outdir,'28_encoder_speed_calibration.png'),'Resolution',180);
fprintf('\nSTEP 09: %s\n',label);
fprintf('Wheel radius: %.1f mm (provisional rolling radius)\n',p.wheelRadius_m*1000);
fprintf('CPR used: %.1f counts/wheel revolution (%s)\n',p.countsPerWheelRev, ternary(p.useMeasuredData,'TEAM-ENTERED','ASSUMED'));
fprintf('Mean absolute speed difference: %.4f m/s\n',mean(abs(err)));
fprintf('No summary.txt or CSV files created.\n');
end
function p=inputs()
p.wheelRadius_m=0.024; % user-reported; loaded rear wheel radius TODO
p.gearRatio=19; % motor internal reduction; do not reapply to wheel counts
p.useMeasuredData=false; % change to true only after data collection
p.countsPerWheelRev=NaN; % TODO measure effective counts/wheel rev
p.testCounts=[ ];         % TODO each interval's encoder counts
p.testSeconds=[ ];        % TODO duration corresponding to counts [s]
p.referenceSpeed_mps=[ ]; % TODO independent measured speed [m/s]
end
function s=ternary(c,a,b)
if c, s=a; else, s=b; end
end
