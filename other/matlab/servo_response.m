%% Steering servo response and control delay

clear; clc; close all;
p = struct();

%% USER INPUTS
% SOURCE: S0017M seller listing. No-load transit speeds; NOT loaded measurements.
p.servoSpeed_6V_secPer60 = 0.20;
p.servoSpeed_4p8V_secPer60 = 0.23;
% USER MEASUREMENT: wheelbase on partially completed YBR vehicle.
p.wheelbase_m = 0.161;
% ALL BELOW ARE SIMULATION ASSUMPTIONS; update after measuring.
p.dt_s = 0.005; p.duration_s = 5.0;
p.servoToWheelGain = 0.65;   % road-wheel degrees / servo-shaft degree (linear approximation)
p.maxServoAngle_deg = 60;    % demo mechanical range, NOT S0017M guarantee
p.deadTime_s = 0.05;         % controller + communication + drive delay (ASSUMED)
p.tau_s = 0.09;              % effective unloaded/loaded response lag (ASSUMED)
p.speedScaleLoaded = 0.70;  % loaded servo speed / advertised no-load speed (ASSUMED)
p.forwardSpeed_mps = 0.50;  % constant vehicle speed for yaw-rate illustration (ASSUMED)
p.referenceVoltage_V = 6.0; % rated-condition example, not measured supply voltage
p.figureVisible = 'on';
assert(p.dt_s>0 && p.duration_s>0 && p.tau_s>0 && p.wheelbase_m>0, 'Positive dt, duration, tau and wheelbase required');
assert(p.servoToWheelGain>0 && p.speedScaleLoaded>0 && p.speedScaleLoaded<=1, 'Check gain and loaded speed scale');
assert(p.deadTime_s>=0 && p.maxServoAngle_deg>0, 'Check delay and angle limit');
assert(p.referenceVoltage_V==6.0 || p.referenceVoltage_V==4.8, 'Choose 6.0 or 4.8 V (do not interpolate as a validated specification)');

outdir = fullfile(fileparts(mfilename('fullpath')), 'results');
if ~exist(outdir,'dir'), mkdir(outdir); end
if p.referenceVoltage_V==6.0
    secPer60 = p.servoSpeed_6V_secPer60;
else
    secPer60 = p.servoSpeed_4p8V_secPer60;
end
advertisedSpeed_degps = 60/secPer60;
loadedSpeed_degps = advertisedSpeed_degps*p.speedScaleLoaded;
t = (0:p.dt_s:p.duration_s)';
% Command to servo SHAFT. Symmetric step, return-to-centre and opposite turn.
servoCmd = zeros(size(t));
servoCmd(t>=0.5 & t<1.7) = 30;
servoCmd(t>=2.2 & t<3.2) = -25;
servoCmd(t>=3.7 & t<4.4) = 15;

% Cases: base, no controller delay, slower under load, higher response lag.
labels = {'Baseline assumptions','Zero command delay','Heavier servo load','Slower dynamics'};
cfg = [p.deadTime_s p.tau_s loadedSpeed_degps;
       0              p.tau_s loadedSpeed_degps;
       p.deadTime_s p.tau_s loadedSpeed_degps*0.55;
       p.deadTime_s p.tau_s*2 loadedSpeed_degps];
shaft = zeros(numel(t),size(cfg,1));
for k=1:size(cfg,1)
    shaft(:,k) = simulate_servo(t,servoCmd,p.dt_s,cfg(k,1),cfg(k,2),cfg(k,3),p.maxServoAngle_deg);
end
wheelTarget = p.servoToWheelGain*servoCmd;
wheel = p.servoToWheelGain*shaft;
err = wheelTarget - wheel;
yawRate_degps = rad2deg(p.forwardSpeed_mps * tand(wheel)/p.wheelbase_m);

% FIGURE 20: commanded and simulated steering, servo shaft and road-wheel.
f=figure('Visible',p.figureVisible,'Color','w','Position',[80 80 1050 730]);
subplot(2,1,1);
plot(t,servoCmd,'k--','LineWidth',1.6); hold on;
plot(t,shaft(:,1),'LineWidth',1.8); grid on;
ylabel('Servo shaft angle (deg)'); legend('Command','Simulated response','Location','best');
title('S0017M shaft response (assumed lag and loading)');
subplot(2,1,2);
plot(t,wheelTarget,'k--','LineWidth',1.6); hold on;
plot(t,wheel(:,1),'LineWidth',1.8); grid on;
xlabel('Time (s)'); ylabel('Equivalent road-wheel angle (deg)');
legend('Assumed target','Simulated','Location','best');
export_png(f,fullfile(outdir,'20_shaft_and_wheel_response.png'));

% FIGURE 21: design-case comparison.
f=figure('Visible',p.figureVisible,'Color','w','Position',[100 90 1050 630]);
plot(t,wheelTarget,'k--','LineWidth',1.5); hold on;
for k=1:size(wheel,2), plot(t,wheel(:,k),'LineWidth',1.4); end
grid on; xlabel('Time (s)'); ylabel('Equivalent road-wheel angle (deg)');
legend([{'Target'},labels],'Location','bestoutside');
title('Sensitivity to assumed delay, lag and load');
export_png(f,fullfile(outdir,'21_scenario_comparison.png'));

% FIGURE 22: absolute tracking-error time series and conservative peak error.
f=figure('Visible',p.figureVisible,'Color','w','Position',[120 100 1000 620]);
for k=1:size(err,2), plot(t,abs(err(:,k)),'LineWidth',1.4); hold on; end
grid on; xlabel('Time (s)'); ylabel('Absolute steering-angle error (deg)');
legend(labels,'Location','bestoutside'); title('Tracking error (target minus model)');
export_png(f,fullfile(outdir,'22_steering_error.png'));

% FIGURE 23: illustrative yaw rate caused by modeled steering at constant speed.
f=figure('Visible',p.figureVisible,'Color','w','Position',[140 110 1000 620]);
plot(t,rad2deg(p.forwardSpeed_mps*tand(wheelTarget)/p.wheelbase_m),'k--','LineWidth',1.5); hold on;
plot(t,yawRate_degps(:,1),'LineWidth',1.8); grid on;
xlabel('Time (s)'); ylabel('Yaw rate (deg/s)');
legend('Ideal wheel-command yaw rate','Simulated baseline yaw rate','Location','best');
title('Ideal kinematic yaw-rate consequence');
export_png(f,fullfile(outdir,'23_yaw_rate_effect.png'));

function response = simulate_servo(t,cmd,dt,deadTime,tau,maxSpeed,maxAngle)
    % Empirical first-order response to delayed setpoint with speed saturation.
    % Does not model servo control electronics, inertia, friction or real load.
    response=zeros(size(cmd));
    for i=2:numel(t)
        delayedT=t(i)-deadTime;
        % Keep initial zero command until time begins; use previous command sample.
        if delayedT<=t(1)
            target=cmd(1);
        else
            j=find(t<=delayedT,1,'last');
            target=cmd(j);
        end
        target=min(max(target,-maxAngle),maxAngle);
        desiredRate=(target-response(i-1))/tau;
        limitedRate=min(max(desiredRate,-maxSpeed),maxSpeed);
        response(i)=response(i-1)+dt*limitedRate;
        response(i)=min(max(response(i),-maxAngle),maxAngle);
    end
end

function export_png(fig,path)
    if exist('exportgraphics','file')==2
        exportgraphics(fig,path,'Resolution',180);
    else
        print(fig,path,'-dpng','-r180');
    end
end
