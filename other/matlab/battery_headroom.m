%% Battery discharge and 12-V rail headroom

% fuse sizes, cell balancing, BMS cutoff, or 12-V regulation quality.

clear; clc; close all;
root = fileparts(mfilename('fullpath'));
if isempty(root), root = pwd; end
out = fullfile(root,'results');
if ~exist(out,'dir'), mkdir(out); end

%% USER INPUTS
p.cells = 4;
p.capacityAh = 1.30;       % GNB battery label: 1300 mAh
p.nominalPackV = 15.2;     % GNB battery label; not a real discharge curve
p.initialSoc = 1.00;      % hypothetical full initial state of charge
p.stepSeconds = 1;        % simulation time step
p.maxHours = 2;

% BELOW is ILLUSTRATIVE open-circuit voltage profile.
% Columns MUST be sorted from 0 to 1 in state of charge.
p.socKnots = [0 .10 .20 .30 .40 .50 .60 .70 .80 .90 1.00];
p.ocvPerCellV = [3.30 3.55 3.66 3.72 3.76 3.80 3.84 3.90 4.00 4.15 4.35];

% Illustrative pack equivalent series resistance, not measured.
p.packResistanceOhm = 0.12;
% These load/efficiency parameters are hypothetical and require measurements.
p.power5V_W = 12;         % 5-V rail total output power [W], ASSUMED
p.power12V_W = 18;        % 12-V rail total output power [W], ASSUMED
p.eta5V = .90;            % constant buck efficiency, ASSUMED
p.eta12V = .90;           % constant buck efficiency, ASSUMED
p.min12InputV = 15.0;     % DFR1208 advertised 15–24 V INPUT: verify revision
p.min5InputV = 6.0;      % DFR1202 published 6-24 V input range
p.screenStopV = 13.2;    % DEMONSTRATION STOP only; NOT a safe cutoff recommendation
p.showUncertaintyV = 0.25; % illustrative uncertainty band around 15-V threshold

% Scenarios vary electrical demands. Values are scaling factors, NOT test data.
scenarioNames = {'Light 0.5x','Nominal 1.0x','Heavy 1.5x'};
scenarioScale = [.5 1 1.5];

%% Check inputs
assert(p.cells==4 && p.capacityAh>0 && p.stepSeconds>0);
assert(p.initialSoc>0 && p.initialSoc<=1);
assert(all(diff(p.socKnots)>0) && p.socKnots(1)==0 && p.socKnots(end)==1);
assert(all(diff(p.ocvPerCellV)>0),'Expected increasing example OCV curve.');
assert(p.packResistanceOhm>=0 && all(scenarioScale>0));
assert(p.eta5V>0 && p.eta5V<=1 && p.eta12V>0 && p.eta12V<=1);

%% Simulation: SOC = integral of delivered battery-side current
% P_battery = P_5V/eta_5V + P_12V/eta_12V.
% Battery terminal voltage Vt = OCV(SOC) - I*R.
% With fixed requested input power P=Vt*I, solve R*I^2 - OCV*I + P=0.
% Lower-root I = 2*P/(OCV + sqrt(OCV^2 - 4*R*P))
% Equivalent algebraically to (OCV-sqrt(...))/(2*R), but numerically stable.
N = numel(scenarioScale); runs = cell(1,N);
for k = 1:N
    requestedBatteryPowerW = scenarioScale(k) * ...
        (p.power5V_W/p.eta5V + p.power12V_W/p.eta12V);
    steps = ceil(p.maxHours*3600/p.stepSeconds)+1;
    t = nan(steps,1); soc = nan(steps,1); ocv = nan(steps,1);
    terminalV = nan(steps,1); currentA = nan(steps,1);
    t(1) = 0; soc(1) = p.initialSoc;
    reason = 'Max simulation time reached';
    for j = 1:steps
        ocv(j) = p.cells*interp1(p.socKnots,p.ocvPerCellV,soc(j),'linear');
        d = ocv(j)^2 - 4*p.packResistanceOhm*requestedBatteryPowerW;
        if d <= 0
            reason = 'Infeasible assumed power/resistance combination';
            break;
        end
        currentA(j) = 2*requestedBatteryPowerW / (ocv(j)+sqrt(d));
        terminalV(j) = ocv(j) - p.packResistanceOhm*currentA(j);
        if terminalV(j) <= p.screenStopV
            reason = 'Illustrative screening stop voltage reached';
            break;
        end
        if soc(j) <= 0
            reason = 'Hypothetical battery state of charge exhausted';
            break;
        end
        if j == steps, break; end
        nextSoc = soc(j)-currentA(j)*p.stepSeconds/(3600*p.capacityAh);
        if nextSoc < 0, nextSoc = 0; end
        soc(j+1) = nextSoc;
        t(j+1) = t(j)+p.stepSeconds;
    end
    keep = isfinite(terminalV);
    t=t(keep); soc=soc(keep); ocv=ocv(keep);
    terminalV=terminalV(keep); currentA=currentA(keep);
    firstBelow = find(terminalV<p.min12InputV,1,'first');
    if isempty(firstBelow), headroomTimeMin=NaN;
    else, headroomTimeMin=t(firstBelow)/60; end
    runs{k}=struct('t',t,'soc',soc,'ocv',ocv,'V',terminalV,...
        'I',currentA,'firstBelowMin',headroomTimeMin,...
        'requestedPowerW',requestedBatteryPowerW,'stopReason',reason);
end

%% Figure 04: Illustrative voltage model versus SOC
f4=figure('Color','w','Position',[70 90 1000 560]);
socAxis=linspace(0,1,400);
ocvAxis=p.cells*interp1(p.socKnots,p.ocvPerCellV,socAxis,'linear');
plot(100*socAxis,ocvAxis,'LineWidth',2); hold on;
yline(p.min12InputV,'--','12-V buck stated minimum input','LineWidth',1.4);
yline(p.min12InputV+p.showUncertaintyV,':','Illustrative margin +0.25 V');
yline(p.nominalPackV,'-.','Battery nominal 15.2 V');
xlabel('Illustrative state of charge (%)'); ylabel('Modeled open-circuit battery voltage (V)');
title('ASSUMED LiHV OCV shape');
grid on; legend('Illustrative OCV curve','Location','northwest');
exportgraphics(f4,fullfile(out,'04_assumed_voltage_vs_soc.png'),'Resolution',180);

%% Figure 05: Three illustrative constant-load scenarios
f5=figure('Color','w','Position',[90 110 1030 580]); hold on;
for k=1:N
    plot(runs{k}.t/60,runs{k}.V,'LineWidth',2,'DisplayName',scenarioNames{k});
end
yline(p.min12InputV,'k--','DFR1208 minimum INPUT (verify)','LineWidth',1.25);
xlabel('Elapsed simulated time (min)'); ylabel('Modeled battery terminal voltage (V)');
title('Battery voltage under HYPOTHETICAL loads'); grid on; legend('Location','northeast');
exportgraphics(f5,fullfile(out,'05_voltage_vs_time_scenarios.png'),'Resolution',180);

%% Figure 06: Nominal hypothetical current and SOC
r = runs{2};
f6=figure('Color','w','Position',[110 100 1050 580]);
yyaxis left; plot(r.t/60,100*r.soc,'LineWidth',2);
ylabel('Modeled SOC (%)'); ylim([0 105]);
yyaxis right; plot(r.t/60,r.I,'LineWidth',2);
ylabel('Estimated battery current (A)');
xlabel('Elapsed simulated time (min)');
title('Modeled charge use & current: nominal EXAMPLE load'); grid on;
exportgraphics(f6,fullfile(out,'06_soc_and_current.png'),'Resolution',180);

%% Figure 07: Input headroom (NOT output voltage)
f7=figure('Color','w','Position',[130 130 1000 550]); hold on;
plot(r.t/60,r.V-p.min12InputV,'LineWidth',2);
yline(0,'k--','Below zero = outside stated input range','LineWidth',1.2);
xlabel('Elapsed simulated time (min)'); ylabel('12-V converter input headroom (V)');
title('DFR1208 input headroom, nominal ASSUMED load'); grid on;
exportgraphics(f7,fullfile(out,'07_12v_input_headroom.png'),'Resolution',180);
