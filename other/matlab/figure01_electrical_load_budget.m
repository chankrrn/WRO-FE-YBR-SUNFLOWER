%% Electrical figure 01/11
% ELECTRICAL LOAD BUDGET - supplier ratings + explicit design scenarios
% WRO 2026 Future Engineers | Engineering Journal 02: Power & Sensor Architecture
%
% This is a PLANNING CALCULATION, not a measured current/power trace.
% Self-contained. Requires base MATLAB R2020a+ (exportgraphics).
% Run as a complete script. Do not use Run Section without the inputs above.
% Outputs: results/01_assumed_load_budget.png, simulation_inputs.mat.
% No CSV or summary.txt files are created.
%
% SOURCE STATUS:
%   Manufacturer/supplier: GNB pack nominal 15.2 V;
%     DFRobot RPLIDAR S2L working current 400 mA at 5 V;
%     DFRobot SEN0253 5 mA at 3.3-5 V;
%     CHP-20GP-180 supplier rated current <=550 mA at 12 V.
%   Design scenario (NOT verified for the installed robot):
%     Pi 5+camera+active cooler 2 A at 5 V;
%     both HAT logic 0.12 A at 5 V;
%     S0017M no-load representative 0.15 A at 5 V (user-supplied generic range);
%     LED button 0.02 A at 3.3 V; battery voltmeter 0.015 A at 15.2 V.
%   The user-supplied S0017M 0.8-1.2 A stall range is a planning envelope,
%   not a verified model-specific published stall-current limit.
%
% Power tree assumption: 5-V buck -> Pi 5V and HATs; Pi 3.3V derived
% supply -> sensor/button. DFR0604 servo PWM + is at nominal 5 V ONLY if
% external VP is unpowered. This assumption must not be used as wiring advice.
% 12-V buck -> DFR0592 motor supply. OLED voltmeter uses switched pack.
%
% Buck efficiency at actual pack/load conditions is unknown. eta=0.90 is
% a SCENARIO value, NOT a guaranteed converter performance specification.

clearvars; close all; clc;

%% 1. USER INPUTS -- adjust only when justified by a better source
inputs.project = 'YBR-SUNFLOWER | WRO 2026 Future Engineers';
inputs.modelType = 'Calculated planning scenario; NOT measured data';
inputs.batteryNominalV = 15.2;     % GNB 4S LiHV manufacturer nominal [V]
inputs.batteryCapacityAh = 1.30;  % GNB battery nameplate [Ah]
inputs.fiveV = 5.0;               % DFR1202 published output nominal [V]
inputs.twelveV = 12.0;            % DFR1208 published output nominal [V]
inputs.threeV3 = 3.3;             % DFR0604 I/O breakout nominal [V]
inputs.eta5 = 0.90;               % MODEL assumption, not guaranteed at load
inputs.eta12 = 0.90;              % MODEL assumption, not guaranteed at load

% The HAT's servo PWM positive rail is taken as 5 V nominal only when
% DFR0604 external VP supply is NOT connected. The assembled VP state is
% a team-specific physical configuration, not inferred from a datasheet.
inputs.servoUses5VviaHAT = true;   % Change only if wiring differs.
inputs.servoCurrentA = 0.150;     % representative NO-LOAD planning case
inputs.servoHighScenarioA = 1.20; % user-supplied STALL planning envelope upper

% Controller/peripherals: do not equate PSU recommendations with load draw.
inputs.piCameraCoolerCurrentA = 2.00; % MODELED combined operating load
inputs.hatElectronicsCurrentA = 0.12; % MODELED both HAT logic loads
inputs.ledButtonCurrentA = 0.020;    % MODELED 3.3 V button demand
inputs.oledVoltmeterCurrentA = 0.015;% MODELED from switched pack

% Published/specification-derived scenario points.
inputs.lidarWorkingCurrentA = 0.400; % DFRobot RPLIDAR S2L (working, 5 V)
inputs.imuCurrentA = 0.005;          % DFRobot SEN0253, ~5 mA
inputs.motorRatedUpperA = 0.550;     % CHP motor supplier: <=0.55 A rated
% Do NOT use the motor's stall rating as a normal continuous load point.

%% 2. Build component inventory (power is V * I at each rail)
% Sources: 'PUBLISHED' means nominal current/maximum-at-rated value is
% reported by the maker/supplier. 'SCENARIO' is explicitly assumed.
items = { ...
 'Pi 5 + OV5647 + active cooler',  inputs.fiveV,                inputs.piCameraCoolerCurrentA, 'SCENARIO', '5 V'; ...
 'CHP-20GP-180 motor (rated bound)', inputs.twelveV,           inputs.motorRatedUpperA,      'PUBLISHED', '12 V'; ...
 'RPLIDAR S2L (working)',           inputs.fiveV,               inputs.lidarWorkingCurrentA,  'PUBLISHED', '5 V'; ...
 'DFR0592 + DFR0604 logic',         inputs.fiveV,               inputs.hatElectronicsCurrentA,'SCENARIO', '5 V'; ...
 'S0017M servo (no-load scenario)', inputs.fiveV,               inputs.servoCurrentA,         'SCENARIO', '5 V'; ...
 'Battery voltmeter',               inputs.batteryNominalV,     inputs.oledVoltmeterCurrentA, 'SCENARIO', 'VBAT'; ...
 'DFR0785-W button',                inputs.threeV3,             inputs.ledButtonCurrentA,     'SCENARIO', '3.3 V'; ...
 'SEN0253 IMU',                     inputs.threeV3,             inputs.imuCurrentA,           'PUBLISHED', '3.3 V' ...
};

names = items(:,1);
voltageV = cell2mat(items(:,2));
currentA = cell2mat(items(:,3));
sourceClass = items(:,4);
rail = items(:,5);
powerW = voltageV .* currentA;

assert(all(voltageV>0) && all(currentA>=0));
assert(inputs.eta5>0 && inputs.eta5<=1 && inputs.eta12>0 && inputs.eta12<=1);
assert(inputs.servoUses5VviaHAT, ...
 'This load inventory assumes the DFR0604 servo rail is 5 V. Reconfigure if VP is powered.');

is5 = strcmp(rail,'5 V');
is3 = strcmp(rail,'3.3 V');
is12 = strcmp(rail,'12 V');
isPack = strcmp(rail,'VBAT');

% Treat the 3.3-V load as an additional load on the upstream Pi 5-V supply.
% This assumes lossless Pi-side 3.3-V conversion for a FIRST-ORDER budget.
% Do not double-count if using a measured total Pi input current later.
power5OutputW = sum(powerW(is5)) + sum(powerW(is3));
power12OutputW = sum(powerW(is12));
powerDirectPackW = sum(powerW(isPack));
powerBatteryApproxW = power5OutputW/inputs.eta5 + ...
    power12OutputW/inputs.eta12 + powerDirectPackW;
batteryCurrentApproxA = powerBatteryApproxW / inputs.batteryNominalV;

% Compare separate servo scenario: stall *planning case* is NOT a published
% S0017M stall-current value, NOT measured, and NOT an advised duty cycle.
servoScenarioExtraW = inputs.fiveV * ...
    (inputs.servoHighScenarioA-inputs.servoCurrentA);
peakCaseBatteryApproxW = powerBatteryApproxW + servoScenarioExtraW/inputs.eta5;

%% 3. Publication styling (all text readable on white)
ink = [0.10 0.15 0.23];
muted = [0.32 0.38 0.47];
blue = [0.16 0.39 0.67];           % assumed / planning scenarios
teal = [0.09 0.52 0.45];           % manufacturer/supplier operating points
gridGrey = [0.86 0.89 0.93];

fig = figure('Color','w','Units','pixels','Position',[90 90 1460 840]);
set(fig,'InvertHardcopy','off');
ax = axes('Parent',fig, ...
    'Position',[0.285 0.225 0.648 0.595], ...
    'Color',[1 1 1]);
hold(ax,'on');

n = size(items,1);
barHandles = gobjects(n,1);
for k=1:n
    color = blue;
    if strcmp(sourceClass{k},'PUBLISHED'), color=teal; end
    barHandles(k) = barh(ax,k,powerW(k),0.64, ...
        'FaceColor',color,'EdgeColor','none','LineWidth',0.5);
end
set(ax,'YDir','reverse', ...
    'YTick',1:n,'YTickLabel',names, ...
    'XColor',ink,'YColor',ink,'FontName','Arial','FontSize',11.4, ...
    'LineWidth',1.0,'TickLength',[0 0], ...
    'Box','off','XGrid','on','YGrid','off','GridColor',gridGrey, ...
    'GridAlpha',0.75);
ax.YLim=[0.45 n+0.58];
ax.XLim=[0 15.6];
ax.XTick=0:2:12;
ax.Layer='bottom';
ax.Color = [1 1 1];
xlabel(ax,'Calculated subsystem power (W)', ...
    'Color',ink,'FontSize',12.5,'FontWeight','bold');

% For small loads (< 0.1 W), still display exact numeric value and units.
for k=1:n
    s=sprintf('%.3g W  /  %g V x %.3g A', ...
        powerW(k),voltageV(k),currentA(k));
    text(ax,powerW(k)+0.16,k,s,'Color',muted, ...
        'FontName','Arial','FontSize',10.4, ...
        'VerticalAlignment','middle','Interpreter','none');
end

% Header and interpretation notes; no pale-gray titles.
annotation(fig,'textbox',[0.065 0.91 0.87 0.06], ...
    'String','01  |  Electrical load budget', ...
    'FontName','Arial','FontSize',21,'FontWeight','bold', ...
    'Color',ink,'LineStyle','none','Interpreter','none');
annotation(fig,'textbox',[0.066 0.864 0.87 0.05], ...
    'String','Manufacturer operating points + explicit design assumptions', ...
    'FontName','Arial','FontSize',12.6,'Color',muted, ...
    'LineStyle','none','Interpreter','none');

hPub=plot(ax,nan,nan,'s','MarkerSize',9,'MarkerFaceColor',teal, ...
    'MarkerEdgeColor',teal,'LineStyle','none');
hSc=plot(ax,nan,nan,'s','MarkerSize',9,'MarkerFaceColor',blue, ...
    'MarkerEdgeColor',blue,'LineStyle','none');
lg=legend(ax,[hPub hSc],{ ...
    'Published manufacturer / supplier operating point', ...
    'Explicit design planning assumption'}, ...
    'Location','southoutside','Orientation','horizontal', ...
    'FontName','Arial','FontSize',10.8,'TextColor',ink, ...
    'Box','off');

summaryString = sprintf(['5-V branch (incl. downstream 3.3 V): %.2f W   |   ', ...
    '12-V branch: %.2f W   |   battery-side model: %.2f W / %.2f A at 15.2 V'], ...
    power5OutputW,power12OutputW,powerBatteryApproxW,batteryCurrentApproxA);
annotation(fig,'textbox',[0.065 0.102 0.88 0.05], ...
    'String',summaryString,'FontName','Arial','FontSize',11.0, ...
    'FontWeight','bold','Color',ink,'LineStyle','none', ...
    'Interpreter','none');

%% 4. Save figure and source inputs; print metrics to Command Window
sourceDir=fileparts(mfilename('fullpath'));
if isempty(sourceDir), sourceDir=pwd; end
outDir=fullfile(sourceDir,'results');
if ~exist(outDir,'dir'), mkdir(outDir); end
figurePath=fullfile(outDir,'01_assumed_load_budget.png');
exportgraphics(fig,figurePath,'Resolution',240,'BackgroundColor','white');
save(fullfile(outDir,'simulation_inputs.mat'),'inputs','items', ...
    'powerW','power5OutputW','power12OutputW', ...
    'powerBatteryApproxW','batteryCurrentApproxA');

fprintf('\nYBR-SUNFLOWER | FIGURE 01/11 | ELECTRICAL LOAD BUDGET\n');
fprintf('Evidence: supplier figures + explicit design assumptions; not measured.\n');
fprintf('5 V branch including idealized 3.3 V subload: %.3f W\n',power5OutputW);
fprintf('12 V motor branch at supplier rated-current upper bound: %.3f W\n',power12OutputW);
fprintf('Pack voltmeter branch: %.3f W\n',powerDirectPackW);
fprintf('Battery-side planning estimate (eta5=%.2f, eta12=%.2f): %.3f W\n', ...
    inputs.eta5,inputs.eta12,powerBatteryApproxW);
fprintf('Estimated battery current at nominal %.1f V: %.3f A\n', ...
    inputs.batteryNominalV,batteryCurrentApproxA);
fprintf('If servo momentarily draws %.2f A (scenario only): %.3f W battery-side estimate.\n', ...
    inputs.servoHighScenarioA,peakCaseBatteryApproxW);
fprintf('SAVED: %s\n',figurePath);
fprintf('CAUTION: Pi power recommendation is NOT a current measurement.\n');
fprintf('CAUTION: converter rated input voltage and motor stall risk need separate engineering analysis.\n\n');
