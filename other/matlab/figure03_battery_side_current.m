%% Electrical Figure 03
% ESTIMATED BATTERY-SIDE CURRENT -- fixed-load planning model
% WRO 2026 Future Engineers | engineering_journal/2-elec.md
%
% This is a CALCULATION, not a measured current or battery-discharge trace.
% Inputs match Figure 01/11 (Electrical Load Budget).
% Manufacturer/supplier values are separated from assumptions below.
%
% Requires base MATLAB R2020a+ (exportgraphics).
% Run the entire script. Output: results/03_estimated_input_current.png
% and results/simulation_inputs.mat. No CSV or summary.txt.

clearvars; close all; clc;

%% 1. USER INPUTS -- same inventory and conditions as Figure 01
p.packNominalV = 15.2;          % GNB 4S LiHV: nominal [V], manufacturer label
p.packFullReferenceV = 17.4;    % 4 x 4.35 V LiHV reference; NOT measured
p.buck5MinInputV = 6.0;        % DFR1202: manufacturer VIN minimum [V]
p.buck12MinInputV = 15.0;      % DFR1208: manufacturer VIN minimum [V]
p.buckInputMaxV = 24.0;        % Both regulators: manufacturer VIN maximum [V]

p.V5 = 5.0;                    % DFR1202 nominal output [V]
p.V12 = 12.0;                  % DFR1208 nominal output [V]
p.V3v3 = 3.3;                  % DFR0604 logic breakout supply [V]
p.eta5 = 0.90;                % ASSUMED effective 5 V conversion efficiency
p.eta12 = 0.90;               % ASSUMED effective 12 V conversion efficiency
% Efficiencies above are fixed modeling values, NOT verified at 15.2 V.

% 5 V branch: planning currents except where a source is identified.
p.piCameraCoolerA = 2.00;      % ASSUMED combined Pi 5 + OV5647 + cooler
p.lidarWorkingA = 0.400;      % DFRobot S2L typical working current
p.hatLogicA = 0.120;          % ASSUMED DFR0592 + DFR0604 logic
p.servoNormalA = 0.150;       % ASSUMED representative no-load servo
p.servoHighA = 1.200;         % User-provided generic high-current scenario;
                                % NOT verified S0017M model-specific rating

% Downstream 3.3 V loads: modeled as ideal power transferred from 5 V.
p.buttonA = 0.020;            % ASSUMED LED button at 3.3 V
p.imuA = 0.005;               % DFRobot SEN0253 specification

% 12 V branch: supplier-rated upper-bound running current for motor.
p.motorRatedBoundA = 0.550;   % CHP-20GP-180 rated <=550 mA at 12 V

% Voltmeter connects directly to switched battery (scenario current).
p.voltmeterA = 0.015;         % ASSUMED battery voltmeter at VBAT_SW

% Visualization limits only: 13 V is NOT a battery cutoff recommendation.
p.viewMinV = 13.0;
p.viewMaxV = p.packFullReferenceV;
p.samples = 500;

%% 2. Validation and component-level power calculation
assert(p.packNominalV > 0 && p.packFullReferenceV >= p.packNominalV);
assert(p.eta5 > 0 && p.eta5 <= 1 && p.eta12 > 0 && p.eta12 <= 1);
assert(p.servoHighA >= p.servoNormalA && p.servoNormalA >= 0);
assert(p.buck12MinInputV > p.buck5MinInputV);
assert(p.viewMinV < p.buck12MinInputV && p.viewMaxV > p.packNominalV);
assert(p.samples >= 100);

% Exactly the Figure 01 inventory. The 3.3 V loads count upstream of 5 V,
% with ideal internal 5 V -> 3.3 V conversion for this first-order model.
P5normalW = p.V5 * (p.piCameraCoolerA + p.lidarWorkingA + ...
    p.hatLogicA + p.servoNormalA) + ...
    p.V3v3 * (p.buttonA + p.imuA);
P5highW = P5normalW + p.V5 * (p.servoHighA - p.servoNormalA);
P12W = p.V12 * p.motorRatedBoundA;
PdirectW = p.packNominalV * p.voltmeterA;

% Fixed regulated-output loads and constant converter-efficiency model:
% I_battery(V) = (P_5V/eta_5V + P_12V/eta_12V)/V + I_voltmeter.
% A direct-wired voltmeter is modeled as constant current, not fixed power.
PconverterNormalW = P5normalW/p.eta5 + P12W/p.eta12;
PconverterHighW = P5highW/p.eta5 + P12W/p.eta12;
% Nominal input powers below match Figure 01 at 15.2 V.
PbatNormalW = PconverterNormalW + PdirectW;
PbatHighW   = PconverterHighW + PdirectW;

vin = linspace(p.viewMinV,p.viewMaxV,p.samples);
valid = vin >= max(p.buck5MinInputV,p.buck12MinInputV) & ...
        vin <= p.buckInputMaxV;
IbatNormalA = nan(size(vin));
IbatHighA = nan(size(vin));
IbatNormalA(valid) = PconverterNormalW ./ vin(valid) + p.voltmeterA;
IbatHighA(valid) = PconverterHighW ./ vin(valid) + p.voltmeterA;

InomNormalA = PconverterNormalW/p.packNominalV + p.voltmeterA;
InomHighA = PconverterHighW/p.packNominalV + p.voltmeterA;
IfullNormalA = PconverterNormalW/p.packFullReferenceV + p.voltmeterA;
IfullHighA = PconverterHighW/p.packFullReferenceV + p.voltmeterA;

%% 3. Publication palette -- ALL backgrounds white, ALL text dark
ink      = [0.10 0.15 0.23];
muted    = [0.33 0.39 0.47];
gridGrey = [0.85 0.88 0.92];
blue     = [0.17 0.40 0.70];   % Normal planning scenario
teal     = [0.10 0.51 0.46];   % High servo-current scenario
amber    = [0.77 0.40 0.14];   % Nominal battery voltage
warn     = [0.72 0.27 0.24];   % Outside 12 V converter VIN rating

fig = figure('Color','white','Units','pixels', ...
    'Position',[65 65 1480 860]);
set(fig,'InvertHardcopy','off');

annotation(fig,'textbox',[0.066 0.920 0.87 0.060], ...
    'String','03  |  Estimated battery-side current', ...
    'FontName','Arial','FontSize',21,'FontWeight','bold', ...
    'Color',ink,'LineStyle','none','Interpreter','none');
annotation(fig,'textbox',[0.067 0.868 0.86 0.050], ...
    'String','Fixed-power estimate based on the Figure 01 load inventory', ...
    'FontName','Arial','FontSize',12.5,'Color',muted, ...
    'LineStyle','none','Interpreter','none');

%% 4. Main panel -- avoid extrapolating outside converter specifications
ax = axes('Parent',fig, ...
    'Position',[0.135 0.300 0.780 0.510], ...
    'Color',[1 1 1]);
hold(ax,'on');

% The left-hand shaded region is OUTSIDE the DFR1208 published VIN range.
% Do not plot the combined 5 V + 12 V system-current model there.
patch(ax,[p.viewMinV p.buck12MinInputV ...
          p.buck12MinInputV p.viewMinV], ...
    [1.12 1.12 2.29 2.29],[1.00 0.93 0.93], ...
    'EdgeColor','none','FaceAlpha',0.88);

hNormal = plot(ax,vin,IbatNormalA,'-', ...
    'Color',blue,'LineWidth',2.8);
hHigh = plot(ax,vin,IbatHighA,'-', ...
    'Color',teal,'LineWidth',2.8);

% Threshold and battery-reference markers.
plot(ax,[p.buck12MinInputV p.buck12MinInputV],[1.12 2.29], ...
    '--','Color',warn,'LineWidth',1.65);
plot(ax,[p.packNominalV p.packNominalV],[1.12 2.29], ...
    ':','Color',amber,'LineWidth',1.8);
plot(ax,[p.packFullReferenceV p.packFullReferenceV],[1.12 2.29], ...
    ':','Color',muted,'LineWidth',1.1);

plot(ax,p.packNominalV,InomNormalA,'o', ...
    'MarkerSize',10,'MarkerFaceColor',blue, ...
    'MarkerEdgeColor','white','LineWidth',1.8);
plot(ax,p.packNominalV,InomHighA,'o', ...
    'MarkerSize',10,'MarkerFaceColor',teal, ...
    'MarkerEdgeColor','white','LineWidth',1.8);

% Non-overlapping, high-contrast point labels.
text(ax,15.38,InomHighA+0.095, ...
    sprintf('High-current scenario: %.2f A',InomHighA), ...
    'Color',ink,'FontName','Arial','FontSize',11.1,'FontWeight','bold');
text(ax,15.38,InomNormalA+0.105, ...
    sprintf('Normal planning case: %.2f A',InomNormalA), ...
    'Color',ink,'FontName','Arial','FontSize',11.1,'FontWeight','bold');
text(ax,13.21,2.055, ...
    sprintf('Below DFR1208\n15 V VIN minimum'), ...
    'Color',warn,'FontName','Arial','FontSize',11, ...
    'HorizontalAlignment','left');
text(ax,15.03,2.23,'15 V', ...
    'Color',warn,'FontName','Arial','FontSize',10.2, ...
    'HorizontalAlignment','left');
text(ax,p.packNominalV+0.04,2.23,'15.2 V nominal', ...
    'Color',amber,'FontName','Arial','FontSize',10.2, ...
    'HorizontalAlignment','left');

set(ax,'XLim',[13 17.70],'YLim',[1.12 2.29], ...
    'XTick',[13 14 15 15.2 16 17 17.4], ...
    'XTickLabel',{'13','14','15','15.2','16','17','17.4'}, ...
    'YTick',1.2:0.2:2.2,'FontName','Arial','FontSize',11.4, ...
    'XColor',ink,'YColor',ink,'LineWidth',1.0, ...
    'TickLength',[0 0],'Box','off', ...
    'XGrid','on','YGrid','on', ...
    'GridColor',gridGrey,'GridAlpha',0.73, ...
    'Layer','bottom');
ax.Color = [1 1 1];

xlabel(ax,'Illustrative battery terminal voltage (V)', ...
    'Color',ink,'FontName','Arial','FontSize',12.2,'FontWeight','bold');
ylabel(ax,'Estimated battery-side current (A)', ...
    'Color',ink,'FontName','Arial','FontSize',12.2,'FontWeight','bold');

lg=legend(ax,[hNormal hHigh],{ ...
    'Normal planning loads (servo 0.15 A)', ...
    'High servo-current case (servo 1.20 A)'}, ...
    'Location','northeast','Orientation','vertical', ...
    'FontName','Arial','FontSize',10.8, ...
    'TextColor',ink,'Color',[1 1 1], ...
    'EdgeColor',gridGrey,'Box','on');

%% 5. Footer -- leave space below X-axis title (no overlap)
metrics=sprintf([ ...
    '5-V load %.2f W  |  12-V load %.2f W  |  ', ...
    'nominal battery: %.2f A (normal) / %.2f A (high servo)'], ...
    P5normalW,P12W,InomNormalA,InomHighA);
annotation(fig,'textbox',[0.066 0.164 0.89 0.052], ...
    'String',metrics, ...
    'FontName','Arial','FontSize',11.0,'FontWeight','bold', ...
    'Color',ink,'LineStyle','none','Interpreter','none');

%% 6. Save result, saved model inputs, and Command Window summary
scriptDir=fileparts(mfilename('fullpath'));
if isempty(scriptDir),scriptDir=pwd;end
outDir=fullfile(scriptDir,'results');
if ~exist(outDir,'dir'),mkdir(outDir);end
figurePath=fullfile(outDir,'03_estimated_input_current.png');
exportgraphics(fig,figurePath,'Resolution',240, ...
    'BackgroundColor','white');
save(fullfile(outDir,'simulation_inputs.mat'), ...
    'p','vin','valid','P5normalW','P5highW','P12W', ...
    'PdirectW','PconverterNormalW','PconverterHighW', ...
    'PbatNormalW','PbatHighW', ...
    'IbatNormalA','IbatHighA','InomNormalA','InomHighA', ...
    'IfullNormalA','IfullHighA');

fprintf('\nYBR-SUNFLOWER | ELECTRICAL FIGURE 03/11\n');
fprintf('Model: Ibatt(V) = (P5/eta5 + P12/eta12)/V + Ivoltmeter.\n');
fprintf('Published/regulator scenario: DFR1208 VIN >= %.1f V.\n', ...
    p.buck12MinInputV);
fprintf('Normal scenario: P5 %.3f W; P12 %.3f W; direct %.3f W.\n', ...
    P5normalW,P12W,PdirectW);
fprintf('Battery-side power: normal %.3f W; high servo %.3f W.\n', ...
    PbatNormalW,PbatHighW);
fprintf('At %.1f V nominal: normal %.3f A; high servo %.3f A.\n', ...
    p.packNominalV,InomNormalA,InomHighA);
fprintf('At %.1f V LiHV reference: normal %.3f A; high servo %.3f A.\n', ...
    p.packFullReferenceV,IfullNormalA,IfullHighA);
fprintf('Saved: %s\n',figurePath);
fprintf('NOT MEASURED: converter efficiencies, Pi load, servo current, cable losses.\n');
fprintf('Do not use this graph as evidence of runtime or regulation quality.\n\n');
