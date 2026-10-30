%% Electrical Figure 08
% 5 V USB-C POWER-PATH SENSITIVITY TO LOAD STEPS AND WIRING RESISTANCE
% WRO 2026 Future Engineers | engineering_journal/2-elec.md
%
% This is a PHYSICS-BASED PLANNING MODEL, NOT MEASURED VOLTAGE DATA.
% Only published ratings below are treated as specifications:
%   DFR1202 nominal output: 5 V +/-1%, maximum 16 A (DFRobot DFR1202).
%   Raspberry Pi low-voltage warning reference: 4.63 V +/-5% (Raspberry Pi).
%   Raspberry Pi reliability guidance also discusses keeping supply >=4.8 V.
% The applied loads, resistances and event times are ALL EXPLICIT ASSUMPTIONS.
%
% The Pi receives DFR1202 power through a USB-C cable. This hypothetical
% path includes the Pi + camera + cooler, HAT electronics and 3.3-V subloads.
% The steering servo shares the Pi-side 5-V rail ONLY if external DFR0604
% VP is unpowered. The RPLIDAR is supplied separately from the 5-V buck;
% its 0.4-A operating load is NOT included in the Pi USB cable current.
%
% Interpretation: voltage at the Pi-side 5-V node is approximated as
% V_pi = V_buck - I_usb * (R_usb_path + R_shared_path).
% This models *resistive* load-step sensitivity, NOT dynamic converter output,
% firmware behavior, actual cable geometry, PD negotiation or servo transients.
%
% Requirements: Base MATLAB R2020a+ for exportgraphics. Run entire script.
% Outputs next to this script:
%   results/08_transient_timeline.png
%   results/simulation_inputs.mat
% No CSV or summary.txt is created.

clearvars; close all; clc;

%% 1. USER INPUTS -- identify every evidence class
p.project = 'YBR-SUNFLOWER | WRO 2026 Future Engineers';
p.figureID = '08 (final electrical MATLAB figure)';
p.modelType = 'Illustrative resistive load-step model; NOT physical measurements';

% Published equipment references, not claimed measurements:
p.converterNominalOutputV = 5.00; % DFR1202: 5 V +/-1%; nominal setpoint
p.converterPublishedToleranceV = 0.05; % 1% of 5 V, not a ripple estimate
p.piLowVoltageWarningV = 4.63;   % Raspberry Pi published low-voltage detection nominal
p.piDesignReferenceV = 4.80;      % Pi reliability guidance / planning reference

% Align the normal/servo values to MATLAB Electrical Figure 01 load budget.
p.piCameraCoolerBaseA = 2.00;     % SCENARIO: combined Pi 5 + camera + cooler
p.hatElectronicsA = 0.120;        % SCENARIO: DFR0592 + DFR0604 logic
p.imuCurrentA = 0.005;           % SEN0253 published operating current (3.3 V)
p.buttonCurrentA = 0.020;        % SCENARIO: DFR0785-W at 3.3 V
p.logic3v3V = 3.3;               % board breakout nominal
p.fiveVBusV = 5.0;
p.servoNoLoadA = 0.150;           % SCENARIO: user's generic 17g servo no-load estimate
p.servoHighLoadA = 1.200;        % SCENARIO: user's generic 17g servo stall-envelope upper
p.piComputeExtraA = 1.00;        % SCENARIO: extra Pi input draw during computation
p.servoViaPi5VRail = true;       % ASSUMPTION: DFR0604 external VP is unpowered

% These are *effective round-trip / common-path resistances*, including
% wires, contacts and supply-path effects. NOT measured cable resistances.
scenarioName = { ...
    'Low-R reference', ...
    'Planning cable + contacts', ...
    'Higher shared resistance', ...
    'Higher USB-C path resistance'};
scenarioUSBohm = [0.020 0.045 0.045 0.100];
scenarioCommonOhm = [0.005 0.010 0.035 0.010];

p.durationS = 6.0;
p.dtS = 0.005;
p.computeEvent1S = [1.60 2.60];
p.computeEvent2S = [3.20 4.20];
p.servoEvent1S = [0.90 1.30];
p.servoEvent2S = [3.50 4.00];

%% 2. Validate assumptions and calculate the electrical power path
assert(p.converterNominalOutputV>0 && p.dtS>0 && p.durationS>0);
assert(p.servoViaPi5VRail, ...
    'This model assumes DFR0604 external VP is unused; edit architecture first.');
assert(p.servoHighLoadA>=p.servoNoLoadA && p.piComputeExtraA>=0);
assert(all(scenarioUSBohm>=0) && all(scenarioCommonOhm>=0));
assert(numel(scenarioName)==numel(scenarioUSBohm));
assert(p.logic3v3V>0 && p.fiveVBusV>p.logic3v3V);

% First-order equivalent of a downstream 3.3 V load, assuming ideal 3.3-V
% conversion (so it is not falsely counted as 3.3 V + 5 V independently).
I3v3EquivalentAt5A = p.logic3v3V * ...
    (p.imuCurrentA+p.buttonCurrentA) / p.fiveVBusV;

% The RPLIDAR has its own DFR1202 output branch, so it is deliberately
% excluded here. This model predicts the USB-C / Pi-side node only.
t = (0:p.dtS:p.durationS)';
computeActive = (t>=p.computeEvent1S(1) & t<p.computeEvent1S(2)) | ...
                (t>=p.computeEvent2S(1) & t<p.computeEvent2S(2));
servoActive = (t>=p.servoEvent1S(1) & t<p.servoEvent1S(2)) | ...
              (t>=p.servoEvent2S(1) & t<p.servoEvent2S(2));

IpiA = p.piCameraCoolerBaseA + p.piComputeExtraA * double(computeActive);
IservoA = p.servoNoLoadA + ...
    (p.servoHighLoadA-p.servoNoLoadA) * double(servoActive);
IusbA = IpiA + p.hatElectronicsA + I3v3EquivalentAt5A + IservoA;

% Quasi-static resistor model; step changes are intentional. DO NOT read
% the sharp steps as actual converter bandwidth or physical rise time.
nScenario = numel(scenarioName);
Vpi = zeros(numel(t),nScenario);
for k=1:nScenario
    Vpi(:,k) = p.converterNominalOutputV - ...
        IusbA * (scenarioUSBohm(k)+scenarioCommonOhm(k));
end

minimumV = min(Vpi,[],1);
referenceCurrentA = p.piCameraCoolerBaseA+p.hatElectronicsA+ ...
    I3v3EquivalentAt5A+p.servoNoLoadA;
peakCurrentA = max(IusbA);

%% 3. Figure style -- MATCH Figures 01-07: white axes, dark typography
ink      = [0.10 0.15 0.23];
muted    = [0.33 0.39 0.47];
gridGrey = [0.85 0.88 0.92];
blue     = [0.17 0.40 0.70];
teal     = [0.10 0.51 0.46];
purple   = [0.52 0.34 0.66];
amber    = [0.77 0.40 0.14];
warn     = [0.72 0.27 0.24];
colors = [blue; teal; purple; amber];

fig=figure('Color',[1 1 1],'Units','pixels', ...
    'Position',[70 70 1480 940]);
set(fig,'InvertHardcopy','off');

annotation(fig,'textbox',[0.066 0.929 0.89 0.053], ...
    'String','08  |  5 V wiring and load-step sensitivity', ...
    'FontName','Arial','FontSize',21,'FontWeight','bold', ...
    'Color',ink,'LineStyle','none','Interpreter','none');
annotation(fig,'textbox',[0.067 0.884 0.88 0.039], ...
    'String','DFR1202 nominal 5 V output | hypothetical USB-C and shared supply-path resistance', ...
    'FontName','Arial','FontSize',12.4,'Color',muted, ...
    'LineStyle','none','Interpreter','none');

%% 4. Upper panel: explicit current-demand timeline
ax1=axes('Parent',fig,'Position',[0.110 0.627 0.800 0.199], ...
    'Color',[1 1 1]);
hold(ax1,'on');
plot(ax1,t,IpiA,'-','Color',blue,'LineWidth',2.5);
plot(ax1,t,IservoA,'--','Color',amber,'LineWidth',2.1);
plot(ax1,t,IusbA,':','Color',teal,'LineWidth',2.5);
set(ax1,'XLim',[0 p.durationS],'YLim',[0 4.9], ...
    'XColor',ink,'YColor',ink,'FontName','Arial','FontSize',11.0, ...
    'LineWidth',0.95,'TickLength',[0 0], ...
    'Box','off','XGrid','on','YGrid','on','GridColor',gridGrey, ...
    'GridAlpha',0.70,'Layer','bottom');
ax1.Color=[1 1 1];
ylabel(ax1,'Modeled current (A)', ...
    'Color',ink,'FontName','Arial','FontSize',11.6,'FontWeight','bold');
lg1=legend(ax1,{'Pi-side compute','Servo','USB-C path total'}, ...
    'Location','northwest','Orientation','horizontal', ...
    'FontName','Arial','FontSize',10.0, ...
    'TextColor',ink,'Box','off');
lg1.Color=[1 1 1];

%% 5. Lower panel: same input schedule, different hypothesized resistances
ax2=axes('Parent',fig,'Position',[0.110 0.300 0.800 0.258], ...
    'Color',[1 1 1]);
hold(ax2,'on');
plots=gobjects(1,nScenario);
for k=1:nScenario
    plots(k)=plot(ax2,t,Vpi(:,k),'-', ...
        'Color',colors(k,:),'LineWidth',2.25);
end
plot(ax2,[0 p.durationS],[p.piDesignReferenceV p.piDesignReferenceV], ...
    '--','Color',amber,'LineWidth',1.45,'HandleVisibility','off');
plot(ax2,[0 p.durationS],[p.piLowVoltageWarningV p.piLowVoltageWarningV], ...
    ':','Color',warn,'LineWidth',1.65,'HandleVisibility','off');

% Reference lines labeled at right, outside the principal event windows.
text(ax2,p.durationS-0.07,p.piDesignReferenceV+0.013, ...
    '4.80 V design reference','Color',amber, ...
    'FontName','Arial','FontSize',9.6,'FontWeight','bold', ...
    'HorizontalAlignment','right','VerticalAlignment','bottom');
text(ax2,p.durationS-0.07,p.piLowVoltageWarningV-0.018, ...
    '4.63 V low-voltage warning reference','Color',warn, ...
    'FontName','Arial','FontSize',9.6, ...
    'HorizontalAlignment','right','VerticalAlignment','top');

set(ax2,'XLim',[0 p.durationS],'YLim',[4.39 5.015], ...
    'XColor',ink,'YColor',ink,'FontName','Arial','FontSize',11.0, ...
    'LineWidth',0.95,'TickLength',[0 0], ...
    'Box','off','XGrid','on','YGrid','on','GridColor',gridGrey, ...
    'GridAlpha',0.70,'Layer','bottom');
ax2.Color=[1 1 1];
ax2.YTick=[4.4 4.5 4.6 4.7 4.8 4.9 5.0];
xlabel(ax2,'Illustrative elapsed time (seconds)', ...
    'Color',ink,'FontName','Arial','FontSize',12.0,'FontWeight','bold');
ylabel(ax2,'Modeled Pi supply (V)', ...
    'Color',ink,'FontName','Arial','FontSize',11.6,'FontWeight','bold');

lg2=legend(ax2,plots,scenarioName,'Location','southwest', ...
    'FontName','Arial','FontSize',9.9,'TextColor',ink,'Box','on');
lg2.Color=[1 1 1];
lg2.EdgeColor=gridGrey;

%% 6. Footer: scenario summary and model limitations
annotation(fig,'textbox',[0.066 0.174 0.88 0.051], ...
    'String',sprintf(['Normal Pi USB path: %.3f A  |  peak modeled USB path: %.3f A  |  ', ...
    'nominal DFR1202 output: %.2f V'], ...
    referenceCurrentA,peakCurrentA,p.converterNominalOutputV), ...
    'FontName','Arial','FontSize',10.9,'FontWeight','bold', ...
    'Color',ink,'LineStyle','none','Interpreter','none');

%% 7. Export exactly one PNG and the reproducible input file
scriptDir=fileparts(mfilename('fullpath'));
if isempty(scriptDir),scriptDir=pwd;end
resultDir=fullfile(scriptDir,'results');
if ~exist(resultDir,'dir'),mkdir(resultDir);end
pngPath=fullfile(resultDir,'08_transient_timeline.png');
exportgraphics(fig,pngPath,'Resolution',240,'BackgroundColor','white');

% Save only data that explain/reproduce this sensitivity study.
save(fullfile(resultDir,'simulation_inputs.mat'), ...
    'p','scenarioName','scenarioUSBohm','scenarioCommonOhm', ...
    'I3v3EquivalentAt5A','t','IpiA','IservoA','IusbA','Vpi','minimumV');

fprintf('\nYBR-SUNFLOWER | ELECTRICAL FIGURE 08 | FINAL MATLAB VISUAL\n');
fprintf('Evidence: spec voltages + scenario currents/resistances; NOT measurements.\n');
fprintf('DFR1202 nominal output: %.2f V; tolerance +/-%.2f V\n', ...
    p.converterNominalOutputV,p.converterPublishedToleranceV);
fprintf('Normal Pi USB path current: %.4f A\n',referenceCurrentA);
fprintf('Peak Pi USB path current: %.4f A\n',peakCurrentA);
for k=1:nScenario
    fprintf('%-31s | R_usb=%.3f ohm  R_common=%.3f ohm  Min V_pi=%.3f V\n', ...
        scenarioName{k},scenarioUSBohm(k),scenarioCommonOhm(k),minimumV(k));
end
fprintf('SAVED: %s\n',pngPath);
fprintf(['CAUTION: The stated voltages are equivalent-circuit results only; ', ...
    'do not infer any measured USB-C or DFR1202 response.\n\n']);
