%% Electrical Figure 05
% SIMULATED BATTERY TERMINAL VOLTAGE VS ELAPSED TIME
% WRO 2026 Future Engineers | engineering_journal/2-elec.md
%
% Evidence classes:
%  SUPPLIER: GNB LiHV 4S, 15.2 V nominal, 1.30 Ah;
%            DFR1208 published 15 V minimum converter INPUT.
%  CHEMISTRY REFERENCE: 17.4 V = 4 x 4.35 V/cell (not measured).
%  MODEL: SOC/OCV knots, R_pack, load currents, efficiency and load scales.
%
% Uses EXACTLY Figure 04's illustrative OCV knots and PCHIP interpolation,
% and Figure 01 / Figure 03's normal planning electrical load inventory.
% Neither curves nor crossing times are robot measurements or certified
% converter operation, battery runtime, or a safe-discharge recommendation.
%
% Base MATLAB R2020a+; no additional toolboxes.
% Output: results/05_voltage_vs_time_scenarios.png (240 dpi)
%         results/simulation_inputs.mat
% No CSV, no summary.txt. Run the entire script.

clearvars; close all; clc;

%% 1. Source-backed ratings and explicit model parameters
p.project = 'YBR-SUNFLOWER | WRO 2026 Future Engineers';
p.figureID = '05/11';
p.evidenceClass = 'Calculated sensitivity study, NOT measured discharge';
p.cells = 4;
p.capacityAh = 1.30;                 % GNB nameplate (Ah)
p.nominalPackV = 15.2;               % GNB nameplate (V)
p.lihvFullCellRefV = 4.35;            % LiHV chemistry reference, NOT measured
p.initialSOC = 1.00;                 % hypothetical 100% initial SOC
p.OCV_SOC_knots = 0:0.10:1.00;      % illustrative values; Figure 04
p.OCV_cell_V = [3.30 3.55 3.66 3.72 3.76 3.80 ...
                3.84 3.90 4.00 4.15 4.35];
p.interpolationMethod = 'pchip';      % same method as Figure 04
p.packSeriesResistanceOhm = 0.12;    % assumed lumped pack+contact resistance
p.converter12MinInputV = 15.0;       % DFR1208 published minimum VIN
p.converter5MinInputV = 6.0;        % DFR1202 published minimum VIN
p.eta5 = 0.90;                      % constant effective efficiency, ASSUMED
p.eta12 = 0.90;                     % constant effective efficiency, ASSUMED

% Figure 01 / 03 loads: identical values, so results stay compatible.
p.V5 = 5.0;
p.V12 = 12.0;
p.V3v3 = 3.3;
p.piCameraCoolerA = 2.00;          % modeled Pi+camera+cooler @ 5 V
p.lidarWorkingA = 0.400;           % DFRobot S2L working-current point
p.hatLogicA = 0.120;               % modeled HAT electronics @ 5 V
p.servoNormalA = 0.150;            % modeled no-load servo @ 5 V
p.ledButtonA = 0.020;              % modeled LED button @ 3.3 V
p.imuA = 0.005;                    % DFRobot SEN0253 @ 3.3 V
p.motorRatedBoundA = 0.550;        % CHP supplier rated-current upper bound @ 12 V
p.voltmeterA = 0.015;              % assumed constant voltmeter draw from pack

% Load multipliers apply to REGULATED-OUTPUT demand, not the voltmeter.
p.scenarioNames = {'Light demand  |  0.5x', ...
                   'Planning demand  |  1.0x', ...
                   'Heavy demand  |  1.5x'};
p.scenarioScale = [0.5 1.0 1.5];
p.stepSeconds = 1.0;
p.maxSimMinutes = 120;             % model time horizon, not expected runtime
p.demoStopTerminalV = 13.2;        % ILLUSTRATIVE endpoint, NOT safe cutoff

%% 2. Check assumptions and reconcile Figure 01 power budget
assert(p.cells == 4 && p.capacityAh > 0 && p.initialSOC > 0);
assert(p.initialSOC <= 1 && p.stepSeconds > 0 && p.maxSimMinutes > 0);
assert(numel(p.OCV_SOC_knots) == numel(p.OCV_cell_V));
assert(p.OCV_SOC_knots(1)==0 && p.OCV_SOC_knots(end)==1);
assert(all(diff(p.OCV_SOC_knots)>0) && all(diff(p.OCV_cell_V)>0));
assert(abs(p.OCV_cell_V(end)-p.lihvFullCellRefV) < 1e-9);
assert(p.packSeriesResistanceOhm >= 0);
assert(all(p.scenarioScale>0) && p.eta5>0 && p.eta5<=1);
assert(p.eta12>0 && p.eta12<=1);
assert(p.demoStopTerminalV < p.converter12MinInputV);

% Include 3.3 V loads as upstream 5 V equivalent power with ideal internal
% conversion, as was explicitly assumed in Figure 01. Do not double-count
% these items if replacing the Pi 5 V estimate with measured input current.
P5_W = p.V5 * (p.piCameraCoolerA + p.lidarWorkingA + ...
    p.hatLogicA + p.servoNormalA) + p.V3v3*(p.ledButtonA+p.imuA);
P12_W = p.V12 * p.motorRatedBoundA;
PconverterInput_W = P5_W/p.eta5 + P12_W/p.eta12;
PdirectNom_W = p.nominalPackV * p.voltmeterA;
PtotalNom_W = PconverterInput_W + PdirectNom_W;
IbatNom_A = PconverterInput_W/p.nominalPackV + p.voltmeterA;
assert(abs(P5_W-13.4325)<1e-9, 'Figure 01 5 V input inventory has changed.');
assert(abs(P12_W-6.6000)<1e-9, 'Figure 01 motor inventory has changed.');

%% 3. Simulate each scenario (terminal voltage includes I*R droop)
N = numel(p.scenarioScale);
runs = cell(1,N);
for k=1:N
    Pload_W = p.scenarioScale(k) * PconverterInput_W;
    runs{k} = simulateBattery(p,Pload_W);
end

%% 4. Consistent publication styling -- WHITE figure and WHITE axes
ink      = [0.10 0.15 0.23];
muted    = [0.33 0.39 0.47];
gridGrey = [0.85 0.88 0.92];
blue     = [0.17 0.40 0.70];
teal     = [0.10 0.51 0.46];
purple   = [0.52 0.34 0.66];
amber    = [0.77 0.40 0.14];
warn     = [0.72 0.27 0.24];
colors   = [blue; teal; purple];

fig = figure('Color','white','Units','pixels','Position',[65 65 1480 860]);
set(fig,'InvertHardcopy','off');
annotation(fig,'textbox',[0.066 0.923 0.88 0.060], ...
    'String','05  |  Simulated battery terminal voltage vs. time', ...
    'FontName','Arial','FontSize',21,'FontWeight','bold', ...
    'Color',ink,'LineStyle','none','Interpreter','none');
annotation(fig,'textbox',[0.067 0.871 0.88 0.042], ...
    'String','GNB 4S LiHV model  |  three constant-demand sensitivity scenarios', ...
    'FontName','Arial','FontSize',12.5,'Color',muted, ...
    'LineStyle','none','Interpreter','none');

ax = axes('Parent',fig, ...
    'Position',[0.125 0.295 0.785 0.520], ...
    'Color',[1 1 1]);
hold(ax,'on');

% Shaded voltage region is OUTSIDE DFR1208's stated minimum INPUT. Beyond
% this threshold the constant-regulated-power scenario is an EXTRAPOLATION.
tMax = max(cellfun(@(r) max(r.timeMinutes),runs));
tEnd = max(5,ceil(tMax/5)*5);
patch(ax,[0 tEnd tEnd 0], ...
    [p.demoStopTerminalV p.demoStopTerminalV ...
     p.converter12MinInputV p.converter12MinInputV], ...
    [1.0 0.935 0.935], ...
    'FaceAlpha',0.75,'EdgeColor','none');

% Three curves: solid within DFR1208 VIN specifications; dotted after
% crossing below 15 V to distinguish unvalidated load extrapolation.
legendHandles = gobjects(1,N);
for k=1:N
    r=runs{k};
    valid = r.terminalV >= p.converter12MinInputV;
    aboveV = r.terminalV;
    aboveV(~valid) = nan;
    belowV = r.terminalV;
    belowV(valid) = nan;
    legendHandles(k) = plot(ax,r.timeMinutes,aboveV, ...
        '-','Color',colors(k,:),'LineWidth',2.8);
    plot(ax,r.timeMinutes,belowV,':', ...
        'Color',colors(k,:),'LineWidth',2.1,'HandleVisibility','off');
    % Re-plot one crossing segment (dotted) so curves don't visually break.
    crossingIdx = find(diff(valid)<0,1,'first');
    if ~isempty(crossingIdx)
        plot(ax,r.timeMinutes(crossingIdx:crossingIdx+1), ...
            r.terminalV(crossingIdx:crossingIdx+1),':', ...
            'Color',colors(k,:),'LineWidth',2.1, ...
            'HandleVisibility','off');
        plot(ax,r.firstBelow15Min,p.converter12MinInputV,'o', ...
            'MarkerSize',7.5,'MarkerFaceColor','white', ...
            'MarkerEdgeColor',colors(k,:),'LineWidth',1.8, ...
            'HandleVisibility','off');
    end
end

% Mark DFR1208 threshold separately from nominal battery voltage.
plot(ax,[0 tEnd],[p.converter12MinInputV p.converter12MinInputV], ...
    '--','Color',warn,'LineWidth',1.6,'HandleVisibility','off');
plot(ax,[0 tEnd],[p.nominalPackV p.nominalPackV], ...
    ':','Color',amber,'LineWidth',1.0,'HandleVisibility','off');

text(ax,tEnd*0.98,p.converter12MinInputV+0.13, ...
    '15.0 V  |  DFR1208 minimum VIN', ...
    'Color',warn,'FontName','Arial','FontSize',10.7, ...
    'FontWeight','bold','HorizontalAlignment','right');
text(ax,tEnd*0.98,14.05, ...
    'Below 15 V: power-demand model extrapolation only', ...
    'Color',warn,'FontName','Arial','FontSize',10.2, ...
    'HorizontalAlignment','right');

set(ax,'XLim',[0 tEnd],'YLim',[13.05 17.65], ...
    'XColor',ink,'YColor',ink,'FontName','Arial', ...
    'FontSize',11.5,'LineWidth',1.0,'TickLength',[0 0], ...
    'Box','off','XGrid','on','YGrid','on', ...
    'GridColor',gridGrey,'GridAlpha',0.70, ...
    'Layer','bottom');
ax.Color = [1 1 1];
xlabel(ax,'Simulated elapsed time (minutes)', ...
    'Color',ink,'FontName','Arial','FontSize',12.3,'FontWeight','bold');
ylabel(ax,'Modeled battery terminal voltage (V)', ...
    'Color',ink,'FontName','Arial','FontSize',12.3,'FontWeight','bold');

lg=legend(ax,legendHandles,p.scenarioNames, ...
    'Location','northeast','FontName','Arial','FontSize',10.6, ...
    'TextColor',ink,'Color',[1 1 1], ...
    'EdgeColor',gridGrey,'Box','on');

% Footer is below the axis and cannot overlap the X-axis title.
nominal = runs{2};
if isfinite(nominal.firstBelow15Min)
    crossingLabel=sprintf('planning-case 15 V crossing: %.1f min (model only)', ...
        nominal.firstBelow15Min);
else
    crossingLabel='planning-case 15 V crossing: not observed in modeled interval';
end
summaryString=sprintf(['Planning load %.2f W converter input (nominal pack)  |  ', ...
    'Rpack %.2f ohm assumed  |  %s'], ...
    PconverterInput_W,p.packSeriesResistanceOhm,crossingLabel);
annotation(fig,'textbox',[0.066 0.156 0.88 0.065], ...
    'String',summaryString,'FontName','Arial','FontSize',10.9, ...
    'FontWeight','bold','Color',ink, ...
    'LineStyle','none','Interpreter','none');

%% 5. Save one figure and inputs; no CSV and no summary text
scriptDir=fileparts(mfilename('fullpath'));
if isempty(scriptDir), scriptDir=pwd; end
outDir=fullfile(scriptDir,'results');
if ~exist(outDir,'dir'),mkdir(outDir);end
figurePath=fullfile(outDir,'05_voltage_vs_time_scenarios.png');
exportgraphics(fig,figurePath,'Resolution',240,'BackgroundColor','white');
save(fullfile(outDir,'simulation_inputs.mat'), ...
    'p','runs','P5_W','P12_W','PconverterInput_W','PdirectNom_W', ...
    'PtotalNom_W','IbatNom_A');

fprintf('\nYBR-SUNFLOWER | FIGURE 05/11 | BATTERY VOLTAGE VS TIME\n');
fprintf('GNB rating: %.1f V nominal; %.2f Ah; 4S LiHV\n', ...
    p.nominalPackV,p.capacityAh);
fprintf('Figure 01 load inventory: P5=%.4f W; P12=%.4f W\n',P5_W,P12_W);
fprintf('Model converter input demand: %.4f W at 1.0x (eta5=%.2f; eta12=%.2f)\n', ...
    PconverterInput_W,p.eta5,p.eta12);
fprintf('Voltmeter added current: %.3f A; nominal-pack total: %.4f W / %.4f A\n', ...
    p.voltmeterA,PtotalNom_W,IbatNom_A);
for k=1:N
    r=runs{k};
    if isnan(r.firstBelow15Min),crossingText='not reached';
    else,crossingText=sprintf('%.2f min',r.firstBelow15Min);end
    fprintf('%s | Pload %.2f W | first terminal voltage <15V: %s | stop: %s\n', ...
        p.scenarioNames{k},r.requestedPowerW,crossingText,r.stopReason);
end
fprintf('Figure saved: %s\n',figurePath);
fprintf('MODEL ONLY: crossing times and voltage paths are NOT observed robot runtime.\n\n');

%% Local simulation function -- supplied IN FULL; no external dependencies
function r=simulateBattery(p,Pload_W)
    % Circuit model: V=OCV-I*R; I=Pload/V + Ivoltmeter.
    % Rearranged: V^2-(OCV-R*Ivoltmeter)*V+R*Pload=0.
    % Physical/high-voltage root gives the normal equivalent-circuit point.
    maxSteps=ceil(p.maxSimMinutes*60/p.stepSeconds)+1;
    t=nan(maxSteps,1); soc=nan(maxSteps,1);
    ocv=nan(maxSteps,1); V=nan(maxSteps,1); I=nan(maxSteps,1);
    t(1)=0; soc(1)=p.initialSOC;
    reason='Maximum visualization time reached';
    for j=1:maxSteps
        ocv(j)=p.cells*interp1(p.OCV_SOC_knots,p.OCV_cell_V, ...
            soc(j),p.interpolationMethod);
        adjustedOCV=ocv(j)-p.packSeriesResistanceOhm*p.voltmeterA;
        discriminant=adjustedOCV^2-4*p.packSeriesResistanceOhm*Pload_W;
        if discriminant < 0
            reason='Equivalent-circuit load infeasible';break;
        end
        V(j)=(adjustedOCV+sqrt(max(0,discriminant)))/2;
        I(j)=Pload_W/V(j)+p.voltmeterA;
        if V(j)<=p.demoStopTerminalV
            reason='Demonstration endpoint reached (not cutoff advice)';break;
        end
        if soc(j)<=0
            reason='Hypothetical SOC exhausted';break;
        end
        if j==maxSteps,break;end
        nextSOC=max(0,soc(j)-I(j)*p.stepSeconds/(3600*p.capacityAh));
        soc(j+1)=nextSOC;
        t(j+1)=t(j)+p.stepSeconds;
    end
    ok=isfinite(V);
    t=t(ok);soc=soc(ok);ocv=ocv(ok);V=V(ok);I=I(ok);
    below=find(V<p.converter12MinInputV,1,'first');
    if isempty(below),crossMin=nan;
    else
        if below==1
            crossMin=0;
        else
            t1=t(below-1);t2=t(below);
            v1=V(below-1);v2=V(below);
            if v1==v2,crossMin=t2/60;
            else
                crossMin=(t1+(p.converter12MinInputV-v1)* ...
                    (t2-t1)/(v2-v1))/60;
            end
        end
    end
    r=struct('timeMinutes',t/60,'SOC',soc,'OCV_V',ocv, ...
        'terminalV',V,'batteryCurrentA',I,'requestedPowerW',Pload_W, ...
        'firstBelow15Min',crossMin,'stopReason',reason);
end
