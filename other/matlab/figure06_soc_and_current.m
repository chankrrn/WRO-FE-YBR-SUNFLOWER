%% Electrical Figure 06
% SIMULATED BATTERY STATE OF CHARGE AND INPUT CURRENT VS. TIME
% WRO 2026 Future Engineers | engineering_journal/2-elec.md
%
% This standalone simulation uses the EXACT illustrative battery model,
% load inventory, three demand multipliers, and local simulation function
% implemented in Electrical Figure 05. All curves are CALCULATED, NOT
% measurements or forecasts of the team's real battery runtime.
%
% SUPPLIER INPUTS: GNB 4S LiHV pack nominal 15.2 V, capacity 1.30 Ah;
% DFR1208 published minimum input 15 V; rated/working currents listed in
% the figure's engineering README.
% MODEL INPUTS: illustrative OCV vs SOC knots (Figure 04), 0.12-ohm pack
% resistance, 90%% buck efficiencies, assumed Pi/HAT/servo/button/voltmeter
% demand and 0.5x/1.0x/1.5x load multipliers (Figure 05).
%
% Solid line: battery model at or above 15 V terminal, inside DFR1208's
% published input-voltage lower limit (NOT proof of regulated 12 V output).
% Dotted line: continuation of the same battery model below that limit;
% DFR1208 output and load behavior are not modeled or established there.
%
% Base MATLAB R2020a+; no toolboxes or external MATLAB files.
% Produces results/06_soc_and_current.png (240 DPI) and
%          results/simulation_inputs.mat. No CSV or summary.txt.
%
% Run the ENTIRE script.

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

%% 4. Publication styling: pure white background and high-contrast text
ink      = [0.10 0.15 0.23];
muted    = [0.33 0.39 0.47];
gridGrey = [0.85 0.88 0.92];
blue     = [0.17 0.40 0.70];
teal     = [0.10 0.51 0.46];
purple   = [0.52 0.34 0.66];
warn     = [0.72 0.27 0.24];
colors   = [blue;teal;purple];

fig=figure('Color',[1 1 1],'Units','pixels','Position',[65 65 1480 950]);
set(fig,'InvertHardcopy','off');
annotation(fig,'textbox',[0.066 0.925 0.88 0.052], ...
    'String','06  |  Simulated battery SOC and current vs. time', ...
    'FontName','Arial','FontSize',21,'FontWeight','bold', ...
    'Color',ink,'LineStyle','none','Interpreter','none');
annotation(fig,'textbox',[0.067 0.881 0.88 0.040], ...
    'String','GNB 4S LiHV model  |  identical assumptions and demand scenarios to Figure 05', ...
    'FontName','Arial','FontSize',12.5,'Color',muted, ...
    'LineStyle','none','Interpreter','none');

% Both subplots use the same scenario colors as Figure 05. They show the
% same model outputs, not two separately fitted datasets.
timeEnd=max(cellfun(@(r) max(r.timeMinutes),runs));
timeEnd=max(5,ceil(timeEnd/5)*5);

axSOC=axes('Parent',fig,'Position',[0.135 0.546 0.745 0.262], ...
    'Color',[1 1 1]);
hold(axSOC,'on');
axCurrent=axes('Parent',fig,'Position',[0.135 0.235 0.745 0.229], ...
    'Color',[1 1 1]);
hold(axCurrent,'on');

legendHandles=gobjects(1,N);
minCurrent=inf;
maxCurrent=0;
for k=1:N
    r=runs{k};
    valid=r.terminalV>=p.converter12MinInputV;

    socPercent=100*r.SOC;
    socValid=socPercent;
    socValid(~valid)=nan;
    socBelow=socPercent;
    socBelow(valid)=nan;

    currentValid=r.batteryCurrentA;
    currentValid(~valid)=nan;
    currentBelow=r.batteryCurrentA;
    currentBelow(valid)=nan;

    legendHandles(k)=plot(axSOC,r.timeMinutes,socValid,'-', ...
        'Color',colors(k,:),'LineWidth',2.8);
    plot(axSOC,r.timeMinutes,socBelow,':', ...
        'Color',colors(k,:),'LineWidth',2.1,'HandleVisibility','off');
    plot(axCurrent,r.timeMinutes,currentValid,'-', ...
        'Color',colors(k,:),'LineWidth',2.8);
    plot(axCurrent,r.timeMinutes,currentBelow,':', ...
        'Color',colors(k,:),'LineWidth',2.1,'HandleVisibility','off');

    % Connect the valid and extrapolated segments with a short dotted
    % crossing segment to avoid a visual gap at the 15 V boundary.
    crossing=find(diff(valid)<0,1,'first');
    if ~isempty(crossing)
        j=crossing:crossing+1;
        plot(axSOC,r.timeMinutes(j),socPercent(j),':', ...
            'Color',colors(k,:),'LineWidth',2.1,'HandleVisibility','off');
        plot(axCurrent,r.timeMinutes(j),r.batteryCurrentA(j),':', ...
            'Color',colors(k,:),'LineWidth',2.1,'HandleVisibility','off');
        plot(axSOC,r.timeMinutes(crossing+1),socPercent(crossing+1),'o', ...
            'MarkerFaceColor','white','MarkerEdgeColor',colors(k,:), ...
            'MarkerSize',7,'LineWidth',1.6,'HandleVisibility','off');
        plot(axCurrent,r.timeMinutes(crossing+1),r.batteryCurrentA(crossing+1),'o', ...
            'MarkerFaceColor','white','MarkerEdgeColor',colors(k,:), ...
            'MarkerSize',7,'LineWidth',1.6,'HandleVisibility','off');
    end
    minCurrent=min(minCurrent,min(r.batteryCurrentA));
    maxCurrent=max(maxCurrent,max(r.batteryCurrentA));
end

% Solid/dotted interpretation is common to both graphs.
set(axSOC,'XLim',[0 timeEnd],'YLim',[0 105], ...
    'YTick',0:20:100,'FontName','Arial','FontSize',11.5, ...
    'XColor',ink,'YColor',ink,'LineWidth',1.0,'TickLength',[0 0], ...
    'Box','off','XGrid','on','YGrid','on', ...
    'GridColor',gridGrey,'GridAlpha',0.72,'Layer','bottom');
axSOC.Color=[1 1 1];
ylabel(axSOC,'Modeled state of charge (%)', ...
    'Color',ink,'FontName','Arial','FontSize',12.2,'FontWeight','bold');
% Keep upper panel x axis unlabeled; the lower panel is the shared time axis.
set(axSOC,'XTickLabel',[]);

lg=legend(axSOC,legendHandles,p.scenarioNames, ...
    'Location','southwest','FontName','Arial','FontSize',10.6, ...
    'TextColor',ink,'Color',[1 1 1], ...
    'EdgeColor',gridGrey,'Box','on');

currentUpper=ceil((maxCurrent+0.12)/0.5)*0.5;
currentUpper=max(2.5,currentUpper);
set(axCurrent,'XLim',[0 timeEnd],'YLim',[0 currentUpper], ...
    'FontName','Arial','FontSize',11.5, ...
    'XColor',ink,'YColor',ink,'LineWidth',1.0,'TickLength',[0 0], ...
    'Box','off','XGrid','on','YGrid','on', ...
    'GridColor',gridGrey,'GridAlpha',0.72,'Layer','bottom');
axCurrent.Color=[1 1 1];
xlabel(axCurrent,'Simulated elapsed time (minutes)', ...
    'Color',ink,'FontName','Arial','FontSize',12.2,'FontWeight','bold');
ylabel(axCurrent,'Modeled battery input current (A)', ...
    'Color',ink,'FontName','Arial','FontSize',12.2,'FontWeight','bold');

% The interpretation note is intentionally placed in the space between
% panels, not inside the plotting axes where it can obscure the curves.
annotation(fig,'textbox',[0.135 0.492 0.745 0.029], ...
    'String',['Solid: modeled terminal voltage >= 15 V   |   ', ...
              'Dotted: extrapolation below DFR1208 minimum VIN'], ...
    'FontName','Arial','FontSize',10.6,'Color',muted, ...
    'LineStyle','none','Interpreter','none');

normal=runs{2};
if isfinite(normal.firstBelow15Min)
    firstCrossing=sprintf('%.1f min',normal.firstBelow15Min);
else
    firstCrossing='not reached';
end
summaryLine=sprintf(['1.0x regulated-load budget: %.2f W input (assumed eta=90%%)  |  ', ...
                     'Initial SOC: 100%% (model)  |  15 V crossing: %s'], ...
    PconverterInput_W,firstCrossing);
annotation(fig,'textbox',[0.066 0.133 0.89 0.046], ...
    'String',summaryLine,'FontName','Arial','FontSize',10.9, ...
    'Color',ink,'FontWeight','bold', ...
    'LineStyle','none','Interpreter','none');

%% 5. Save exactly one figure and model inputs
scriptDir=fileparts(mfilename('fullpath'));
if isempty(scriptDir),scriptDir=pwd;end
outDir=fullfile(scriptDir,'results');
if ~exist(outDir,'dir'),mkdir(outDir);end
figurePath=fullfile(outDir,'06_soc_and_current.png');
exportgraphics(fig,figurePath,'Resolution',240,'BackgroundColor','white');
save(fullfile(outDir,'simulation_inputs.mat'), ...
    'p','runs','P5_W','P12_W','PconverterInput_W', ...
    'PdirectNom_W','PtotalNom_W','IbatNom_A');

fprintf('\nYBR-SUNFLOWER | FIGURE 06/11 | BATTERY SOC AND CURRENT\n');
fprintf('Model sources: GNB pack 15.2 V nominal, %.2f Ah; all SOC/OCV points assumed.\n',p.capacityAh);
fprintf('Figure 01 budget: P5 = %.4f W; P12 = %.4f W\n',P5_W,P12_W);
fprintf('Figure 05 constant-power model: regulated input demand = %.4f W at 1.0x\n',PconverterInput_W);
fprintf('Equivalent pack resistance assumed %.3f ohm, voltmeter %.3f A\n', ...
    p.packSeriesResistanceOhm,p.voltmeterA);
for k=1:N
    r=runs{k};
    if isfinite(r.firstBelow15Min)
        crossingText=sprintf('%.2f min',r.firstBelow15Min);
    else
        crossingText='not reached';
    end
    fprintf(['%s | initial SOC %.1f%% -> final %.1f%% | ', ...
        'battery current %.3f -> %.3f A | 15 V crossing %s | %s\n'], ...
        p.scenarioNames{k},100*r.SOC(1),100*r.SOC(end), ...
        r.batteryCurrentA(1),r.batteryCurrentA(end), ...
        crossingText,r.stopReason);
end
fprintf('Saved: %s\n',figurePath);
fprintf('NOT measured: SOC, current curves, inferred runtime or converter output.\n\n');

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
