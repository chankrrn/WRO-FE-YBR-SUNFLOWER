%% Electrical Figure 07
% DFR1208 INPUT HEADROOM VS. SIMULATED ELAPSED TIME
% WRO 2026 Future Engineers | engineering_journal/2-elec.md
%
% This is the FINAL battery-discharge figure selected for the electrical
% journal. The plot concentrates on the critical 15 V DFR1208 input limit.
% It reuses EXACTLY Figure 05/06's battery model, loads and assumptions.
%
% SOURCE-BACKED:
%   GNB LiHV 4S battery: nominal 15.2 V, capacity 1.30 Ah.
%   DFR1208 input operating range: 15-24 V (DFRobot product 2908).
%   Motor supplier: rated current <=0.550 A @12 V.
%   DFRobot RPLIDAR S2L: working current 0.400 A @5 V.
%   DFRobot SEN0253: nominal operating current 0.005 A.
%
% ASSUMED/ILLUSTRATIVE (NOT MEASURED):
%   OCV-vs-SOC knots and PCHIP interpolation; pack R=0.12 ohm;
%   Pi+camera+fan 2 A; both HATs 0.12 A; servo normal 0.15 A;
%   LED button 0.02 A; voltmeter 0.015 A; both buck efficiencies 0.90;
%   constant demand multipliers 0.5x, 1.0x and 1.5x.
%
% IMPORTANT INTERPRETATION:
%   The model estimates BATTERY TERMINAL voltage, not DFR1208 OUTPUT.
%   Headroom = V_terminal - 15 V. Positive headroom only screens VIN.
%   Dotted segments below zero are UNVALIDATED constant-power extrapolation.
%   The display is intentionally cropped shortly after first crossing:
%   it does not represent usable battery runtime or a safe discharge limit.
%
% Requires base MATLAB R2020a+; run the entire script.
% Produces results/07_12v_input_headroom.png (240 DPI) and
%          results/simulation_inputs.mat. No CSV or summary.txt.

clearvars; close all; clc;

%% 1. Inputs -- synchronized to Electrical Figures 01, 04, 05 and 06
p.project = 'YBR-SUNFLOWER | WRO 2026 Future Engineers';
p.figureID = '07/11';
p.evidenceClass = 'Manufacturer input limit + illustrative battery simulation';
p.cells = 4;
p.capacityAh = 1.30;                 % GNB product nameplate [Ah]
p.nominalPackV = 15.2;               % GNB product nameplate [V]
p.lihvFullCellRefV = 4.35;            % LiHV chemistry reference [V/cell]
p.initialSOC = 1.00;                 % assumed fully charged initial SOC
p.OCV_SOC_knots = 0:0.10:1.00;      % illustrative -- same as Figures 04-06
p.OCV_cell_V = [3.30 3.55 3.66 3.72 3.76 3.80 ...
                3.84 3.90 4.00 4.15 4.35];
p.interpolationMethod = 'pchip';
p.packSeriesResistanceOhm = 0.12;    % pack + contact resistance [ohm], assumed
p.converter12MinInputV = 15.0;       % DFR1208 published minimum VIN [V]
p.converter12MaxInputV = 24.0;       % DFR1208 published maximum VIN [V]
p.converter5MinInputV = 6.0;         % DFR1202 published minimum VIN [V]
p.eta5 = 0.90;                       % MODEL efficiency, not guaranteed
p.eta12 = 0.90;                      % MODEL efficiency, not guaranteed

% Exactly the Figure 01/05/06 normal-load inventory.
p.V5 = 5.0;
p.V12 = 12.0;
p.V3v3 = 3.3;
p.piCameraCoolerA = 2.00;           % assumed @5 V
p.lidarWorkingA = 0.400;            % DFRobot working point @5 V
p.hatLogicA = 0.120;                % assumed @5 V
p.servoNormalA = 0.150;             % 17g servo generic no-load scenario @5 V
p.ledButtonA = 0.020;              % assumed @3.3 V
p.imuA = 0.005;                    % DFRobot SEN0253 @3.3 V
p.motorRatedBoundA = 0.550;        % CHP motor published rated-current bound
p.voltmeterA = 0.015;              % assumed pack-powered voltmeter

p.scenarioNames = {'Light demand  |  0.5x', ...
                   'Planning demand  |  1.0x', ...
                   'Heavy demand  |  1.5x'};
p.scenarioScale = [0.5 1.0 1.5];
p.stepSeconds = 1.0;
p.maxSimMinutes = 120;
p.demoStopTerminalV = 13.2;         % demonstration endpoint, NOT cutoff
p.minutesAfterCrossingDisplayed = 8; % visual crop to keep focus on headroom

%% 2. Sanity checks and Figure 01 load-budget reconciliation
assert(p.cells==4 && p.capacityAh>0 && p.initialSOC>0 && p.initialSOC<=1);
assert(numel(p.OCV_SOC_knots)==numel(p.OCV_cell_V));
assert(all(diff(p.OCV_SOC_knots)>0) && all(diff(p.OCV_cell_V)>0));
assert(abs(p.OCV_cell_V(end)-p.lihvFullCellRefV)<1e-9);
assert(p.packSeriesResistanceOhm>=0 && p.stepSeconds>0 && p.maxSimMinutes>0);
assert(p.eta5>0 && p.eta5<=1 && p.eta12>0 && p.eta12<=1);
assert(all(p.scenarioScale>0) && p.demoStopTerminalV<p.converter12MinInputV);

P5_W = p.V5 * (p.piCameraCoolerA+p.lidarWorkingA + ...
       p.hatLogicA+p.servoNormalA) + ...
       p.V3v3*(p.ledButtonA+p.imuA);
P12_W = p.V12*p.motorRatedBoundA;
PconverterInput_W = P5_W/p.eta5 + P12_W/p.eta12;
PdirectNom_W = p.nominalPackV*p.voltmeterA;
PtotalNom_W = PconverterInput_W + PdirectNom_W;
IbatNom_A = PconverterInput_W/p.nominalPackV + p.voltmeterA;
nominalHeadroomV = p.nominalPackV - p.converter12MinInputV;

assert(abs(P5_W-13.4325)<1e-9, 'Figure 01 5-V inventory has changed.');
assert(abs(P12_W-6.6000)<1e-9, 'Figure 01 motor inventory has changed.');
assert(abs(nominalHeadroomV-0.20)<1e-9, 'Unexpected nominal VIN headroom.');

%% 3. Simulate battery voltage and compute VIN margin from the SAME model
N = numel(p.scenarioScale);
runs = cell(1,N);
firstCrossingMinutes = nan(1,N);
for k=1:N
    requestedPower_W = p.scenarioScale(k)*PconverterInput_W;
    runs{k} = simulateBattery(p,requestedPower_W);
    runs{k}.headroomV = runs{k}.terminalV-p.converter12MinInputV;
    firstCrossingMinutes(k) = runs{k}.firstBelow15Min;
end

finiteCrossings=firstCrossingMinutes(isfinite(firstCrossingMinutes));
if isempty(finiteCrossings)
    plotEndMinutes=min(p.maxSimMinutes,60);
else
    plotEndMinutes=ceil((max(finiteCrossings)+ ...
        p.minutesAfterCrossingDisplayed)/5)*5;
    plotEndMinutes=min(plotEndMinutes,p.maxSimMinutes);
end
plotEndMinutes=max(5,plotEndMinutes);

%% 4. Same white-background / dark-text publication style as Figures 01-06
ink      = [0.10 0.15 0.23];
muted    = [0.33 0.39 0.47];
gridGrey = [0.85 0.88 0.92];
blue     = [0.17 0.40 0.70];
teal     = [0.10 0.51 0.46];
purple   = [0.52 0.34 0.66];
warn     = [0.72 0.27 0.24];
colors   = [blue;teal;purple];

fig=figure('Color',[1 1 1],'Units','pixels', ...
    'Position',[70 70 1480 880]);
set(fig,'InvertHardcopy','off');
annotation(fig,'textbox',[0.066 0.922 0.88 0.056], ...
    'String','07  |  DFR1208 input-voltage headroom', ...
    'FontName','Arial','FontSize',21,'FontWeight','bold', ...
    'Color',ink,'LineStyle','none','Interpreter','none');
annotation(fig,'textbox',[0.067 0.869 0.88 0.044], ...
    'String','Battery terminal voltage minus the published 15 V minimum converter input', ...
    'FontName','Arial','FontSize',12.4,'Color',muted, ...
    'LineStyle','none','Interpreter','none');

ax=axes('Parent',fig,'Position',[0.118 0.302 0.794 0.505], ...
    'Color',[1 1 1]);
hold(ax,'on');
yLow=-0.82; yHigh=2.60;

% Meaningful, pale shaded region: outside published DFR1208 VIN range.
patch(ax,[0 plotEndMinutes plotEndMinutes 0], ...
    [yLow yLow 0 0],[1.0 0.944 0.944], ...
    'FaceAlpha',0.72,'EdgeColor','none','HandleVisibility','off');

legendHandles=gobjects(1,N);
for k=1:N
    r=runs{k};
    selected = r.timeMinutes <= plotEndMinutes;
    if isfinite(r.firstBelow15Min)
        selected = selected & (r.timeMinutes <= ...
            r.firstBelow15Min+p.minutesAfterCrossingDisplayed);
    end
    t=r.timeMinutes(selected);
    h=r.headroomV(selected);
    valid=h>=0;

    hAbove=h;hAbove(~valid)=nan;
    hBelow=h;hBelow(valid)=nan;
    legendHandles(k)=plot(ax,t,hAbove,'-', ...
        'Color',colors(k,:),'LineWidth',2.8);
    plot(ax,t,hBelow,':','Color',colors(k,:), ...
        'LineWidth',2.2,'HandleVisibility','off');

    crossing=find(diff(valid)<0,1,'first');
    if ~isempty(crossing)
        j=crossing:crossing+1;
        plot(ax,t(j),h(j),':','Color',colors(k,:), ...
            'LineWidth',2.2,'HandleVisibility','off');
    end

    % Crossing markers and subtle vertical leaders under the zero line.
    if isfinite(r.firstBelow15Min) && r.firstBelow15Min<=plotEndMinutes
        tc=r.firstBelow15Min;
        plot(ax,tc,0,'o','MarkerSize',9, ...
            'MarkerFaceColor','white','MarkerEdgeColor',colors(k,:), ...
            'LineWidth',2.0,'HandleVisibility','off');
        plot(ax,[tc tc],[-0.01 -0.48],':', ...
            'Color',colors(k,:),'LineWidth',1.05, ...
            'HandleVisibility','off');
        shortNames={'Light','Planning','Heavy'};
        text(ax,tc,-0.53, ...
            sprintf('%s\n%.1f min',shortNames{k},tc), ...
            'Color',colors(k,:),'FontName','Arial','FontSize',10.3, ...
            'FontWeight','bold','HorizontalAlignment','center', ...
            'VerticalAlignment','top','Interpreter','none');
    end
end

% Zero headroom is the manufacturer's MINIMUM VIN limit, not an output-dropout curve.
plot(ax,[0 plotEndMinutes],[0 0],'--', ...
    'Color',warn,'LineWidth',1.7,'HandleVisibility','off');
text(ax,plotEndMinutes*0.985,0.13, ...
    '0 V margin = 15.0 V minimum VIN', ...
    'Color',warn,'FontName','Arial','FontSize',10.8, ...
    'FontWeight','bold','HorizontalAlignment','right');

set(ax,'XLim',[0 plotEndMinutes],'YLim',[yLow yHigh], ...
    'XColor',ink,'YColor',ink,'FontName','Arial', ...
    'FontSize',11.5,'LineWidth',1.0,'TickLength',[0 0], ...
    'Box','off','XGrid','on','YGrid','on', ...
    'GridColor',gridGrey,'GridAlpha',0.70,'Layer','top');
ax.Color=[1 1 1];
ax.YTick=[-0.5 0 0.5 1.0 1.5 2.0 2.5];
xlabel(ax,'Simulated elapsed time (minutes)', ...
    'Color',ink,'FontName','Arial','FontSize',12.2,'FontWeight','bold');
ylabel(ax,'DFR1208 input headroom (V)', ...
    'Color',ink,'FontName','Arial','FontSize',12.2,'FontWeight','bold');

legend(ax,legendHandles,p.scenarioNames, ...
    'Location','northeast','FontName','Arial','FontSize',10.8, ...
    'TextColor',ink,'Color',[1 1 1], ...
    'EdgeColor',gridGrey,'Box','on');

%% 5. Footer: manufacturer nominal comparison and model limitations
annotation(fig,'textbox',[0.066 0.177 0.89 0.053], ...
    'String',sprintf(['GNB nominal %.1f V - DFR1208 minimum %.1f V = ', ...
                      '+%.2f V nominal margin   |   assumed pack R = %.2f ohm'], ...
                      p.nominalPackV,p.converter12MinInputV, ...
                      nominalHeadroomV,p.packSeriesResistanceOhm), ...
    'FontName','Arial','FontSize',11.0,'FontWeight','bold', ...
    'Color',ink,'LineStyle','none','Interpreter','none');

%% 6. Save ONLY the figure intended for the electrical journal
scriptDir=fileparts(mfilename('fullpath'));
if isempty(scriptDir),scriptDir=pwd;end
outDir=fullfile(scriptDir,'results');
if ~exist(outDir,'dir'),mkdir(outDir);end
figurePath=fullfile(outDir,'07_12v_input_headroom.png');
exportgraphics(fig,figurePath,'Resolution',240,'BackgroundColor','white');
save(fullfile(outDir,'simulation_inputs.mat'), ...
    'p','runs','nominalHeadroomV','firstCrossingMinutes', ...
    'P5_W','P12_W','PconverterInput_W','PdirectNom_W', ...
    'PtotalNom_W','IbatNom_A','plotEndMinutes');

fprintf('\nYBR-SUNFLOWER | ELECTRICAL FIGURE 07 | 12 V INPUT HEADROOM\n');
fprintf('GNB published nominal: %.2f V; DFR1208 min VIN: %.2f V\n', ...
    p.nominalPackV,p.converter12MinInputV);
fprintf('Nameplate-only nominal input margin: +%.2f V\n',nominalHeadroomV);
fprintf('Figure 01 5 V output demand (incl. ideal 3.3 V): %.4f W\n',P5_W);
fprintf('Figure 01 motor demand: %.4f W; assumed buck input power: %.4f W\n', ...
    P12_W,PconverterInput_W);
for k=1:N
    if isfinite(firstCrossingMinutes(k))
        fprintf('%s | MODEL 15 V crossing: %.2f minutes\n', ...
            p.scenarioNames{k},firstCrossingMinutes(k));
    else
        fprintf('%s | MODEL 15 V crossing: not reached\n',p.scenarioNames{k});
    end
end
fprintf('Saved only: %s\n',figurePath);
fprintf('No measured discharge, converter dropout or usable runtime is inferred.\n\n');

%% Local simulation -- self-contained and identical to Figures 05/06
function r=simulateBattery(p,Pload_W)
    % V=OCV-I*R; I=Pload/V + Ivoltmeter.
    % Thus V^2-(OCV-R*Ivoltmeter)*V+R*Pload=0.
    % Use the high-voltage root of this equivalent-circuit equation.
    maxSteps=ceil(p.maxSimMinutes*60/p.stepSeconds)+1;
    t=nan(maxSteps,1); soc=nan(maxSteps,1);
    ocv=nan(maxSteps,1); V=nan(maxSteps,1); I=nan(maxSteps,1);
    t(1)=0;soc(1)=p.initialSOC;
    reason='Maximum visualization time reached';
    for j=1:maxSteps
        ocv(j)=p.cells*interp1(p.OCV_SOC_knots,p.OCV_cell_V, ...
            soc(j),p.interpolationMethod);
        adjustedOCV=ocv(j)-p.packSeriesResistanceOhm*p.voltmeterA;
        discriminant=adjustedOCV^2-4*p.packSeriesResistanceOhm*Pload_W;
        if discriminant<0
            reason='Equivalent-circuit load infeasible';break;
        end
        V(j)=(adjustedOCV+sqrt(max(0,discriminant)))/2;
        I(j)=Pload_W/V(j)+p.voltmeterA;
        if V(j)<=p.demoStopTerminalV
            reason='Demonstration endpoint reached; not a safe cutoff';break;
        end
        if soc(j)<=0
            reason='Hypothetical SOC exhausted';break;
        end
        if j==maxSteps,break;end
        soc(j+1)=max(0,soc(j)-I(j)*p.stepSeconds/(3600*p.capacityAh));
        t(j+1)=t(j)+p.stepSeconds;
    end
    ok=isfinite(V);
    t=t(ok);soc=soc(ok);ocv=ocv(ok);V=V(ok);I=I(ok);
    below=find(V<p.converter12MinInputV,1,'first');
    if isempty(below)
        crossMin=nan;
    elseif below==1
        crossMin=0;
    else
        t1=t(below-1);t2=t(below);
        v1=V(below-1);v2=V(below);
        if v1==v2
            crossMin=t2/60;
        else
            crossMin=(t1+(p.converter12MinInputV-v1)* ...
                (t2-t1)/(v2-v1))/60;
        end
    end
    r=struct('timeMinutes',t/60,'SOC',soc,'OCV_V',ocv, ...
        'terminalV',V,'batteryCurrentA',I,'requestedPowerW',Pload_W, ...
        'firstBelow15Min',crossMin,'stopReason',reason);
end
