%% Electrical Figure 04
% ILLUSTRATIVE LiHV OPEN-CIRCUIT VOLTAGE VS STATE OF CHARGE
% WRO 2026 Future Engineers | engineering_journal/2-elec.md
%
% PURPOSE: Show the difference between a battery's published nameplate
% voltage and a hypothetical open-circuit voltage (OCV) vs SOC shape.
%
% EVIDENCE STATUS:
% PUBLISHED/SUPPLIER: GNB 4S1P LiHV 15.2 V nominal, 1300 mAh.
% CHEMISTRY REFERENCE: 4.35 V/cell upper LiHV reference => 17.4 V/pack.
% MODEL ASSUMPTION: EVERY OCV/SOC KNOT BELOW IS ILLUSTRATIVE.
% These OCV values were NOT measured on the team's pack and are NOT a
% calibrated voltage-to-SOC lookup, battery discharge curve, or charge
% or discharge operating instruction.
%
% The model is designed to serve as the OCV assumption for subsequent
% battery headroom studies (Figures 05-07), provided those scripts use
% the SAME knots and interpolation method.
%
% Base MATLAB R2020a+; run entire script. No toolboxes required.
% Output: results/04_assumed_voltage_vs_soc.png (240 DPI) and
%         results/simulation_inputs.mat. No CSV and no summary.txt.

clearvars; close all; clc;

%% 1. USER INPUTS -- source-based pack ratings vs hypothetical OCV shape
p.project = 'YBR-SUNFLOWER | WRO 2026 Future Engineers';
p.figureID = '04/11';
p.evidenceClass = 'Published battery ratings + illustrative OCV/SOC knots';

p.cells = 4;                       % GNB product: 4S1P pack
p.capacityAh = 1.30;              % GNB nameplate: 1300 mAh
p.nominalPackV = 15.2;            % GNB nameplate: 15.2 V nominal
p.lihvFullCellRefV = 4.35;       % LiHV chemistry reference, NOT measured
p.fullPackReferenceV = p.cells * p.lihvFullCellRefV; % 17.4 V

% HYPOTHETICAL battery rest-OCV at each SOC knot; these are NOT measured.
% Retained from the team's earlier Step 02 example model for traceability.
% SOC=0 at 3.30 V/cell is a MODEL ENDPOINT, NOT safe cutoff guidance.
p.socKnots = [0 .10 .20 .30 .40 .50 .60 .70 .80 .90 1.00];
p.ocvPerCellV = [3.30 3.55 3.66 3.72 3.76 3.80 ...
                  3.84 3.90 4.00 4.15 4.35];
p.interpolationMethod = 'pchip'; % monotone shape-preserving interpolation
p.samples = 700;

%% 2. Validation and reproducible open-circuit voltage calculation
assert(p.cells == 4 && p.capacityAh > 0 && p.nominalPackV > 0);
assert(numel(p.socKnots) == numel(p.ocvPerCellV));
assert(p.socKnots(1) == 0 && p.socKnots(end) == 1);
assert(all(diff(p.socKnots) > 0));
assert(all(diff(p.ocvPerCellV) > 0));
assert(p.samples >= 100);
assert(abs(p.ocvPerCellV(end) - p.lihvFullCellRefV) < 1e-9);
assert(p.fullPackReferenceV > p.nominalPackV);

socFraction = linspace(0,1,p.samples);
socPercent = 100*socFraction;
packOCV = p.cells * interp1( ...
    p.socKnots,p.ocvPerCellV,socFraction,p.interpolationMethod);
packKnotVoltages = p.cells * p.ocvPerCellV;

% The nominal voltage is a *nameplate* characteristic, not a point on a
% published SOC curve. The model's crossing of 15.2 V at an SOC value
% must never be interpreted as a measured or transferable calibration.
nominalLabelVoltage = p.nominalPackV;
nominalEnergyWh = p.nominalPackV * p.capacityAh; % 19.76 Wh nameplate arithmetic

%% 3. Consistent publication palette -- WHITE figure AND WHITE axes
ink      = [0.10 0.15 0.23];
muted    = [0.33 0.39 0.47];
gridGrey = [0.85 0.88 0.92];
blue     = [0.17 0.40 0.70]; % illustrative OCV model
teal     = [0.10 0.51 0.46]; % LiHV 17.4 V full-charge reference
amber    = [0.77 0.40 0.14]; % published pack nominal voltage

fig = figure('Color','white','Units','pixels', ...
    'Position',[65 65 1480 860]);
set(fig,'InvertHardcopy','off');

annotation(fig,'textbox',[0.066 0.923 0.87 0.058], ...
    'String','04  |  Battery open-circuit voltage vs. state of charge', ...
    'FontName','Arial','FontSize',21,'FontWeight','bold', ...
    'Color',ink,'LineStyle','none','Interpreter','none');
annotation(fig,'textbox',[0.067 0.872 0.87 0.042], ...
    'String','GNB 4S LiHV ratings with an illustrative, unmeasured OCV model', ...
    'FontName','Arial','FontSize',12.5,'Color',muted, ...
    'LineStyle','none','Interpreter','none');

%% 4. Graph: hypothetical OCV curve and source-based reference voltages
ax = axes('Parent',fig, ...
    'Position',[0.135 0.295 0.780 0.520], ...
    'Color',[1 1 1]);
hold(ax,'on');

% A restrained, light dashed line marks published nameplate voltage.
hNominal = plot(ax,[0 100],[nominalLabelVoltage nominalLabelVoltage], ...
    '--','Color',amber,'LineWidth',1.85);

% Smooth but shape-preserving hypothetical OCV curve, plus model knots.
hOCV = plot(ax,socPercent,packOCV, ...
    '-','Color',blue,'LineWidth',3.0);
hKnots = plot(ax,100*p.socKnots,packKnotVoltages,'o', ...
    'MarkerSize',5.7,'MarkerFaceColor','white', ...
    'MarkerEdgeColor',blue,'LineWidth',1.5, ...
    'LineStyle','none');

% Published chemistry-based endpoint reference, NOT a pack measurement.
plot(ax,100,p.fullPackReferenceV,'o', ...
    'MarkerSize',10.0,'MarkerFaceColor',teal, ...
    'MarkerEdgeColor','white','LineWidth',1.7);

% Annotation positions deliberately avoid line/axis overlaps.
text(ax,96,17.62,'17.4 V  |  4 x 4.35 V/cell reference', ...
    'FontName','Arial','FontSize',10.7,'Color',teal, ...
    'FontWeight','bold','HorizontalAlignment','right');
text(ax,97,15.30,'15.2 V nominal pack label', ...
    'FontName','Arial','FontSize',10.8,'Color',amber, ...
    'FontWeight','bold','HorizontalAlignment','right');

% Compact plot note: OCV points are not a voltage-to-SOC calibration.
text(ax,3,17.50,'Hypothetical OCV shape  |  model knots shown as circles', ...
    'FontName','Arial','FontSize',10.4,'Color',muted, ...
    'HorizontalAlignment','left');

set(ax,'XLim',[0 100],'YLim',[12.9 17.85], ...
    'XTick',0:10:100,'YTick',13:0.5:17.5, ...
    'FontName','Arial','FontSize',11.4, ...
    'XColor',ink,'YColor',ink,'LineWidth',1.0, ...
    'TickLength',[0 0],'Box','off', ...
    'XGrid','on','YGrid','on', ...
    'GridColor',gridGrey,'GridAlpha',0.70, ...
    'Layer','bottom');
ax.Color = [1 1 1]; % Explicitly override MATLAB dark-mode axis defaults.

xlabel(ax,'Illustrative state of charge (%)', ...
    'Color',ink,'FontName','Arial','FontSize',12.2,'FontWeight','bold');
ylabel(ax,'Modeled open-circuit pack voltage (V)', ...
    'Color',ink,'FontName','Arial','FontSize',12.2,'FontWeight','bold');

lg = legend(ax,[hOCV hKnots hNominal],{ ...
    'Hypothetical OCV curve (PCHIP)', ...
    'Assumed SOC-voltage knots', ...
    'GNB battery nameplate: 15.2 V nominal'}, ...
    'Location','southeast','FontName','Arial','FontSize',10.3, ...
    'TextColor',ink,'Color',[1 1 1], ...
    'EdgeColor',gridGrey,'Box','on');

%% 5. Footer: meaningful specification vs assumptions, well below X-axis
headline = sprintf([ ...
    'GNB pack: %dS1P  |  %.2f V nominal  |  %.2f Ah  |  ', ...
    'nominal energy %.2f Wh'], ...
    p.cells,p.nominalPackV,p.capacityAh,nominalEnergyWh);
annotation(fig,'textbox',[0.066 0.168 0.89 0.048], ...
    'String',headline, ...
    'FontName','Arial','FontSize',11.2,'FontWeight','bold', ...
    'Color',ink,'LineStyle','none','Interpreter','none');

%% 6. Save figure and calculation inputs; display numeric results
scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir), scriptDir = pwd; end
outDir = fullfile(scriptDir,'results');
if ~exist(outDir,'dir'), mkdir(outDir); end
figurePath = fullfile(outDir,'04_assumed_voltage_vs_soc.png');
exportgraphics(fig,figurePath,'Resolution',240,'BackgroundColor','white');
save(fullfile(outDir,'simulation_inputs.mat'), ...
    'p','socFraction','socPercent','packOCV','packKnotVoltages', ...
    'nominalEnergyWh');

fprintf('\nYBR-SUNFLOWER | FIGURE 04/11 | LiHV OCV vs SOC\n');
fprintf('GNB label: %dS1P, %.2f V nominal, %.2f Ah\n', ...
    p.cells,p.nominalPackV,p.capacityAh);
fprintf('Nominal nameplate energy: %.2f Wh\n',nominalEnergyWh);
fprintf('Illustrative full-charge reference: %.2f V (%d x %.2f V)\n', ...
    p.fullPackReferenceV,p.cells,p.lihvFullCellRefV);
fprintf('OCV/SOC curve: %d ASSUMED knots, %s interpolation.\n', ...
    numel(p.socKnots),p.interpolationMethod);
fprintf('ILLUSTRATIVE endpoints: %.2f V (SOC=0) to %.2f V (SOC=100%%).\n', ...
    packKnotVoltages(1),packKnotVoltages(end));
fprintf('WARNING: model SOC axis is not pack-calibrated and gives no cutoff guidance.\n');
fprintf('Saved PNG: %s\n\n',figurePath);
