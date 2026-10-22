%% Power architecture screening

clear; clc; close all;
scriptFolder = fileparts(mfilename('fullpath'));
if isempty(scriptFolder), scriptFolder = pwd; end
outputFolder = fullfile(scriptFolder,'results');
if ~exist(outputFolder,'dir'), mkdir(outputFolder); end

%% USER INPUTS: replace assumptions with datasheets and bench measurements
% Battery: GAONENG GNB 4S LiHV, nominal 15.2 V and 1.3 Ah.
battery.nominalV = 15.2;           % supplied label [V]
battery.capacityAh = 1.30;         % supplied label [Ah]
battery.fullCellV = 4.35;          % illustrative LiHV full-charge voltage; VERIFY actual charger/battery
battery.lowCellV = 3.50;           % illustrative cutoff for SCREENING ONLY, not BMS guidance
battery.cells = 4;
% Converter operating input limits: CHECK against exact purchased revisions.
buck5.minimumInputV = 6.0;        % DFR1202 published 6-24 V INPUT range
buck12.minimumInputV = 15.0;      % DFR1208 advertised 15-24 V input; verify datasheet
buck5.nominalOutputV = 5.0;
buck12.nominalOutputV = 12.0;
% Illustrative efficiency estimates: replace with measured efficiency-vs-load data.
buck5.efficiency = 0.90;
buck12.efficiency = 0.90;
% EXAMPLE currents, NOT measurements or component ratings. Edit freely.
loads = {'Pi 5 + camera','LiDAR S2L','HATs + button','Drive motor','Servo (rail unverified)'};
rail = {'5 V','5 V','5 V','12 V','5 V'}; % Verify motor and servo ACTUAL power routing!
currentA = [2.0, 0.5, 0.15, 1.0, 0.7];
% This assumed servo supply is NOT verified. Never wire from a plot!
% Added SEN0253 IMU is not modeled separately: add its stated 5 mA to a verified rail during measurements.

%% Sanity checks
assert(battery.cells == 4 && battery.capacityAh > 0);
assert(all(currentA >= 0) && buck5.efficiency > 0 && buck5.efficiency <= 1);
assert(buck12.efficiency > 0 && buck12.efficiency <= 1);
assert(numel(loads)==numel(currentA) && numel(rail)==numel(loads));

%% Figure 01: illustrative electrical power budget
voltage = zeros(size(currentA));
for k=1:numel(currentA)
    if strcmp(rail{k},'5 V'), voltage(k)=5; else, voltage(k)=12; end
end
powerW = voltage .* currentA;
p5 = sum(powerW(strcmp(rail,'5 V')));
p12 = sum(powerW(strcmp(rail,'12 V')));
pInput = p5/buck5.efficiency + p12/buck12.efficiency;
figure('Color','w','Position',[80 80 980 510]);
bar(powerW,'FaceColor',[0.2 0.43 0.68]);
set(gca,'XTick',1:numel(loads),'XTickLabel',loads,'XTickLabelRotation',22);
ylabel('Illustrative load power (W)');
title('Example load budget'); grid on;
for k=1:numel(powerW), text(k,powerW(k)+0.25,sprintf('%.1f W',powerW(k)),'HorizontalAlignment','center'); end
ylim([0,max(powerW)*1.25+1]);
exportgraphics(gcf,fullfile(outputFolder,'01_assumed_load_budget.png'),'Resolution',180);

%% Figure 02: allowable input-voltage screening, NOT output regulation prediction
fullV = battery.cells*battery.fullCellV;
lowV = battery.cells*battery.lowCellV;
Vin = linspace(lowV,fullV,250);
valid5 = Vin >= buck5.minimumInputV;
valid12 = Vin >= buck12.minimumInputV;
figure('Color','w','Position',[110 100 950 510]); hold on;
plot(Vin,double(valid5),'LineWidth',2);
plot(Vin,double(valid12),'LineWidth',2);
xline(battery.nominalV,'--','Nominal 15.2 V','LineWidth',1.2);
xlabel('Battery terminal voltage (V)');
ylabel('Meets assumed/minimum converter INPUT requirement (1 = yes)');
title('Input-range screening ONLY (not proof of output stability)');
yticks([0 1]); ylim([-0.12 1.18]); grid on;
legend('5 V converter: published min input','12 V converter: stated 15 V min input','Location','southwest');
exportgraphics(gcf,fullfile(outputFolder,'02_input_range_screen.png'),'Resolution',180);

%% Figure 03: illustrative battery current against voltage under fixed load
% Assumes constant output loads and constant converter efficiency. No discharge curve modeled.
Iin = pInput ./ Vin;
figure('Color','w','Position',[130 120 950 510]);
plot(Vin,Iin,'LineWidth',2); hold on;
xline(buck12.minimumInputV,'--','12 V buck stated minimum');
xline(battery.nominalV,':','Battery nominal');
xlabel('Battery voltage (V)'); ylabel('Estimated input current (A)');
title('Battery input current estimate | assumed fixed loads'); grid on;
exportgraphics(gcf,fullfile(outputFolder,'03_estimated_input_current.png'),'Resolution',180);
