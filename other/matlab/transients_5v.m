%% Five-volt supply voltage-drop and transient-load sensitivity

clearvars; close all; clc;

%% USER INPUTS
p.nominalV = 5.05;         % V, modeled DFR1202 output, ASSUMPTION
p.minDesignV = 4.80;       % V, conservative comparison line, not official trip threshold
p.piLowWarningV = 4.63;    % V, nominal Raspberry Pi undervoltage warning threshold
p.piIdleA = 1.4;           % A, modeled Pi + peripheral current, ASSUMPTION
p.piPeakExtraA = 1.3;      % A, extra Pi processing load, ASSUMPTION
p.servoIdleA = 0.10;       % A, modeled servo current; not measured
p.servoPeakExtraA = 1.80;  % A, modeled steering transient; not measured
p.piCableOhm = 0.050;      % ohm, combined USB-C cable/contacts path, ASSUMPTION
p.regulatorOhm = 0.020;    % ohm, illustrative quasi-static regulator droop
p.regulatorTransientOhm = 0.045; % ohm, illustrative extra transient droop
p.regulatorTimeConstantS = 0.012; % s, illustrative regulator response constant
p.commonIsolatedOhm = 0.004; % ohm, assumed coupling when servo has separate rail
p.commonSharedOhm = 0.045; % ohm, assumed coupling when servo shares 5V path
p.durationS = 6.0;
p.stepS = 0.001;
p.resultsFolder = 'results';

assert(p.nominalV>0 && p.stepS>0 && p.durationS>0, 'Check time/voltage inputs');
assert(p.piIdleA>=0 && p.servoPeakExtraA>=0 && p.piCableOhm>=0, 'Invalid current or resistance');
assert(p.regulatorTimeConstantS>0, 'Regulator time constant must be positive');
if ~exist(p.resultsFolder,'dir'), mkdir(p.resultsFolder); end

t = (0:p.stepS:p.durationS)';
% Defined synthetic events: steering at 1.0-1.35 s and 3.1-3.55 s;
% compute load at 2.0-2.8 s and 3.25-4.15 s.
servoPulse = double((t>=1.00 & t<1.35) | (t>=3.10 & t<3.55));
computePulse = double((t>=2.00 & t<2.80) | (t>=3.25 & t<4.15));
Ipi = p.piIdleA + p.piPeakExtraA .* computePulse;
Iservo = p.servoIdleA + p.servoPeakExtraA .* servoPulse;

% Models deliberately separate the servo's assumed rail coupling from its
% operating voltage: THIS DOES NOT specify or endorse an actual servo hookup.
Visolated = modelRail(t, Ipi, Iservo, p, p.commonIsolatedOhm, p.piCableOhm);
Vshared = modelRail(t, Ipi, Iservo, p, p.commonSharedOhm, p.piCableOhm);
VhighCable = modelRail(t, Ipi, Iservo, p, p.commonIsolatedOhm, 0.110);
Vreference = modelRail(t, Ipi, 0*Iservo, p, 0, p.piCableOhm);

%% FIGURE 08 -- Modeled transient events and Pi input voltage
f8=figure('Color','w','Position',[100 100 1100 720]);
subplot(3,1,1); plot(t,Ipi,'LineWidth',1.6); ylabel('Pi-side current (A)'); grid on;
title('5 V rail under synthetic actuator and compute events');
subplot(3,1,2); plot(t,Iservo,'LineWidth',1.6); ylabel('Servo current (A)'); grid on;
subplot(3,1,3); plot(t,Visolated,'LineWidth',1.8); hold on;
plot(t,Vshared,'LineWidth',1.6); yline(p.minDesignV,'--','4.80 V design reference');
yline(p.piLowWarningV,':','4.63 V warning reference');
ylabel('Pi supply voltage (V)'); xlabel('Time (s)'); grid on;
legend('Lower assumed coupling','Higher assumed coupling','Location','best');
savePng(f8,fullfile(p.resultsFolder,'08_transient_timeline.png'));

%% FIGURE 09 -- Wiring assumptions comparison
f9=figure('Color','w','Position',[100 100 1150 520]);
plot(t,Vreference,'LineWidth',1.6); hold on;
plot(t,Visolated,'LineWidth',1.7);
plot(t,Vshared,'LineWidth',1.7);
plot(t,VhighCable,'LineWidth',1.7);
yline(p.minDesignV,'--','4.80 V'); yline(p.piLowWarningV,':','4.63 V');
grid on; xlabel('Time (s)'); ylabel('Modeled Pi input voltage (V)');
title(' Sensitivity to shared wiring and cable resistance');
legend('No servo transient','Lower shared impedance','Higher shared impedance','Longer/higher-R cable','Location','best');
savePng(f9,fullfile(p.resultsFolder,'09_wiring_scenarios.png'));

%% FIGURE 10 -- Voltage headroom against two reference levels
f10=figure('Color','w','Position',[100 100 1100 650]);
subplot(2,1,1); plot(t,Visolated-p.minDesignV,'LineWidth',1.7); hold on;
plot(t,Vshared-p.minDesignV,'LineWidth',1.7); yline(0,'--');
ylabel('Headroom above 4.80 V (V)'); grid on;
title('Headroom is an indicator, not a hardware qualification');
legend('Lower coupling','Higher coupling','Location','best');
subplot(2,1,2); plot(t,Visolated-p.piLowWarningV,'LineWidth',1.7); hold on;
plot(t,Vshared-p.piLowWarningV,'LineWidth',1.7); yline(0,'--');
ylabel('Headroom above 4.63 V (V)'); xlabel('Time (s)'); grid on;
savePng(f10,fullfile(p.resultsFolder,'10_voltage_headroom.png'));

%% FIGURE 11 -- Cable resistance x peak-load sweep (illustrative)
cableValues = linspace(0.01,0.14,24);
peakValues = linspace(0.20,3.50,26);
minimumVoltage = zeros(length(peakValues),length(cableValues));
for a=1:length(peakValues)
    for b=1:length(cableValues)
        pp=p; pp.piPeakExtraA=peakValues(a);
        current=pp.piIdleA+pp.piPeakExtraA.*computePulse;
        v=modelRail(t,current,Iservo,pp,pp.commonIsolatedOhm,cableValues(b));
        minimumVoltage(a,b)=min(v);
    end
end
f11=figure('Color','w','Position',[100 100 900 620]);
imagesc(cableValues,peakValues,minimumVoltage); set(gca,'YDir','normal');
colorbar; xlabel('Cable + contact resistance (ohm)');
ylabel('Additional Pi-side current step (A)');
title('Lowest Pi input voltage (V) across synthetic event schedule');
hold on; contour(cableValues,peakValues,minimumVoltage,[p.minDesignV p.minDesignV],'k','LineWidth',1.6);
savePng(f11,fullfile(p.resultsFolder,'11_resistance_load_sweep.png'));

%% Local functions -- MATLAB R2018b or newer
function voltage=modelRail(t,Ipi,Iservo,p,commonResistance,cableResistance)
    % Approximate converter response via a first-order filtered Pi load.
    % The fast part of a load step experiences an additional modeled droop.
    slow=zeros(size(Ipi)); slow(1)=Ipi(1);
    alpha=1-exp(-p.stepS/p.regulatorTimeConstantS);
    for n=2:length(t)
       slow(n)=slow(n-1)+alpha*(Ipi(n)-slow(n-1));
    end
    transientCurrent=Ipi-slow;
    voltage=p.nominalV - cableResistance.*Ipi ...
        - p.regulatorOhm.*slow ...
        - p.regulatorTransientOhm.*transientCurrent ...
        - commonResistance.*Iservo;
end
function savePng(fig,filename)
   set(fig,'PaperPositionMode','auto');
   print(fig,filename,'-dpng','-r160');
end
