clear all
close all
clc

% Operating conditions
VCC = 12;
Fsw = 20e3;
Dmin = 2/100;    %  1 %
Dmax = (1-Dmin); % 99 %
D = Dmax; % 98/100;      % Percent value
bs_rip = 6/100;  % Maximum allowable ripple

disp('Bootstrap Circuit Calculations');

% MOSFET Selection
disp(' - N-Channel Mosfet: PSMN1R4-30YLD');
Rdson = 1.85e-3;
Qgate = 27.6e-9;  % Total gate charge
VGS_min = 4.5;
VGS_th = 2.2;
IGSS = 100e-9;
SW_Ton = 23e-9;
SW_Toff = 31.5e-9;
SW_Tr = 28e-9;
SW_Tf = 20.6e-9;
SW_QGS = 7.2e-9;
SW_QGD = 8.5e-9;
SW_CGD = 251e-12;
C_in = 3840e-12;
C_out = 1785e-12;

% Driver Selection
disp(' - Half-bridge driver: 2ED2184S06F');
ILK = 12.5e-6; % Offset Supply Leakage Current
IQBS = 170e-6; % Quiescent VBS Supply Current
IQCC = 550e-6; % Quiescent VCC Supply Current
Q_LS = 1.2e-9;
Tdt = 400e-9;  % Dead time

td_on = 840e-9; % Turn-on delay time
td_off = 300e-9; % Turn-off delay time
tr = 30e-9; % Rise time
tf = 30e-9; % Fall time
Icapability = 2.5;

% Bootstrap Diode Selection
disp(' - Bootstrap diode: PMEG60T30ELR');
V_D = 0.62;
ILK_D = 1.8e-6;
V_R = 60;  % Reverse Voltage
IFSM = 60; % Non-repetitive peak forward current

% Bootstrap capacitor calculation
ILKCAP = 0;

% T_L = (D/Fsw) + Tdt;
T_Lmin = (Dmin/Fsw) + Tdt;
T_Lmax = (Dmax/Fsw) + Tdt;
% T_H = ((1-D)/Fsw) - Tdt;
T_Hmin = ((1-Dmax)/Fsw) - Tdt;


V_boot = VCC - V_D - VGS_min;
dV_boot = bs_rip * V_boot;

I_LK = ILKCAP + IGSS + IQBS + ILK + ILK_D;
% Q_CB = Qgate + (T_Lmax * I_LK) + Q_LS;
Q_CB = Qgate + (T_Hmin * I_LK) + Q_LS;
C_B = Q_CB / dV_boot;

% Aplicação de margem de segurança (30%)
C_B = 1.3*round(C_B * 10000e9) / 10000e9;  % Arredondamento para um valor padrão

disp([' - Calculated bootstrap capacitor: ', num2str(C_B*1e9), ' nF']);
C_B = max(C_B, 100e-9); % Garante um valor mínimo razoável
disp([' - *Specified bootstrap capacitor: ', num2str(C_B*1e9), ' nF <---']);

% Bootstrap Resistor Calculation
R_B_min = T_Hmin / (4 * C_B);
R_B_max = T_Hmin / (3 * C_B);

% Aplicação de margem de segurança
R_B = round(1000 * ((R_B_min + R_B_max))) / 2000;
R_B = max(R_B, 2);
disp([' - *Specified bootstrap resistor: ', num2str(R_B), ' Ohm <---']);

% Verificação do tempo de recarga (deve ser menor que THmin)
tau = R_B * C_B;
if tau > T_Hmin
    disp(' - Warning: Bootstrap RC time constant is too large. Adjust R_B or C_B.');
end


% Current Calculations
I_avg = Q_CB / T_Hmin;
disp([' - Average current on bootstrap capacitor: ', num2str(I_avg), ' A']);

I_pk = (VCC - V_D) / R_B;
disp([' - Peak current on bootstrap capacitor: ', num2str(I_pk), ' A']);

% Verificação do diodo bootstrap
if I_pk > IFSM  % PMEG60T30ELR suporta até 60A de pico
    disp(' - Warning: Peak current exceeds bootstrap diode rating! Consider a stronger diode.');
end
if V_R < (VCC + 20)  % Verificação da tensão reversa
    disp(' - Warning: Reverse voltage rating of the diode is too low!');
end

% Voltage Drop Calculations
VR_B = I_avg * R_B;
VR_Bpk = I_pk * R_B;
disp([' - Average voltage drop on bootstrap resistor: ', num2str(VR_B), ' V']);
disp([' - Peak voltage drop on bootstrap resistor: ', num2str(VR_Bpk), ' V']);

% Power Dissipation in Bootstrap Resistor
PR_B = R_B * (I_avg^2);
PR_Bpk = R_B * (I_pk^2);
disp([' - Average power dissipated by bootstrap resistor: ', num2str(PR_B), ' W']);
disp([' - Peak power dissipated by bootstrap resistor: ', num2str(PR_Bpk), ' W']);

% Ripple Voltage Verification
deltaVBoot = Qgate / C_B;
disp([' - Specified ripple voltage: ', num2str(deltaVBoot), ' V <---']);

% Cálculo dos resistores de gate-on (Rg_on) e gate-off (Rg_off)

T_SWon_off = td_on;
I_GavSw = Qgate/T_SWon_off

I_Src = 1.5*Qgate/tr
I_Snk = 1.5*Qgate/tf

I_Gavr = (SW_QGS + SW_QGD)/tr

dVout_dt = VCC / SW_Tf;  % Tempo de queda do MOSFET

Rdrv_on = VCC / 2.5  % Resistência do driver (Ω)
Rdrv_off = VCC / 2.5  % Resistência do driver (Ω)
t_sw = 100e-9; % Tempo de chaveamento desejado (100 ns)
Ig_avg = Qgate / t_sw;  % Corrente média durante a comutação
% R_total = (VCC - VGS_th) / Ig_avg  % Resistência total necessária
R_total = (VCC - VGS_th) / (SW_CGD * dVout_dt);


RG_on = R_total - Rdrv_on;
PR_on = RG_on*((I_avg)^2)

RG_off = (VGS_th / (SW_CGD * dVout_dt)) - Rdrv_off;
PR_off = RG_off*((I_avg)^2);

% Exibir resultados
disp([' - Gate-On Resistor: ', num2str(RG_on), ' (Ω)']);
disp([' - Gate-Off Resistor: ', num2str(RG_off), ' (Ω)']);

disp([' - Gate-On Resistor Average PD: ', num2str(PR_on), ' (W)']);
disp([' - Gate-Off Resistor Average PD: ', num2str(PR_off), ' (W)']);

% PCB Layout Recommendations
disp(' ');
disp('PCB Layout Recommendations:');
disp(' - Place bootstrap capacitor as close as possible to the driver IC.');
disp(' - Use a low-ESR ceramic capacitor for bootstrap capacitor.');
disp(' - Minimize parasitic inductances in the bootstrap circuit path.');
disp(' - Ensure the bootstrap diode is properly rated for reverse voltage and recovery time.');
disp(' - Consider an additional small decoupling capacitor in parallel with C_B.');
disp('Bootstrap Circuit Design Complete.');
