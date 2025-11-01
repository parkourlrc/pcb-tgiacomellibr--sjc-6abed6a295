clear all
close all
clc

a = sprintf('\n\r Bootstrap Circuit calcs:');
disp(a)

% N-Channel Mosfet: BSC0902NSI
a = sprintf('\r - N-Channel Mosfet: %s', 'BSC0902NSI');
disp(a)
Rdson = 3.7e-3; % 2.8e-3/3.7e-3 @ 10Vgs/4.5Vgs;
Qgate = 32e-9;
Q_G = Qgate;
VGS_min = 4.5;
VGS_th = 2; % Gate threshold voltage VGS
IGSS = 100e-9; % Gate-source leakage current
ILKGS = IGSS;
SW_Ton = 3.9e-9;
SW_Toff = 20e-9;
SW_Tr = 5.4e-9;
SW_Tf = 3.8e-9;
SW_QGS = 4e-9;  % Gate to source charge
SW_QGD = 4e-9;  % Gate to drain charge
SW_CGD = 88e-12; % Reverse transfer capacitance

% Driver: FAN7382MX (https://www.onsemi.com/pub/collateral/and9674-d.pdf)
a = sprintf('\r - Half-bridge driver: %s', 'FAN7382MX');
disp(a)

ILK = 50e-6; % Offset Supply Leakage Current
IQBS = 120e-6; % Quiescent VBS Supply Current
IQCC = 180e-6; % Quiescent VCC Supply Current
IPBS = 600e-6; % Operating VBS Supply Current
IPCC = 600e-6; % Operating VCC Supply Current
Q_LS = 5e-9;
Tdt = 50e-9; % Dead time
I_B = IPBS; % B is the maximum quiescent
            % supply current for the
            % high side driver

C_in = 2000e-12; % Input capacitance
C_out = 840e-12; % Output capacitance
td_on = 300e-9; % Turn-on delay time
td_off = 300e-9; % Turn-off delay time
tr = 140e-9; % Rise time
tf = 80e-9; % Fall time

% Bootstrap diode: PMEG60T30ELR
a = sprintf('\r - Bootstrap diode: %s', 'PMEG60T20ELR');
disp(a)
V_D = 0.62;
ILK_D = 1.8e-6;
V_R = 60;

% Bootstrap capacitor
ILKCAP = 0; % Ceramic capacitor

% Operating conditions
a = sprintf('\r - %s: %s', 'Operating conditions');
disp(a)
VCC = 15;
Fsw = 20e3;
T = 1/Fsw;

a = sprintf('\r   - Vcc: %.2f', VCC);
disp(a)
a = sprintf('\r   - Swithing frequency: %.2f', Fsw);
disp(a)

D = 95/100;      % Percent value
Dmin = 1.5/100;    %  1 %
Dmax = (1-Dmin); % 99 %
bs_rip = 10/100; % Maximum allowable ripple
a = sprintf('\r   - Min. duty cycle: %.2f(%%)', Dmin*100);
disp(a)
a = sprintf('\r   - Maximum allowable ripple: %.2f(V)', bs_rip);
disp(a)

T_H = ( (1-D)/Fsw ) - Tdt; % On time of Q2
T_L = (D/Fsw) + Tdt; % Off time of Q2

T_Hmin = ( (1-Dmax)/Fsw ) - Tdt; % Minimum time Q2 is on
T_Lmin = (Dmin/Fsw) + Tdt; % Minimum time Q2 is off
%T_Lmax = (D/Fsw) + Tdt; % Maximum time Q2 is off
T_Lmax = (Dmax/Fsw) + Tdt; % Maximum time Q2 is off


V_boot = VCC - V_D - VGS_min;
dV_boot = bs_rip*V_boot;

I_LK = ILKCAP + ILKGS + IQBS + ILK + ILK_D;
Q_CB = Q_G + (T_Lmax*I_LK) + Q_LS;
C_B = Q_CB/dV_boot;
a = sprintf('\r   - Calculated bootstrap capacitor: %.2f(nF)', C_B*1e9);
disp(a)
C_B = round(1.20*C_B*2e9)/2e9;
%C_B = 68e-9
%C_B = 82e-9
a = sprintf('\r   - *Specified bootstrap capacitor: %.2f(nF) <---', C_B*1e9);
disp(a)
dV = Q_CB/C_B;

R_B = T_Hmin/(4*C_B);
a = sprintf('\r   - Calculated bootstrap resistor: %.2f(Ohm)', R_B);
disp(a)
R_B = round(10*R_B)/10;
a = sprintf('\r   - *Specified bootstrap resistor: %.2f(Ohm) <---', R_B);
disp(a)
%R_B = 2
I_avg = Q_CB/T_Hmin;
a = sprintf('\r   - Average current on bootstrap capacitor: %.2f(A)', I_avg);
disp(a)
I_pk = (VCC-V_D)/R_B;
a = sprintf('\r   - Peak current on bootstrap capacitor: %.2f(A)', I_pk);
disp(a)
VR_B = I_avg*R_B;
a = sprintf('\r   - Average voltage drop on bootstrap resistor: %.2f(V)', VR_B);
disp(a)
VR_Bpk = I_pk*R_B;
a = sprintf('\r   - Average voltage drop on bootstrap resistor: %.2f(V)', VR_Bpk);
disp(a)
PR_B = R_B*(I_avg^2);
a = sprintf('\r   - Average power dissipated by bootstrap resistor: %.2f(W)', PR_B);
disp(a)
PR_Bpk = R_B*(I_pk^2);
a = sprintf('\r   - Peak power dissipated by bootstrap resistor: %.2f(W)', PR_Bpk);
disp(a)
deltaVBoot = Q_G/C_B;
tau = R_B*C_B/Dmin;
a = sprintf('\r   - Specified ripple voltage: %.2f(V)  <---', deltaVBoot);
disp(a)
a = sprintf('\r   - Bootstrap network time constant: %.2f(s)  <---', tau);
disp(a)

a = sprintf('\r   - Total bootstrap gate charge: %.2f(nC)', Q_CB*1e9);
disp(a)

a = sprintf('\r   - DeltaVb: %.2f(V)', deltaVBoot);
disp(a)



% Drive capability

T_SWon_off = td_on
I_GavSw = Q_G/T_SWon_off

I_Src = 1.5*Q_G/tr
I_Snk = 1.5*Q_G/tf

I_Gavr = (SW_QGS + SW_QGD)/tr
dVout_dt = 0.9e9;
VGS_th_min = VGS_th
R_Tot = (VCC-VGS_th_min)/(SW_CGD*dVout_dt)

% On state gate resistor
Rdrv_on = VCC/350e-3
RG_on = R_Tot - Rdrv_on
PR_on = RG_on*((I_avg)^2)
% Off state gate resistor
Rdrv_off = VCC/650e-3
RG_off = (VGS_th_min/(SW_CGD*dVout_dt)) - Rdrv_off
PR_off = RG_off*((I_avg)^2)
