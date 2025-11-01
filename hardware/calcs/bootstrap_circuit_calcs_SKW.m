clear all
close all
clc

a = sprintf('\n\r Bootstrap Circuit calcs:');
disp(a)

% N-Channel Mosfet: BSC0902NSI
Rdson = 3.7e-3; % 2.8e-3/3.7e-3 @ 10Vgs/4.5Vgs;
Qgate = 32e-9;
Q_G = Qgate;
VGS_min = 4.5;
VGS_th = 2; % Gate threshold voltage VGS
IGSS = 100e-9; % Gate-source leakage current
ILKGS = IGSS;

% Driver: FAN7382MX (https://www.onsemi.com/pub/collateral/and9674-d.pdf)
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

% Auxiliar bootstrap diode
V_D = 0.6;
ILK_D = 0.65e-6;
V_R = 60;

% Bootstrap capacitor
ILKCAP = 0; % Ceramic capacitor

% Operating conditions
VCC = 15;
Fsw = 20e3;
T = 1/Fsw;

D = 95/100;    % Percent value
Dmin = 2/100;  %  1 %
Dmax = (1-Dmin); % 99 %

T_H = ( (1-D)/Fsw ) - Tdt; % On time of Q2
T_L = (D/Fsw) + Tdt; % Off time of Q2

T_Hmin = ( (1-Dmax)/Fsw ) - Tdt % Minimum time Q2 is on
T_Lmin = (Dmin/Fsw) + Tdt % Minimum time Q2 is off
%T_Lmax = (D/Fsw) + Tdt; % Maximum time Q2 is off
T_Lmax = (Dmax/Fsw) + Tdt % Maximum time Q2 is off

bs_rip = 10/100; % Maximum allowable ripple
dV_CB = bs_rip*VCC;

Q_CB = Q_G + (T_Lmax*I_B)
C_B = Q_CB/dV_CB
C_B = round(C_B*2e9)/2e9
dV = Q_CB/C_B

R_B = T_Hmin/(4*C_B)
R_B = round(10*R_B)/10
I_avg = Q_CB/T_Hmin
I_pk = (VCC-V_D)/R_B
PR_B = R_B*(I_pk^2)
dV_Boot = Q_G/C_B
