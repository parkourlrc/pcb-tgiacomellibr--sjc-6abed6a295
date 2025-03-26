disp(' - N-Channel Mosfet: PSMN3R3-40MLH');
Rdson = 7.2e-3;
Qgate = 60e-9;  % Total gate charge (54.8nC)
VGS_min = 3.5; %(above Fig. 9. Transfer characteristics; drain current as a function of gate-source voltage; typical values)
VGS_th = 2.15;
IGSS = 100e-9; % Gate-source leakage current
SW_Ton = 16e-9;
SW_Toff = 17e-9;
SW_Tr = 19e-9;
SW_Tf = 11e-9;
SW_QGS = 10.2e-9;
SW_QGD = 8e-9;
SW_CGD = 222e-12; % Reverse transfer capacitance (Crss)
C_in = 3794e-12;
C_out = 877e-12;