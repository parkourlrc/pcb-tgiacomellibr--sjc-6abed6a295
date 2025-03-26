disp(' - N-Channel Mosfet: PSMN1R4-30YLD');
Rdson = 1.85e-3;
Qgate = 60e-9;  % Total gate charge (54.8nC)
VGS_min = 4.0; %(above Fig. 9. Transfer characteristics; drain current as a function of gate-source voltage; typical values)
VGS_th = 2.2;
IGSS = 100e-9; % Gate-source leakage current
SW_Ton = 23e-9;
SW_Toff = 31.5e-9;
SW_Tr = 28e-9;
SW_Tf = 20.6e-9;
SW_QGS = 7.2e-9;
SW_QGD = 8.5e-9;
SW_CGD = 251e-12; % Reverse transfer capacitance (Crss)
C_in = 3840e-12;
C_out = 1785e-12;