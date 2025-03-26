disp(' - N-Channel Mosfet: IAUC100N04S6N028');
IGSS = 100e-9; % Gate-source leakage current
Rdson = 3.9e-3;
Qgate = 35e-9;  % Total gate charge (29nC @ 10V)
VGS_min = 5; %(above Fig. 9. Transfer characteristics; drain current as a function of gate-source voltage; typical values)
VGS_th = 3;
SW_Ton = 4e-9;
SW_Toff = 9e-9;
SW_Tr = 2e-9;
SW_Tf = 5e-9;
SW_QGS = 8.4e-9;
SW_QGD = 7.2e-9;
SW_CGD = 37e-12; % Reverse transfer capacitance (Crss)
C_in = 1781e-12;
C_out = 558e-12;