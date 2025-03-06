clear all;
close all;
clc;

% Exibir título
disp('Bootstrap Circuit Calcs - No Adjustments');

% Constantes do MOSFET
disp('- N-Channel Mosfet: BSC0902NSI');
Rdson = 3.7e-3;
Qgate = 32e-9;
Q_G = Qgate;
VGS_min = 4.5;
VGS_th = 2;
IGSS = 100e-9;
ILKGS = IGSS;
SW_Ton = 3.9e-9;
SW_Toff = 20e-9;
SW_QGS = 4e-9;
SW_QGD = 4e-9;
SW_CGD = 88e-12;
SW_tr = 5.4e-9;  % Tempo de subida do MOSFET
SW_tf = 3.8e-9;  % Tempo de descida do MOSFET

% Constantes do Driver 2ED2184S06F
disp('- Half-bridge driver: 2ED2184S06F');
ILK = 50e-6;
IQBS = 120e-6;
IPBS = 600e-6;
Q_LS = 5e-9;
Tdt = 400e-9;

% Constantes do bootstrap
disp('- Bootstrap Components');
V_D = 0.65;
I_drv_capability = 2000e-3;
Dmin = 2/100;
Dmax = (1-Dmin);
Fsw = 20e3;
T = 1/Fsw;

% Condições de operação
disp('- Operating conditions');
VCC_range = 12:0.5:15;

% Inicialização dos arrays para armazenamento de resultados
results = [];

for VCC = VCC_range
    
    % Cálculo do tempo de condução e recarga
    T_H = max(((1-Dmax)/Fsw) - Tdt, 0);
    T_Lmax = (Dmax/Fsw) + Tdt;
    T_Lmin = max((Dmin/Fsw) + Tdt, 0);
    
    % Ajuste para a capacitância C_GD conforme a tensão de acionamento
    V_gs_operacao = (VCC + VGS_min) / 2; % Tensão média de acionamento
    C_GD_ajustado = SW_CGD * (25 / V_gs_operacao); % Ajuste baseado no datasheet
    
    % Cálculo do bootstrap
    V_boot = VCC - V_D - VGS_min;
    bs_rip = 0.05; dV_boot = bs_rip * V_boot;
    I_LK = ILKGS + IQBS + ILK;
    Q_CB = Q_G + (T_Lmax * I_LK) + Q_LS;
    C_B = Q_CB / dV_boot;
    
    % Cálculo do resistor de bootstrap RB
    R_B = T_H / (4 * C_B);
    
    % Considerando que a recarga de Cboot é pulsada
    t_recarga = T_H / 10;
    
    % Corrente de carga e potência do bootstrap
    I_pk = max(Q_CB / (3 * t_recarga), (VCC - V_D) / (3 * R_B));
    PR_B = R_B * (I_pk^2) * (T_H / (2*T)); % Ajuste para carga pulsada
    PR_Bpk = R_B * (I_pk^2);
    
    % Resistências de gate sem ajustes empíricos
    dVout_dt = 0.9e9;
    RG_on = max((VCC - VGS_th) / (C_GD_ajustado * dVout_dt) - (VCC / I_drv_capability), 0);
    RG_off = max((VGS_th) / (C_GD_ajustado * dVout_dt) - (VCC / I_drv_capability), 0.1); % Impede RG_off = 0
    
    % Tempos efetivos de chaveamento usando SW_tr e SW_tf
    t_r_used = max((RG_on + (VCC / I_drv_capability)) * C_GD_ajustado, SW_tr);
    t_f_used = max((RG_off + (VCC / I_drv_capability)) * C_GD_ajustado, SW_tf);
    
    % Potências dissipadas no gate
    PR_on = (I_pk^2) * RG_on * (T_H / (2*T));
    PR_off = (I_pk^2) * RG_off * (T_H / (2*T));
    
    
    % Cálculo dos tempos médios de chaveamento sem ajustes empíricos
    SW_on_avg = (RG_avg + VCC / I_drv_capability) * C_GD_ajustado + SW_tr;
    SW_off_avg = (RG_avg + VCC / I_drv_capability) * C_GD_ajustado + SW_tf;
    
    % Cálculo dos tempos separados sem ajuste empírico
    SW_on_separated = (RG_on + (VCC / I_drv_capability)) * C_GD_ajustado + SW_tr;
    SW_off_separated = (RG_off + (VCC / I_drv_capability)) * C_GD_ajustado + SW_tf;
    
    % Cálculo de RG_avg sem balanceamento artificial
    RG_avg = (SW_on_sep * RG_on + SW_off_sep * RG_off) / (SW_on_sep + SW_off_sep);
    
    % Potência dissipada usando RG_avg
    PR_avg = (I_pk^2) * RG_avg * (T_H / (2*T));
    
    % Diferença percentual entre SW_on e SW_off nos dois casos
    balanceamento_separado = abs(SW_on_separated - SW_off_separated) / max(SW_on_separated, SW_off_separated) * 100;
    balanceamento_avg = abs(SW_on_avg - SW_off_avg) / max(SW_on_avg, SW_off_avg) * 100;
    
    % Armazenamento dos resultados no formato correto
    results = [results; ...
        VCC, C_B*1e9, R_B, PR_B, PR_Bpk, ...
        RG_on, RG_off, RG_avg, PR_on, PR_off, PR_avg, ...
        SW_on_separated*1e9, SW_off_separated*1e9, SW_on_avg*1e9, SW_off_avg*1e9, ...
        balanceamento_separado, balanceamento_avg];
end

% Criando a tabela formatada
results_table = array2table(results, 'VariableNames', ...
    {'VCC (V)', 'Cboot (nF)', 'Rboot (Ω)', 'P_RB_avg (W)', 'P_RB_peak (W)', ...
    'RG_on (Ω)', 'RG_off (Ω)', 'RG_avg (Ω)', 'P_R_on (W)', 'P_R_off (W)', 'P_R_avg (W)', ...
    'SW_on_sep (ns)', 'SW_off_sep (ns)', 'SW_on_avg (ns)', 'SW_off_avg (ns)', ...
    'Bal_sep (%)', 'Bal_avg (%)'});

disp(results_table);
