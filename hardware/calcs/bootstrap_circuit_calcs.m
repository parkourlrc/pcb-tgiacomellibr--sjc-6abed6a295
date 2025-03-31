clear all
close all
clc

%% === Design Margins ===
CB_margin      = 1.5;    % Bootstrap capacitor safety margin
ripple_margin  = 0.7;    % Ripple vs spec threshold
rc_margin      = 0.8;    % Max allowed RC time constant as fraction of T_Hmin

%% === Operating Conditions ===
VCC = 12;               % Driver supply voltage [V]
VS = 24;                % High-side supply (e.g., bus) [V]
Fsw = 20e3;             % Switching frequency [Hz]
Dmin = 2/100;           % Minimum duty cycle [%]
Dmax = 1 - Dmin;        % Maximum duty cycle [%]
D = Dmax;
bs_rip = 5/100;         % Max allowable ripple (% of V_boot)

disp('Bootstrap Circuit Calculations');

%% === MOSFET Selection ===
run('mosfet_params/IAUC100N04S6N028.m')

%% === Gate Driver Parameters ===
disp(' - Half-bridge driver: 2ED2184S06F');
ILK   = 12.5e-6;
IQBS  = 170e-6;
IQCC  = 550e-6;
Q_LS  = 1.2e-9;
Tdt   = 400e-9;

td_on = 840e-9;
td_off = 300e-9;
tr = 30e-9;
tf = 30e-9;
Icapability = 2.5;

%% === Bootstrap Diode Parameters ===
disp(' - Bootstrap diode: PMEG60T30ELR');
V_D    = 0.62;
ILK_D  = 1.8e-6;
V_R    = 60;
IFSM   = 60;
D_Ptot = 0.68;

%% === Bootstrap Capacitor Calculation ===
ILKCAP = 0;

T_Lmin = (Dmin / Fsw) + Tdt;
T_Lmax = (Dmax / Fsw) + Tdt;
T_Hmin = ((1 - Dmax) / Fsw) - Tdt;

V_boot = VCC - V_D - VGS_min;
dV_boot = bs_rip * V_boot;

I_LK = ILKCAP + IGSS + IQBS + ILK + ILK_D;
Q_CB = Qgate + (T_Hmin * I_LK) + Q_LS;
C_Bnom = Q_CB / dV_boot;
C_Braw = CB_margin * C_Bnom;

%% === Critical Low Duty Cycle Evaluation ===
D_critical = Dmin;
T_Hcrit = ((1 - D_critical)/Fsw) - Tdt;
Q_CBcrit = Qgate + (T_Hcrit * I_LK) + Q_LS;
C_Bcrit = CB_margin * Q_CBcrit / dV_boot;

% Choose the worst-case requirement
CB_target = max(C_Braw, C_Bcrit);

% Commercial ceramic capacitor values (X7R, typical for 0805/1206/1210, 50V+)
CB_commercial_values = [
    10e-9; 22e-9; 33e-9; 47e-9; 68e-9; 82e-9;
    100e-9; 150e-9; 220e-9; 330e-9; 470e-9; 680e-9;
    1e-6; 1.5e-6; 2.2e-6; 3.3e-6; 4.7e-6; 6.8e-6
    ];

% Select the smallest commercial value ≥ CB_target
idx = find(CB_commercial_values >= CB_target, 1, 'first');
if isempty(idx)
    warning('No commercial capacitor available ≥ required value. Using largest available.');
    C_B = CB_commercial_values(end);
else
    C_B = CB_commercial_values(idx);
end

% Display capacitor selection
disp([' - Calculated bootstrap capacitor (nominal): ', num2str(C_Bnom * 1e9), ' nF']);
disp([' - Calculated bootstrap capacitor (with margin): ', num2str(C_Braw * 1e9), ' nF']);
disp([' - C_B required for critical duty cycle (with margin): ', num2str(C_Bcrit * 1e9), ' nF']);
disp([' - *Selected standard bootstrap capacitor: ', num2str(C_B * 1e9), ' nF <---']);

%% === Voltage rating recommendation for C_B ===
V_CB_applied = VCC;  % Differential voltage over C_B
V_CB_required = 2 * V_CB_applied;
disp([' - C_B actual applied voltage: ', num2str(V_CB_applied), ' V']);
disp([' - Recommended C_B voltage rating: ≥ ', num2str(V_CB_required), ' V']);
if V_CB_required > 50
    disp('   → Suggest using MLCC rated for 63 V or 100 V.');
end

%% === Bootstrap Resistor Calculation ===
R_B_min = T_Hmin / (4 * C_B);
R_B_max = T_Hmin / (3 * C_B);
R_B = round(1000 * ((R_B_min + R_B_max))) / 2000;
disp([' - Calculated bootstrap resistor: ', num2str(R_B), ' Ohm']);
if (R_B < 1)
    R_B = max(R_B, 2);
elseif (R_B < 2)
    R_B = max(R_B, 3);
end
disp([' - *Specified bootstrap resistor: ', num2str(R_B), ' Ohm <---']);

%% === Ripple Verification ===
deltaVBoot = Qgate / C_B;
ripple_percent = deltaVBoot / V_boot * 100;
disp([' - Target ripple voltage (spec): ', num2str(dV_boot), ' V']);
disp([' - Resulting ripple voltage (calc): ', num2str(deltaVBoot), ' V']);
disp([' - Ripple voltage (actual): ', num2str(ripple_percent), ' %']);
if deltaVBoot > dV_boot
    disp(' - Warning: Calculated ripple exceeds the maximum allowed value!');
end

%% === RC Time Constant Check ===
tau = R_B * C_B;
if tau > rc_margin * T_Hmin
    disp([' - Warning: Bootstrap RC time constant exceeds ', num2str(rc_margin*100), '% of T_Hmin.']);
    if deltaVBoot < ripple_margin * dV_boot
        disp('   → Ripple is significantly below target. Consider reducing C_B and/or R_B to improve recharge time.');
    else
        disp('   → Consider reducing R_B to improve recharge time.');
        disp('   → Avoid reducing C_B unless ripple remains within limits.');
    end
end

%% === Current Calculations ===
I_avg = Q_CB / T_Hmin;
I_pk = (VCC - V_D) / R_B;

disp([' - Average current from bootstrap capacitor: ', num2str(I_avg), ' A']);
disp([' - Peak current from bootstrap capacitor: ', num2str(I_pk), ' A']);

%% === Diode Check ===
if I_pk > IFSM
    disp(' - Warning: Peak current exceeds bootstrap diode surge rating!');
end
if V_R < (VCC + VS)
    disp(' - Warning: Reverse voltage rating of diode may be insufficient!');
end

%% === Voltage Drops and Power Dissipation ===
VR_B = I_avg * R_B;
VR_Bpk = I_pk * R_B;
disp([' - Voltage drop across R_B (avg): ', num2str(VR_B), ' V']);
disp([' - Voltage drop across R_B (peak): ', num2str(VR_Bpk), ' V']);

PR_B = R_B * I_avg^2;
PR_Bpk = R_B * I_pk^2;
disp([' - Power dissipation on R_B (avg): ', num2str(PR_B), ' W']);
disp([' - Power dissipation on R_B (peak): ', num2str(PR_Bpk), ' W']);

% Resistor power limit check
RB_max_power = 1; % [W]
if PR_B > RB_max_power
    disp(' - Warning: Average power on R_B exceeds 1W resistor rating!');
end

P_diode_avg = V_D * I_avg;
P_diode_pk = V_D * I_pk;
disp([' - Bootstrap diode power dissipation (avg): ', num2str(P_diode_avg), ' W']);
disp([' - Bootstrap diode power dissipation (peak): ', num2str(P_diode_pk), ' W']);
if P_diode_avg > D_Ptot
    disp(' - Warning: Diode average power exceeds thermal limit!');
end

%% === Gate Resistor Calculations ===
T_SWon_off = td_on;
I_GavSw = Qgate / T_SWon_off;
I_Src = 1.5 * Qgate / tr;
I_Snk = 1.5 * Qgate / tf;
I_Gavr = (SW_QGS + SW_QGD) / tr;

% Check driver capability
if I_Src > Icapability
    disp(' - Warning: Required source current exceeds driver capability!');
end
if I_Snk > Icapability
    disp(' - Warning: Required sink current exceeds driver capability!');
end

dVout_dt = VCC / SW_Tf;
Rdrv_on  = VCC / 2.5;
Rdrv_off = VCC / 2.5;
t_sw     = 100e-9;
Ig_avg   = Qgate / t_sw;

R_total = (VCC - VGS_th) / (SW_CGD * dVout_dt);
RG_on = R_total - Rdrv_on;
RG_off = (VGS_th / (SW_CGD * dVout_dt)) - Rdrv_off;

if RG_on < 0
    disp(' - Warning: RG_on calculated negative → set to zero.');
    RG_on = 0;
end
if RG_off < 0
    disp(' - Warning: RG_off calculated negative → set to zero.');
    RG_off = 0;
end

disp([' - Gate-On resistor: ', num2str(RG_on), ' Ohm']);
disp([' - Gate-Off resistor: ', num2str(RG_off), ' Ohm']);

PR_on  = RG_on * I_avg^2;
PR_off = RG_off * I_avg^2;

disp([' - Power dissipation on RG_on: ', num2str(PR_on), ' W']);
disp([' - Power dissipation on RG_off: ', num2str(PR_off), ' W']);

%% === Driver Supply Decoupling Capacitance Suggestion ===
Cdrv = 10 * I_pk * T_Hmin / VCC;
disp([' - Suggested driver bulk capacitance: ', num2str(Cdrv * 1e6), ' uF']);

%% === PCB Layout Recommendations ===
disp(' ');
disp('PCB Layout Recommendations:');
disp(' - Place the bootstrap capacitor as close as possible to the driver IC.');
disp(' - Use a low-ESR ceramic capacitor for the bootstrap capacitor.');
disp(' - Minimize parasitic inductance in the bootstrap loop.');
disp(' - Ensure the bootstrap diode is properly rated for reverse voltage and recovery time.');
disp(' - Consider adding a small decoupling capacitor in parallel with C_B.');
disp('Bootstrap Circuit Design Complete.');
