% LMR23625CFDDAR Passive Component Calculator (SI Base Units - Clean Version)
clc; clear;

%% === INPUT PARAMETERS ===
Vin_max = 30;               % V
Vout = 12;                  % V
Iout_nom = 1.5;             % A
fsw = 2.1e6;                % Hz
K_ind = 0.2;                % 20% ripple current factor
Vref = 1.0;                 % V
f_z = 5e3;                  % Hz

%% === DESIGN CONSTRAINTS ===
vout_ripple_max = 0.025 * Vout;      % V (max ripple)
vout_ripple_target = 0.02 * Vout;    % V (target ripple for Cout sizing)

%% === SAFETY MARGINS ===
L_margin = 1.30;             % 30%
C_margin = 2.00;             % 100%

fprintf('=== LMR23625 PASSIVE COMPONENT SPECIFICATION ===\n');

%% === INDUCTOR SELECTION ===
IL_ripple_target = K_ind * Iout_nom;                                          % A
L_min = (Vout * (Vin_max - Vout)) / (Vin_max * fsw * IL_ripple_target);      % H
L_spec = L_min * L_margin;                                                   % H
L_sel = ceil(L_spec / 0.1e-6) * 0.1e-6;                                       % H, rounded to 0.1 µH
IL_ripple_actual = (Vout * (Vin_max - Vout)) / (Vin_max * fsw * L_spec);     % A
IL_peak = Iout_nom + IL_ripple_actual / 2;                                   % A

fprintf('\n--- INDUCTOR ---\n');
fprintf('Calculated minimum inductance: %.2f µH\n', L_min * 1e6);
fprintf('Specified inductance (+30%% margin): %.2f µH\n', L_spec * 1e6);
fprintf('Selected commercial inductance: %.1f µH\n', L_sel * 1e6);
fprintf('Actual ripple current: %.0f mA (%.1f%% of Iout)\n', IL_ripple_actual * 1e3, 100 * IL_ripple_actual / Iout_nom);
fprintf('Inductor peak current: %.2f A\n', IL_peak);
fprintf('Required specification for inductor:\n');
fprintf(' • Inductance: ≥ %.1f µH\n', L_sel * 1e6);
fprintf(' • Saturation current (Isat): ≥ %.2f A\n', IL_peak * 1.2);
fprintf(' • DC resistance (DCR): as low as possible (e.g., < 50 mΩ)\n');
fprintf(' • Core material: ferrite or low-loss composite\n');
fprintf(' • Frequency optimization: ≥ %.1f MHz\n', fsw / 1e6);

%% === OUTPUT CAPACITOR SELECTION ===
C_ripple = IL_ripple_actual / (8 * fsw * vout_ripple_target);               % F
C_transient = 4 * Iout_nom / (fsw * 0.05 * Vout);                            % F
C_min = max(C_ripple, C_transient);                                         % F
C_spec = C_min * C_margin;                                                  % F
C_sel = ceil(C_spec / 4.7e-6) * 4.7e-6;                                      % F, rounded to multiple of 4.7 µF
vout_ripple_actual = IL_ripple_actual / (8 * fsw * C_sel);                  % V

fprintf('\n--- OUTPUT CAPACITOR ---\n');
fprintf('Ripple voltage target: %.0f mV (%.1f%% of %.1f V)\n', vout_ripple_target * 1e3, ...
        100 * vout_ripple_target / Vout, Vout);
fprintf('Calculated minimum Cout:\n');
fprintf(' • Ripple criterion: %.2f µF\n', C_ripple * 1e6);
fprintf(' • Transient criterion: %.2f µF\n', C_transient * 1e6);
fprintf('Specified (100%% margin): %.2f µF\n', C_spec * 1e6);
fprintf('Selected commercial capacitance: %.1f µF\n', C_sel * 1e6);
fprintf('Expected ripple with selected C: %.2f mV\n', vout_ripple_actual * 1e3);
fprintf('Required specification for output capacitor:\n');
fprintf(' • Capacitance: ≥ %.1f µF\n', C_sel * 1e6);
fprintf(' • Voltage rating: ≥ %.1f V\n', Vout * 1.5);
fprintf(' • Dielectric: X7R or X5R\n');
fprintf(' • ESR: ≤ 20 mΩ\n');
fprintf(' • ESL: ≤ 1 nH\n');
fprintf(' • Temperature rating: ≥ 85°C\n');

%% === FEEDBACK RESISTOR DIVIDER ===
R_FB_B = 10e3;                                                           % Ω
R_FB_T = R_FB_B * ((Vout / Vref) - 1);                                   % Ω
R_FB_T_real = round(R_FB_T / 1e3) * 1e3;                                 % Ω

fprintf('\n--- FEEDBACK RESISTOR DIVIDER ---\n');
fprintf('Calculated R_FB_T: %.1f kΩ\n', R_FB_T / 1e3);
fprintf('Rounded commercial value: %.0f Ω\n', R_FB_T_real);
fprintf('Fixed R_FB_B: %.0f Ω\n', R_FB_B);

%% === FEEDFORWARD CAPACITOR ===
Cff = 1 / (2 * pi * R_FB_T_real * f_z);                                  % F

fprintf('\n--- FEEDFORWARD CAPACITOR (Cff) ---\n');
fprintf('Calculated Cff for %.0f Hz zero: %.1f pF\n', f_z, Cff * 1e12);
fprintf('Specification:\n');
fprintf(' • Dielectric: C0G/NP0 or X7R\n');
fprintf(' • Voltage rating: ≥ 16 V\n');
fprintf(' • Tolerance: ±10%% or better\n');

%% === INPUT CAPACITOR ===
D = Vout / Vin_max;                                                     % Duty cycle
vin_ripple_target = 0.01 * Vin_max;                                     % V
Cin = Iout_nom * D * (1 - D) / (fsw * vin_ripple_target);               % F
Cin_spec = Cin * 1.5;                                                   % F
Cin_sel = ceil(Cin_spec / 1e-6) * 1e-6;                                 % F

fprintf('\n--- INPUT CAPACITOR (Cin) ---\n');
fprintf('Calculated minimum Cin: %.2f µF\n', Cin * 1e6);
fprintf('Specified Cin (+50%% margin): %.2f µF\n', Cin_spec * 1e6);
fprintf('Selected commercial value: %.0f µF\n', Cin_sel * 1e6);
fprintf('Required specification for input capacitor:\n');
fprintf(' • Capacitance: ≥ %.0f µF\n', Cin_sel * 1e6);
fprintf(' • Voltage rating: ≥ %.0f V\n', Vin_max * 1.25);
fprintf(' • Dielectric: X7R or X5R\n');
fprintf(' • ESR: ≤ 20 mΩ\n');
fprintf(' • Placement: close to VIN and GND pins\n');

%% === BOOTSTRAP CAPACITOR ===
Qg = 10e-9;                         % C
Vboot_drop = 0.1;                  % V
Cboot = Qg / Vboot_drop;           % F
Cboot_spec = Cboot * 2;            % F
Cboot_sel = ceil(Cboot_spec / 0.1e-6) * 0.1e-6;  % F

fprintf('\n--- BOOTSTRAP CAPACITOR (Cboot) ---\n');
fprintf('Calculated minimum Cboot: %.2f µF\n', Cboot * 1e6);
fprintf('Specified (+100%% margin): %.2f µF\n', Cboot_spec * 1e6);
fprintf('Selected commercial value: %.1f µF\n', Cboot_sel * 1e6);
fprintf('Required specification for bootstrap capacitor:\n');
fprintf(' • Capacitance: ≥ %.1f µF\n', Cboot_sel * 1e6);
fprintf(' • Dielectric: X7R\n');
fprintf(' • Voltage rating: ≥ 16 V\n');
fprintf(' • Typical range: 0.1 µF to 0.47 µF\n');

fprintf('\n=== END OF SPECIFICATION ===\n');
