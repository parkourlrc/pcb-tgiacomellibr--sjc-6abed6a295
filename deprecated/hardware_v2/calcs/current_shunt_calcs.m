clear all
close all
clc

plot_on = 1

%% ACS 712 calcs
% ---------------------------------------------------------------------------- %
a = sprintf('\n\r ACS 712 Circuit calcs:');
disp(a)

Rfint = 1.7e3;
Cfilt = 68e-9;
Fc = 1/(2*pi*Rfint*Cfilt);
Av_ACS712 = [96 100 104]*1e-3;
Vcc_ACS712 = 5;
Vref = 0.5*Vcc_ACS712;
a = sprintf('\r - Low-pass filter resistor (kOhm): %.2f', Rfint*1e-3);
disp(a)
a = sprintf('\r - Low-pass filter capacitor (nF): %.2f', Cfilt*1e9);
disp(a)
a = sprintf('\r - Low-pass filter cutoff frequency (kHz): %.2f', Fc*1e-3);
disp(a)

Imeas = 25;
Imeas_ACS712 = linspace(-Imeas, Imeas, 100);

a = sprintf('\n\r - Input to 3.3V range resistive conditioner calcs:');
disp(a)
Rup = 560;
Rdown = 1.1e3;
Av_input_3v3 = Rdown/(Rup+Rdown);
a = sprintf('\r   - R_{up} (kOhm): %.2f', Rup*1e-3);
disp(a)
a = sprintf('\r   - R_{down} (kOhm): %.2f', Rdown*1e-3);
disp(a)
a = sprintf('\r   - Av (V/V): %.4f', Av_input_3v3);
disp(a)
a = sprintf('\r   - Ex. For 5V input, the output is %.2fV...', Av_input_3v3*5);
disp(a)

Av_min = min(Av_ACS712);
Av_max = max(Av_ACS712);
Av_std = Av_ACS712(2);
Vout_ACS712_min = Av_input_3v3*(Vref+Av_min*Imeas_ACS712);
Vout_ACS712_max = Av_input_3v3*(Vref+Av_max*Imeas_ACS712);
Vout_ACS712_std = Av_input_3v3*(Vref+Av_std*Imeas_ACS712);

I=((Vout_ACS712_std/Av_input_3v3)-Vref)/Av_std;

a = sprintf('\r   - Ex. For %.3f A (Min input) @ %.3f V, the output is %.3fV...', min(Imeas_ACS712), min(Vout_ACS712_std)/Av_input_3v3, min(Vout_ACS712_std));
disp(a)
a = sprintf('\r   - Ex. For %.3f A (Max input) @ %.3f V, the output is %.3fV...', max(Imeas_ACS712), max(Vout_ACS712_std)/Av_input_3v3, max(Vout_ACS712_std));
disp(a)

if (plot_on > 0)
  figure;
  hold on;
  plot(Imeas_ACS712, Vout_ACS712_std, '-k', Imeas_ACS712, Vout_ACS712_min, '--g', Imeas_ACS712, Vout_ACS712_max, '-.r');
  legend('Std. curve', 'Min. sensibility', 'Max. sensibility')
  title('ACS 712 current transfer function')
  xlabel('I (A)');
  ylabel('Viout (V)');
endif



%% INA240 calcs
% ---------------------------------------------------------------------------- %
a = sprintf('\n\r INA240 Circuit calcs:');
disp(a)


% mode:
% - 0 - 16A
% - 1 - 25A

mode = 1
Vin = 5;

if mode == 1
  Vref = Vin/2;
  Rshunt = 2e-3;
  Imax = 25;
else % mode = 0
  Vref = Vin/2;
  Rshunt = 3e-3;
  Imax = 16.667;
end

Av_INA240 = 50;
Rshunt_tol = 0.1/100; % 0.1%
Imeas_INA240 = linspace(-Imax,Imax,100);
RshuntValues = Rshunt*[(1-Rshunt_tol) 1 (1+Rshunt_tol)];

a = sprintf('\n\r - Input to 3.3V range resistive conditioner calcs:');
disp(a)
Rup = 560;
Rdown = 1.1e3;

Av_input_3v3 = Rdown/(Rup+Rdown);
Vout_INA240(1,:) = Av_input_3v3.*((Av_INA240.*Imeas_INA240.*RshuntValues(1)) + Vref);
Vout_INA240(2,:) = Av_input_3v3.*((Av_INA240.*Imeas_INA240.*RshuntValues(2)) + Vref);
Vout_INA240(3,:) = Av_input_3v3.*((Av_INA240.*Imeas_INA240.*RshuntValues(3)) + Vref);

a = sprintf('\r   - R_{up} (kOhm): %.2f', Rup*1e-3);
disp(a)
a = sprintf('\r   - R_{down} (kOhm): %.2f', Rdown*1e-3);
disp(a)
a = sprintf('\r   - Av (V/V): %.4f', Av_input_3v3);
disp(a)
a = sprintf('\r   - Ex. For %.3f A (Min input) @ %.3f V, the output is %.3fV...', min(Imeas_INA240), min(Vout_INA240(2,:))/Av_input_3v3, min(Vout_INA240(2,:)));
disp(a)
a = sprintf('\r   - Ex. For %.3f A (Max input) @ %.3f V, the output is %.3fV...', max(Imeas_INA240), max(Vout_INA240(2,:))/Av_input_3v3, max(Vout_INA240(2,:)));
disp(a)

Vadc = Vout_INA240(2);
I_adc_meas = (([min(Vadc) max(Vadc)]./Av_input_3v3)-Vref)/(RshuntValues(2)*Av_INA240); % Equation to recover current from adc measurement

Pshunt_mW = (max(Imeas_INA240)^2)*RshuntValues(3)*1000;
a = sprintf('\n\r - Rshunt power rating (mW): %.2f', Pshunt_mW);
disp(a)

if (plot_on > 0)
  figure;
  plot(Imeas_INA240, Vout_INA240(2,:), '-k', Imeas_INA240, Vout_INA240(1,:), '--g', Imeas_INA240, Vout_INA240(3,:), '-.r');
  legend('Std. curve', 'Min. shunt tol.', 'Max. shunt tol.')
  title('INA240 current transfer function')
  xlabel('I (A)');
  ylabel('Viout (V)');
endif

%% Frequency filter
% (2nd order + 1st order) Butterworth filter
R = 1.0e3;
C = 100e-9;
Fc = 1/(2*pi*R*C);

a = sprintf('\n\r Analog frontend calcs:');
disp(a)
a = sprintf('\r - Low-pass filter resistor (kOhm): %.2f', R*1e-3);
disp(a)
a = sprintf('\r - Low-pass filter capacitor (nF): %.2f', C*1e9);
disp(a)
a = sprintf('\r - Low-pass filter cutoff frequency (kHz): %.2f', Fc*1e-3);
disp(a)


