clear all
close all
clc

plot_on = 0

%% ACS 712 calcs
% ---------------------------------------------------------------------------- %
a = sprintf('\n\r ACS 712 Circuit calcs:');
disp(a)

Rfint = 1.7e3;
Cfilt = 22e-9;
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

Imeas_ACS712 = linspace(-20,20,100);

a = sprintf('\n\r - Input to 3.3V range resistive conditioner calcs:');
disp(a)
Rup = 510;
Rdown = 1.02e3;
Av_input_3v3 = Rdown/(Rup+Rdown);
a = sprintf('\r   - R_{up} (kOhm): %.2f', Rup*1e-3);
disp(a)
a = sprintf('\r   - R_{down} (kOhm): %.2f', Rdown*1e-3);
disp(a)
a = sprintf('\r   - Av (V/V): %.2f', Av_input_3v3);
disp(a)
a = sprintf('\r   - Ex. For 5V input, the output is %.2fV...', Av_input_3v3*5);
disp(a)

Av_min = min(Av_ACS712);
Av_max = max(Av_ACS712);
Av_std = Av_ACS712(2);
Vout_ACS712_min = Av_input_3v3*(Vref+Av_min*Imeas_ACS712);
Vout_ACS712_max = Av_input_3v3*(Vref+Av_max*Imeas_ACS712);
Vout_ACS712_std = Av_input_3v3*(Vref+Av_std*Imeas_ACS712);
I=((Vout_ACS712_std/Av_input_3v3)-Vref)/Av_std

if (plot_on > 0)
  figure;
  hold on;
  plot(Imeas_ACS712, Vout_ACS712_std, '-k', Imeas_ACS712, Vout_ACS712_min, '--g', Imeas_ACS712, Vout_ACS712_max, '-.r');
  legend('Std. curve', 'Min. sensibility', 'Max. sensibility')
  title('ACS 712 current transfer function')
  xlabel('I (A)');
  ylabel('Viout (V)');
endif

%% AD8418 calcs
% ---------------------------------------------------------------------------- %
a = sprintf('\n\r AC8418 Circuit calcs:');
disp(a)

Av_AD8418 = 20;
Vin = 5;
Vref = Vin/2;
Rshunt = 8e-3;
Rshunt_tol = 0.1/100; % 0.1%
Imax = 10;
Imeas_AD8418 = linspace(-Imax,Imax,100);
Rshunt_std = Rshunt;
Rshunt_min = Rshunt*(1-Rshunt_tol);
Rshunt_max = Rshunt*(1+Rshunt_tol);

a = sprintf('\n\r - Input to 3.3V range resistive conditioner calcs:');
disp(a)
Rup = 270;
Rdown = 1.02e3;
Av_input_3v3 = Rdown/(Rup+Rdown);

Av_input_3v3 = Rdown/(Rup+Rdown);
Vout_AD814_std = Av_input_3v3*((Av_AD8418.*Imeas_AD8418.*Rshunt_std) + Vref);
Vout_AD814_min = Av_input_3v3*((Av_AD8418.*Imeas_AD8418.*Rshunt_min) + Vref);
Vout_AD814_max = Av_input_3v3*((Av_AD8418.*Imeas_AD8418.*Rshunt_max) + Vref);

a = sprintf('\r   - R_{up} (kOhm): %.2f', Rup*1e-3);
disp(a)
a = sprintf('\r   - R_{down} (kOhm): %.2f', Rdown*1e-3);
disp(a)
a = sprintf('\r   - Av (V/V): %.2f', Av_input_3v3);
disp(a)
a = sprintf('\r   - Ex. For %.3f A (Min input) @ %.2f V, the output is %.2fV...', min(Imeas_AD8418), min(Vout_AD814_std)/Av_input_3v3, min(Vout_AD814_std));
disp(a)
a = sprintf('\r   - Ex. For %.3f A (Max input) @ %.2f V, the output is %.2fV...', max(Imeas_AD8418), max(Vout_AD814_std)/Av_input_3v3, max(Vout_AD814_std));
disp(a)

Vadc = Vout_AD814_std;
I_adc_meas = (([min(Vadc) max(Vadc)]./Av_input_3v3)-Vref)/(Rshunt_std*Av_AD8418); % Equation to recover current from adc measurement

Pshunt_mW = (max(Imeas_AD8418)^2)*Rshunt_max*1000;
a = sprintf('\n\r - Rshunt power rating (mW): %.2f', Pshunt_mW);
disp(a)
if (plot_on > 0)
  figure;
  plot(Imeas_AD8418, Vout_AD814_std, '-k', Imeas_AD8418, Vout_AD814_min, '--g', Imeas_AD8418, Vout_AD814_max, '-.r');
  legend('Std. curve', 'Min. shunt tol.', 'Max. shunt tol.')
  title('AD8418 current transfer function')
  xlabel('I (A)');
  ylabel('Viout (V)');
endif

%% Frequency filter
% (2nd order + 1st order) Butterworth filter
R = 1.5e3;
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


% Theoretical cals
I = 5.305
duty = [30 70]

duty_pos = (duty./100);
duty_neg = 1-duty_pos;
I_pos = (I*duty_pos);
I_neg = (-I*duty_neg);

V_adc_pos = Av_input_3v3.*((I_pos.*Av_AD8418.*Rshunt_std)+Vref).*duty_pos;
V_adc_neg = Av_input_3v3.*((I_neg.*Av_AD8418.*Rshunt_std)+Vref).*duty_neg;

I_avg = I_pos + I_neg
V_adc_avg = V_adc_pos + V_adc_neg

I_adc_meas = ((V_adc_avg./Av_input_3v3)-Vref)./(Rshunt_std*Av_AD8418)



