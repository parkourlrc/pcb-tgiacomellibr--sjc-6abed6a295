%%
clear all
close all
clc

Fpwm = 20e3;
Tpwm = 1/Fpwm;
ADC_rate = 4e6;
Tconv_adc = 1/ADC_rate;
Adc_cycles_pwm = Tpwm/Tconv_adc;

a = sprintf('\n ADC calcs for current sense:');
disp(a)

a = sprintf('\r - T_{PWM} (us): %.2f', Tpwm*1e6);
disp(a)

a = sprintf('\r - ADC_{rate} (MS/s): %.2f', ADC_rate*1e-6);
disp(a)

a = sprintf('\r - ADC T_{conv} (us): %.2f', Tconv_adc*1e6);
disp(a)

a = sprintf('\r - ADC conversions by pwm period (us): %.2f', Adc_cycles_pwm);
disp(a)


