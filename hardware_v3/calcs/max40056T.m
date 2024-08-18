clear all
close all
clc


disp('------------------------------------------------------------------------')
disp('Max40056T Specs:')
disp(' ')

VDD = 3.3
VREF = VDD/2
R_SENSE = 2e-3
GAIN = 20
I_SENSE = -40:0.001:40;
VOUT = ( (I_SENSE*R_SENSE)*GAIN ) + VREF;
disp('------------------------------------------------------------------------')
disp(' ')
disp('------------------------------------------------------------------------')
disp('Current Limit Specs:')
disp(' ')


##IVALUE = (VOUT-VREF)/(GAIN*R_SENSE);

I_LIMIT = 33
Rup = 10.0e3
Rdown = 1.1e3
V_div = VDD*Rdown/(Rup+Rdown)
I_LIMIT = abs((V_div-VREF)/(GAIN*R_SENSE))

MIN_IN_OFFSET = 0.05;
VOUT_LIM_X_AXIS = [min(I_SENSE) max(I_SENSE)];

VOUT_UPPER_LIM = ( (I_LIMIT*R_SENSE)*GAIN ) + VREF;
VOUT_LOWER_LIM = ( (-I_LIMIT*R_SENSE)*GAIN ) + VREF;
VOUT_UPPER_LIM_CUR = (VOUT_UPPER_LIM-VREF)/(GAIN*R_SENSE);
VOUT_LOWER_LIM_CUR = (VOUT_LOWER_LIM-VREF)/(GAIN*R_SENSE);

figure(1); plot(I_SENSE, VOUT, '--k', [VOUT_LOWER_LIM_CUR, VOUT_UPPER_LIM_CUR], [VOUT_LOWER_LIM, VOUT_UPPER_LIM], '-xg'); hold;
stem([VOUT_LOWER_LIM_CUR, VOUT_UPPER_LIM_CUR], [VOUT_LOWER_LIM, VOUT_UPPER_LIM], '-r'); grid on;
legend('Full range', 'Allowed range', 'Limits');
xlabel('Input current [A]');
ylabel('Output voltage [V]');
title('MAX40056T IxV');
disp(' ')
disp('------------------------------------------------------------------------')
disp('Comparator Specs:')
disp(' ')
VCIP_1 = (VREF - (80/1000) );
VCIP_2 = (VDD - 1.25);
VCIP_RANGE = [80/1000 min(VCIP_1, VCIP_2)]
VCIP = max(min(VCIP_RANGE),VOUT_LOWER_LIM)
disp(' ')
VA_RANGE = [(VREF + (80/1000)) ((2*VREF)-(80/1000))]
VA = (2*VREF) - VCIP
disp(' ')
disp('------------------------------------------------------------------------')
disp('Comparator Limits:')
disp(' ')
VA_CURRENT = (VA-VREF)/(GAIN*R_SENSE)
VCIP_CURRENT = (VCIP-VREF)/(GAIN*R_SENSE)
