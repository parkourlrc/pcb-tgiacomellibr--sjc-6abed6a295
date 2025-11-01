clear all
close all
clc

Vfwd = [1.1 1.25 1.4]
Ifwd = 5e-3
Vin = [9 30]
Rled = ( min(Vin)-max(Vfwd) )/Ifwd
Rled = round(Rled/100)*100
Vrled_max = (max(Vin)-min(Vfwd))
Prled = (Vrled_max^2)/Rled

Vin_range = linspace(5, 30, 100);
Prled_max = ((Vin_range - min(Vfwd)).^2)/Rled;
Iled_max = (Vin_range - min(Vfwd))/Rled;
figure;
plot(Vin_range, Prled_max*1000, '-or'); grid on;
xlabel('Vin (V)')
ylabel('P_{LED} (mW)')
figure;
plot(Vin_range, Iled_max*1000, '-or'); grid on;
xlabel('Vin (V)')
ylabel('I_{LED} (mA)')