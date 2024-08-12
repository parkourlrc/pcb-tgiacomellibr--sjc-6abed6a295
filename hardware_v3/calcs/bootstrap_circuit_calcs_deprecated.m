clear all
close all
clc

a = sprintf('\n\r Bootstrap Circuit calcs:');
disp(a)

% L6384E alternative FAN7382MX (https://www.onsemi.com/pub/collateral/and9674-d.pdf)
ILK = 10e-6
IQBS = 100e-6

Dmin = 1/100;
Dmax = 1-Dmin;
Fsw = 20e3;

T = 1/Fsw
Tmin = Dmin*T;
Tmax = Dmax*T;
Rdson_drv = 125;
Vdd = 12;
QLS = 22e-9;

a = sprintf('\r - N-Channel Mosfet: IRLR8726');
disp(a)

% BSC0902NSI
Rdson = 2.8e-3;
Qgate = 32e-9;
VGS_min = 4.5;
ILKGS = 100e-9;
C_in = 2000e-12;
C_out = 840e-12;

% Auxiliar bootstrap diode
##Vf_bd = 0.6;
##ILK_bd = 0.65e-6;
Vf_bd = 0.65;
ILK_bd = 1.8e-6;

% Capacitor
ILKCAP = 0; %Ceramic Capacitor



a = sprintf('\r   - RDS_{On}: %.2f mOhm',Rdson*1e3);
disp(a)
a = sprintf('\r   - Q_{gate}: %.2f nF',Qgate*1e9);
disp(a)

Qg = Qgate + ((ILKCAP+ILK+IQBS+ILK_bd)*Dmax/Fsw) + (QLS/Fsw);
a = sprintf('\r   - Q_{gate}_tot: %.2f nF',Qg*1e9);
disp(a)

Rdson_drv = 125;

deltaVboot = Vdd - VGS_min - Vf_bd
Cext = Qg/deltaVboot;
Cboot_calc = 10*Cext;
Cboot = 66e-9;

tau = Rdson_drv*Cboot/0.02

a = sprintf('\r - Bootstrap circuit: ');
disp(a)
a = sprintf('\r   - Cext (nF): %.2f', Cext*1e9);
disp(a)
a = sprintf('\r   - Cboot [calc] (nF): %.2f', Cboot_calc*1e9);
disp(a)
a = sprintf('\r   - Cboot (nF): %.2f', Cboot*1e9);
disp(a)

Ipeak = Cboot*deltaVboot/Tmin
Vdrop = Qg*Rdson_drv/Tmin;
a = sprintf('Vdrop (mV): %.2f', Vdrop*1e3);
disp(a)

a = sprintf('\r - Switching parameters: ');
disp(a)

Cgate = Qgate/Vdd
Csw = max([Cgate, C_in, C_out])
a = sprintf('\r   - Mosfet Capacitange (nF): %.2f', Cext*1e9);
disp(a)
Ipeak_sw = Cgate*deltaVboot/Tmin

