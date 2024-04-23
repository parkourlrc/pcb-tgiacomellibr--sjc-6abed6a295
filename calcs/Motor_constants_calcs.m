%%
clear all; close all; clc

%% Joints X and L
w_rpm = 82;
Vnom = 24;

T_stall = 35;
I_stall = 10;

w_rad_sec = w_rpm*pi/30.0

Ka = Vnom/w_rad_sec

Kt = T_stall/I_stall
1/Ka