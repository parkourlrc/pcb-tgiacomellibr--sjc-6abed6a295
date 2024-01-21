clear all;
close all; clc

MBTimerDiv = 1;
MBTimerClock = 90e6
MBTimerPer = 1/MBTimerClock
MBDesiredPer = 50e-6;
MBTimerPscReg = (MBDesiredPer/MBTimerPer)-1
TimCountPeriod = 0

MBTimerFreq = (MBTimerClock)/( (MBTimerPscReg+1)*(TimCountPeriod+1) )

MBTimerMs = 0.1e-3
MBTimerFreq = 1/MBTimerMs
TimCountPeriod = ( ( (MBTimerClock)/MBTimerFreq )/(MBTimerPscReg+1) ) -1

NETWORK_BAUD_RATE = 9600;

Period = ceil(3.5 * 220000/NETWORK_BAUD_RATE)