clear all;
close all;
clc;

PwmTimerBaseClock = 180e6;
PwmTimerPscReg = 8;
TimCountPeriod = 999;

PwmTimerRealClock = (PwmTimerBaseClock)/(1+PwmTimerPscReg);
PwmTimerRealClockPeriod = 1/PwmTimerRealClock;
PwmTimerRealClockPeriod_us = 1e6*PwmTimerRealClockPeriod

PwmFreq = (PwmTimerBaseClock)/( (PwmTimerPscReg+1)*(TimCountPeriod+1) )
PwmPeriod = 1/PwmFreq

PwmFreq = 20e3
TimCountPeriod = ( ( (PwmTimerBaseClock)/PwmFreq )/(PwmTimerPscReg+1) ) -1