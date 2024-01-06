clear all;
close all;
clc;
AnalogTsCheckBaseClock = 90e6;
AnalogTsCheckPscReg = 89;
AnalogTsCheckRealClock = (AnalogTsCheckBaseClock)/(1+AnalogTsCheckPscReg)
AnalogTsCheckRealClockPeriod_us = 1e6/AnalogTsCheckRealClock
AnalogTsCheckCountPeriod = 999;

AnalogTsCheckClockFreq = (AnalogTsCheckBaseClock)/( (AnalogTsCheckPscReg+1)*(AnalogTsCheckCountPeriod+1) )
