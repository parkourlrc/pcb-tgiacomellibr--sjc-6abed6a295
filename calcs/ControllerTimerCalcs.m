clear all;
close all;
clc;
ControllerTimerBaseClock = 90e6;
ControllerTimerPscReg = 89;
ControllerTimerRealClock = (ControllerTimerBaseClock)/(1+ControllerTimerPscReg)
ControllerTimerRealClockPeriod = 1/ControllerTimerRealClock;
ControllerTimerRealClockPeriod_us = 1e6*ControllerTimerRealClockPeriod;

controller_freq = 1000;
RuntimeDelay = 1*20*1e-6;
RuntimeDelay_us = 1e6*RuntimeDelay

ControllerEventPeriod = (1/controller_freq)+RuntimeDelay;
ControllerEventPeriod_ms = 1000*ControllerEventPeriod

ControllerTimerCountPeriod =  (ControllerEventPeriod/ControllerTimerRealClockPeriod)-1

ControllerTimerClockFreq = (ControllerTimerBaseClock)/( (ControllerTimerPscReg+1)*(ControllerTimerCountPeriod+1) )
ControllerTimerClockPeriod = 1/ControllerTimerClockFreq
ControllerTimerClockPeriod_us = 1e6/ControllerTimerClockFreq
