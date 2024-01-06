clear all;
close all;
clc;
AdcInjectedTimerkBaseClock = 90e6;
AdcInjectedTimerkPscReg = 89;
AdcInjectedTimerkRealClock = (AdcInjectedTimerkBaseClock)/(1+AdcInjectedTimerkPscReg)
AdcInjectedTimerkRealClockPeriod_us = 1e6/AdcInjectedTimerkRealClock
AdcInjectedTimerkCountPeriod = 99;

AdcInjectedTimerkClockFreq = (AdcInjectedTimerkBaseClock)/( (AdcInjectedTimerkPscReg+1)*(AdcInjectedTimerkCountPeriod+1) )
AdcInjectedTimerkClockPeriod = 1/AdcInjectedTimerkClockFreq
