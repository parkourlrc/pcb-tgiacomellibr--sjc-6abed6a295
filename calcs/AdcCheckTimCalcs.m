# TIM4

clear all;
close all;
clc;
AdcSampleCheckTimerBaseClock = 90e6;
AdcSampleCheckTimerPscReg = 89;
AdcSampleCheckTimerRealClock = (AdcSampleCheckTimerBaseClock)/(1+AdcSampleCheckTimerPscReg)
AdcSampleCheckTimerRealClockPeriod = 1/AdcSampleCheckTimerRealClock;
AdcSampleCheckTimerRealClockPeriod_us = 1e6*AdcSampleCheckTimerRealClockPeriod;

AdcSampleCheck_freq = 50;

AdcSampleCheckEventPeriod = (1/AdcSampleCheck_freq);
AdcSampleCheckEventPeriod_ms = 1000*AdcSampleCheckEventPeriod

AdcSampleCheckTimerCountPeriod = (AdcSampleCheckEventPeriod/AdcSampleCheckTimerRealClockPeriod)-1

AdcSampleCheckTimerClockFreq = (AdcSampleCheckTimerBaseClock)/( (AdcSampleCheckTimerPscReg+1)*(AdcSampleCheckTimerCountPeriod+1) )
AdcSampleCheckTimerClockPeriod = 1/AdcSampleCheckTimerClockFreq
AdcSampleCheckTimerClockPeriod_ms = 1e3/AdcSampleCheckTimerClockFreq
AdcSampleCheckTimerClockPeriod_us = 1e6/AdcSampleCheckTimerClockFreq