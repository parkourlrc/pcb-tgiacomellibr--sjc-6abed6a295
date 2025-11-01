# TIM6

clear all;
close all;
clc;
StatsTimerDiv = 1;
StatsTimerClock = 90e6
StatsTimerPer = 1/StatsTimerClock
StatsDesiredPer = 1e-5;
StatsTimerPscReg = (StatsDesiredPer/StatsTimerPer)-1
00.1*10e3