PwmClock = 180e6;
PwmPscReg = 8;
TimCountPeriod = 999;

PwmFreq = (PwmClock)/( (PwmPscReg+1)*(TimCountPeriod+1) )


PwmFreq = 16e3
TimCountPeriod = ( ( (PwmClock)/PwmFreq )/(PwmPscReg+1) ) -1

PwmClockPer = 1/PwmClock

Dt = 50e-9

X = Dt/PwmClockPer