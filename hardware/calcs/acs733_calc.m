Ifault = 31 % 32 A
Vcc = 3.3
Ipr_max = 40

V_voc = (Vcc/5)*Ifault/Ipr_max

Rup = 8.2e3
Rdown = 1.5e3

Voc = Vcc*Rdown/(Rup+Rdown)


Ifault_eff = Ipr_max*5*Voc/Vcc